#include "UBotMultirotorDynamics.h"

#include "UBotMultirotorMath.h"

FUBotMultirotorAirframe::FUBotMultirotorAirframe()
{
    Rotors = UBot::Multirotor::MakeQuadXRotors(0.225);
}

namespace UBot::Multirotor
{
    namespace
    {
        double SpinSign(EUBotRotorSpinDirection Direction)
        {
            return Direction == EUBotRotorSpinDirection::Clockwise ? 1.0 : -1.0;
        }

        // Inverts a 4x4 matrix (row-major) by Gauss-Jordan elimination with partial pivoting.
        bool Invert4(const double In[16], double Out[16])
        {
            double A[4][8];
            for (int32 Row = 0; Row < 4; ++Row)
            {
                for (int32 Col = 0; Col < 4; ++Col)
                {
                    A[Row][Col] = In[Row * 4 + Col];
                    A[Row][Col + 4] = Row == Col ? 1.0 : 0.0;
                }
            }
            for (int32 Col = 0; Col < 4; ++Col)
            {
                int32 Pivot = Col;
                for (int32 Row = Col + 1; Row < 4; ++Row)
                {
                    if (FMath::Abs(A[Row][Col]) > FMath::Abs(A[Pivot][Col]))
                    {
                        Pivot = Row;
                    }
                }
                if (FMath::Abs(A[Pivot][Col]) < 1e-15)
                {
                    return false;
                }
                if (Pivot != Col)
                {
                    for (int32 K = 0; K < 8; ++K)
                    {
                        Swap(A[Pivot][K], A[Col][K]);
                    }
                }
                const double Scale = 1.0 / A[Col][Col];
                for (int32 K = 0; K < 8; ++K)
                {
                    A[Col][K] *= Scale;
                }
                for (int32 Row = 0; Row < 4; ++Row)
                {
                    if (Row != Col)
                    {
                        const double Factor = A[Row][Col];
                        for (int32 K = 0; K < 8; ++K)
                        {
                            A[Row][K] -= Factor * A[Col][K];
                        }
                    }
                }
            }
            for (int32 Row = 0; Row < 4; ++Row)
            {
                for (int32 Col = 0; Col < 4; ++Col)
                {
                    Out[Row * 4 + Col] = A[Row][Col + 4];
                }
            }
            return true;
        }
    }

    TArray<FUBotRotorSpec> MakeQuadXRotors(double ArmLengthMeters)
    {
        TArray<FVector> Positions;
        TArray<EUBotRotorSpinDirection> Spins;
        GetQuadXRotorLayout(ArmLengthMeters, Positions, Spins);
        TArray<FUBotRotorSpec> Rotors;
        for (int32 Index = 0; Index < Positions.Num(); ++Index)
        {
            FUBotRotorSpec& Rotor = Rotors.AddDefaulted_GetRef();
            Rotor.PositionMeters = Positions[Index];
            Rotor.SpinDirection = Spins[Index];
        }
        return Rotors;
    }

    double GetMaxRotorThrust(const FUBotRotorSpec& Rotor)
    {
        return FMath::Max(Rotor.ThrustCoefficient, 0.0) * FMath::Square(FMath::Max(Rotor.MaxSpeedRadiansPerSecond, 0.0));
    }

    double CommandToRotorSpeed(const FUBotRotorSpec& Rotor, EUBotMotorCommandMapping Mapping, double Command)
    {
        const double U = FMath::Clamp(Command, 0.0, 1.0);
        const double MaxSpeed = FMath::Max(Rotor.MaxSpeedRadiansPerSecond, 0.0);
        return Mapping == EUBotMotorCommandMapping::Thrust ? MaxSpeed * FMath::Sqrt(U) : MaxSpeed * U;
    }

    double RotorSpeedToCommand(const FUBotRotorSpec& Rotor, EUBotMotorCommandMapping Mapping, double SpeedRadiansPerSecond)
    {
        const double MaxSpeed = FMath::Max(Rotor.MaxSpeedRadiansPerSecond, 1e-9);
        const double Share = FMath::Clamp(SpeedRadiansPerSecond / MaxSpeed, 0.0, 1.0);
        return Mapping == EUBotMotorCommandMapping::Thrust ? Share * Share : Share;
    }

    double GetHoverCommand(const FUBotMultirotorAirframe& Airframe, double GravityMetersPerSecondSquared)
    {
        double TotalMax = 0.0;
        for (const FUBotRotorSpec& Rotor : Airframe.Rotors)
        {
            TotalMax += GetMaxRotorThrust(Rotor);
        }
        if (TotalMax <= 0.0)
        {
            return 0.0;
        }
        const double ThrustShare = FMath::Clamp(Airframe.MassKilograms * FMath::Abs(GravityMetersPerSecondSquared) / TotalMax, 0.0, 1.0);
        return Airframe.MotorCommandMapping == EUBotMotorCommandMapping::Thrust ? ThrustShare : FMath::Sqrt(ThrustShare);
    }

    bool IsAirframeValid(const FUBotMultirotorAirframe& Airframe, FString* OutError)
    {
        const auto Fail = [OutError](const TCHAR* Why)
        {
            if (OutError)
            {
                *OutError = Why;
            }
            return false;
        };
        if (!(Airframe.MassKilograms > 0.0))
        {
            return Fail(TEXT("the mass must be positive"));
        }
        if (!(Airframe.InertiaKilogramSquareMeters.GetMin() > 0.0))
        {
            return Fail(TEXT("every moment of inertia must be positive"));
        }
        if (Airframe.Rotors.IsEmpty())
        {
            return Fail(TEXT("there are no rotors"));
        }
        double TotalMax = 0.0;
        for (const FUBotRotorSpec& Rotor : Airframe.Rotors)
        {
            TotalMax += GetMaxRotorThrust(Rotor);
        }
        if (!(TotalMax > 0.0))
        {
            return Fail(TEXT("the rotors produce no thrust"));
        }
        return true;
    }

    void ComputeRotorWrench(const FUBotMultirotorAirframe& Airframe, TConstArrayView<double> RotorSpeeds, double& OutThrust, FVector& OutTorque)
    {
        OutThrust = 0.0;
        OutTorque = FVector::ZeroVector;
        for (int32 Index = 0; Index < Airframe.Rotors.Num(); ++Index)
        {
            const FUBotRotorSpec& Rotor = Airframe.Rotors[Index];
            const double Speed = RotorSpeeds.IsValidIndex(Index) ? RotorSpeeds[Index] : 0.0;
            const double SpeedSquared = Speed * Speed;
            const double Thrust = Rotor.ThrustCoefficient * SpeedSquared;
            OutThrust += Thrust;
            // r x (0, 0, T), plus the reaction torque against the spin.
            OutTorque += FVector::CrossProduct(Rotor.PositionMeters, FVector(0.0, 0.0, Thrust));
            OutTorque.Z -= SpinSign(Rotor.SpinDirection) * Rotor.TorqueCoefficient * SpeedSquared;
        }
    }

    void StepRigidBody(const FUBotMultirotorAirframe& Airframe, TConstArrayView<double> Commands, const FVector& Gravity,
        const FVector& Wind, double StepSeconds, FRigidBodyState& State)
    {
        const double H = FMath::Max(StepSeconds, 0.0);
        const int32 RotorCount = Airframe.Rotors.Num();
        State.RotorSpeeds.SetNumZeroed(RotorCount);

        // Motors: first order toward the commanded speed, solved exactly over the step.
        for (int32 Index = 0; Index < RotorCount; ++Index)
        {
            const FUBotRotorSpec& Rotor = Airframe.Rotors[Index];
            const double Target = CommandToRotorSpeed(Rotor, Airframe.MotorCommandMapping, Commands.IsValidIndex(Index) ? Commands[Index] : 0.0);
            const double Decay = Rotor.MotorTimeConstantSeconds > 0.0 ? FMath::Exp(-H / Rotor.MotorTimeConstantSeconds) : 0.0;
            State.RotorSpeeds[Index] = Target + (State.RotorSpeeds[Index] - Target) * Decay;
        }

        double Thrust = 0.0;
        FVector Torque = FVector::ZeroVector;
        ComputeRotorWrench(Airframe, State.RotorSpeeds, Thrust, Torque);

        const FVector AirVelocity = State.Orientation.UnrotateVector(State.Velocity - Wind);
        const FVector Drag = -Airframe.LinearDragCoefficients * AirVelocity
            - Airframe.QuadraticDragCoefficients * AirVelocity.GetAbs() * AirVelocity;
        const FVector BodyForce = FVector(0.0, 0.0, Thrust) + Drag;
        const double Mass = FMath::Max(Airframe.MassKilograms, 1e-6);
        const FVector Acceleration = State.Orientation.RotateVector(BodyForce) / Mass + Gravity;

        State.Velocity += Acceleration * H;
        State.Position += State.Velocity * H;
        State.Acceleration = Acceleration;

        const FVector Inertia = Airframe.InertiaKilogramSquareMeters.ComponentMax(FVector(1e-9));
        const FVector& W = State.AngularVelocity;
        const FVector AngularMomentum = Inertia * W;
        const FVector TotalTorque = Torque - Airframe.AngularDragCoefficients * W - FVector::CrossProduct(W, AngularMomentum);
        State.AngularVelocity += (TotalTorque / Inertia) * H;
        State.Orientation = IntegrateBodyRate(State.Orientation, State.AngularVelocity, H);
    }

    bool FThrustAllocator::Build(const FUBotMultirotorAirframe& Airframe)
    {
        RotorCount = 0;
        Inverse.Reset();
        MaxThrust.Reset();
        MaxTotalThrust = 0.0;
        Mapping = Airframe.MotorCommandMapping;
        const int32 Count = Airframe.Rotors.Num();
        if (Count == 0)
        {
            return false;
        }

        // Effectiveness: rows (thrust, torque x, torque y, torque z) per unit thrust of each rotor.
        TArray<double> B;
        B.SetNumZeroed(4 * Count);
        for (int32 Index = 0; Index < Count; ++Index)
        {
            const FUBotRotorSpec& Rotor = Airframe.Rotors[Index];
            const double TorqueRatio = Rotor.ThrustCoefficient > 0.0 ? Rotor.TorqueCoefficient / Rotor.ThrustCoefficient : 0.0;
            B[0 * Count + Index] = 1.0;
            B[1 * Count + Index] = Rotor.PositionMeters.Y;
            B[2 * Count + Index] = -Rotor.PositionMeters.X;
            B[3 * Count + Index] = -SpinSign(Rotor.SpinDirection) * TorqueRatio;
            MaxThrust.Add(GetMaxRotorThrust(Rotor));
            MaxTotalThrust += MaxThrust.Last();
        }

        // Pseudo-inverse B^T (B B^T)^-1.
        double BBt[16];
        for (int32 Row = 0; Row < 4; ++Row)
        {
            for (int32 Col = 0; Col < 4; ++Col)
            {
                double Sum = 0.0;
                for (int32 Index = 0; Index < Count; ++Index)
                {
                    Sum += B[Row * Count + Index] * B[Col * Count + Index];
                }
                BBt[Row * 4 + Col] = Sum;
            }
        }
        double BBtInverse[16];
        if (!Invert4(BBt, BBtInverse) || MaxTotalThrust <= 0.0)
        {
            MaxThrust.Reset();
            MaxTotalThrust = 0.0;
            return false;
        }
        Inverse.SetNumZeroed(Count * 4);
        for (int32 Index = 0; Index < Count; ++Index)
        {
            for (int32 Col = 0; Col < 4; ++Col)
            {
                double Sum = 0.0;
                for (int32 K = 0; K < 4; ++K)
                {
                    Sum += B[K * Count + Index] * BBtInverse[K * 4 + Col];
                }
                Inverse[Index * 4 + Col] = Sum;
            }
        }
        RotorCount = Count;
        return true;
    }

    void FThrustAllocator::Allocate(double Thrust, const FVector& Torque, TArray<double>& OutCommands) const
    {
        OutCommands.SetNumZeroed(RotorCount);
        if (RotorCount == 0)
        {
            return;
        }

        // Rotor thrusts without the yaw torque (Base) and from the yaw torque alone (Yaw).
        TArray<double, TInlineAllocator<8>> Base;
        TArray<double, TInlineAllocator<8>> Yaw;
        Base.SetNumZeroed(RotorCount);
        Yaw.SetNumZeroed(RotorCount);
        for (int32 Index = 0; Index < RotorCount; ++Index)
        {
            const double* Row = &Inverse[Index * 4];
            Base[Index] = Row[0] * Thrust + Row[1] * Torque.X + Row[2] * Torque.Y;
            Yaw[Index] = Row[3] * Torque.Z;
        }

        // Yaw gives way first.
        double YawScale = 1.0;
        for (int32 Index = 0; Index < RotorCount; ++Index)
        {
            const double Total = Base[Index] + Yaw[Index];
            if (Total > MaxThrust[Index] && Yaw[Index] > 0.0)
            {
                YawScale = FMath::Min(YawScale, FMath::Max(0.0, (MaxThrust[Index] - Base[Index]) / Yaw[Index]));
            }
            else if (Total < 0.0 && Yaw[Index] < 0.0)
            {
                YawScale = FMath::Min(YawScale, FMath::Max(0.0, -Base[Index] / Yaw[Index]));
            }
        }

        // Then the collective thrust shifts to keep roll and pitch.
        double High = -UE_DOUBLE_BIG_NUMBER;
        double Low = UE_DOUBLE_BIG_NUMBER;
        double Shift = 0.0;
        for (int32 Index = 0; Index < RotorCount; ++Index)
        {
            const double Value = Base[Index] + YawScale * Yaw[Index];
            High = FMath::Max(High, Value - MaxThrust[Index]);
            Low = FMath::Min(Low, Value);
        }
        if (High > 0.0)
        {
            Shift = -High;
        }
        else if (Low < 0.0)
        {
            Shift = FMath::Min(-Low, -High);
        }

        for (int32 Index = 0; Index < RotorCount; ++Index)
        {
            const double RotorThrust = FMath::Clamp(Base[Index] + YawScale * Yaw[Index] + Shift, 0.0, MaxThrust[Index]);
            const double Share = MaxThrust[Index] > 0.0 ? RotorThrust / MaxThrust[Index] : 0.0;
            OutCommands[Index] = Mapping == EUBotMotorCommandMapping::Thrust ? Share : FMath::Sqrt(Share);
        }
    }
}
