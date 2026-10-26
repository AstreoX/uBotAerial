#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS
#include "UBotAerialTestWorld.h"
#include "UBotAerialTestTypes.h"
#include "UBotAerialBodyStateHistory.h"
#include "UBotBodyStateHistory.h"
#include "UBotSimpleFlightComponent.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialHistoryRingTest, "UBotAerial.BodyState.History.RingAndDiscontinuities", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FUBotAerialHistoryRingTest::RunTest(const FString& Parameters)
{
    UBotAerialTests::FScopedAerialTestWorld Scope;
    AActor* Actor = Scope.Get()->SpawnActor<AActor>();
    USceneComponent* Body = NewObject<USceneComponent>(Actor);
    Actor->SetRootComponent(Body); Body->RegisterComponent();
    FUBotAerialBodyStateHistory History(Body, 4);
    FUBotBodyState State;
    State.bHasLinearAcceleration = State.bHasAngularAcceleration = true;
    for (int32 Index = 0; Index < 6; ++Index)
    {
        State.TimestampSeconds = Index;
        State.PositionMeters.X = Index;
        State.Orientation = FQuat(FVector::UpVector, Index * 0.1);
        History.Append(State);
    }
    FUBotBodyStateHistoryRange Range;
    TestTrue(TEXT("Ring has a range"), History.GetRange(Range));
    TestEqual(TEXT("Old states evicted"), Range.OldestSeconds, 2.0);
    uint64 Segment = 0;
    TestEqual(TEXT("Exact endpoint"), History.Query(5.0, State, Segment), EUBotBodyStateQueryResult::Exact);
    TestEqual(TEXT("No extrapolation"), History.Query(5.01, State, Segment), EUBotBodyStateQueryResult::NotReady);
    TestEqual(TEXT("No overwritten state"), History.Query(1.99, State, Segment), EUBotBodyStateQueryResult::Unavailable);
    TestEqual(TEXT("Interpolated state"), History.Query(3.5, State, Segment), EUBotBodyStateQueryResult::Interpolated);
    TestTrue(TEXT("Position interpolation"), FMath::Abs(State.PositionMeters.X - 3.5) < 1.e-12);
    TestTrue(TEXT("Shortest arc orientation"), State.Orientation.AngularDistance(FQuat(FVector::UpVector, 0.35)) < 1.e-6);
    History.BreakSegment();
    State.TimestampSeconds = 6; History.Append(State);
    TestEqual(TEXT("Cannot interpolate an impulse"), History.Query(5.5, State, Segment), EUBotBodyStateQueryResult::Unavailable);
    History.Clear();
    TestFalse(TEXT("Reset clears old states"), History.GetRange(Range));
    State.TimestampSeconds = 0; History.Append(State);
    TestEqual(TEXT("Time reset starts a new range"), History.Query(0, State, Segment), EUBotBodyStateQueryResult::Exact);
    History.Invalidate();
    TestFalse(TEXT("Surviving view is invalidated"), History.GetRange(Range));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialHistoryLifetimeTest, "UBotAerial.BodyState.History.LifetimeAndSteps", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FUBotAerialHistoryLifetimeTest::RunTest(const FString& Parameters)
{
    UBotAerialTests::FScopedAerialTestWorld Scope;
    AUBotAerialTestNavigationDrone* Drone = Scope.SpawnAndBegin<AUBotAerialTestNavigationDrone>(FVector(0, 0, 200));
    UUBotMultirotorMovementComponent* Movement = Drone->GetMultirotorMovement();
    auto A = Movement->AcquireBodyStateHistory(Drone->GetRootComponent());
    auto B = Movement->AcquireBodyStateHistory(Drone->GetRootComponent());
    TestTrue(TEXT("Consumers share storage"), A && A == B);
    TWeakPtr<const IUBotBodyStateHistory> Weak = A;
    Scope.Tick(0.05f);
    FUBotBodyStateHistoryRange Range;
    TestTrue(TEXT("Completed steps recorded"), A->GetRange(Range));
    FUBotBodyState State; uint64 Segment = 0;
    TestEqual(TEXT("Intermediate physical step exists"), A->Query(0.02, State, Segment), EUBotBodyStateQueryResult::Exact);
    Movement->TeleportMultirotor(FVector(500, 0, 200), FRotator::ZeroRotator, true);
    TestEqual(TEXT("Teleport removes old history"), A->Query(0.02, State, Segment), EUBotBodyStateQueryResult::Unavailable);
    A.Reset(); TestTrue(TEXT("Other consumer retains history"), Weak.IsValid());
    B.Reset(); TestFalse(TEXT("Provider does not own the last handle"), Weak.IsValid());
    A = Movement->AcquireBodyStateHistory(Drone->GetRootComponent());
    Movement->MaxStepsPerFrame = 2;
    Scope.Tick(0.05f);
    TestTrue(TEXT("History resumes after dropped steps"), A->GetRange(Range));
    TestTrue(TEXT("No state fabricated in skipped time"), Range.OldestSeconds >= Scope.Now() - 0.005);
    Movement->EndPlay(EEndPlayReason::RemovedFromWorld);
    TestFalse(TEXT("EndPlay invalidates before unregister or GC"), A->GetRange(Range));
    TestFalse(TEXT("Ended producer cannot create another view"), Movement->AcquireBodyStateHistory(Drone->GetRootComponent()).IsValid());
    Drone->Destroy();
    TestFalse(TEXT("Actor end invalidates surviving handle"), A->GetRange(Range));
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialHistoryWorldTest, "UBotAerial.BodyState.History.WorldIsolationAndResize", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FUBotAerialHistoryWorldTest::RunTest(const FString& Parameters)
{
    UBotAerialTests::FScopedAerialTestWorld First;
    auto* A = First.SpawnAndBegin<AUBotAerialTestNavigationDrone>(FVector(0,0,200));
    auto ViewA = A->GetMultirotorMovement()->AcquireBodyStateHistory(A->GetRootComponent());
    First.Tick(0.05f);
    FUBotBodyStateHistoryRange BeforeA, BeforeB, After;
    TestTrue(TEXT("First range ready"), ViewA->GetRange(BeforeA));
    UBotAerialTests::FScopedAerialTestWorld Second;
    auto* B = Second.SpawnAndBegin<AUBotAerialTestNavigationDrone>(FVector(100,0,200));
    auto ViewB = B->GetMultirotorMovement()->AcquireBodyStateHistory(B->GetRootComponent());
    Second.Tick(0.1f);
    TestTrue(TEXT("Second range ready"), ViewB->GetRange(BeforeB));
    ViewA->GetRange(After);
    TestEqual(TEXT("Advancing another world does not advance this history"), After.NewestSeconds, BeforeA.NewestSeconds);
    A->GetMultirotorMovement()->MaxStepsPerFrame = 8;
    First.Tick(0.01f);
    ViewA->GetRange(After);
    TestTrue(TEXT("Capacity change starts a new generation"), After.Generation != BeforeA.Generation);
    FUBotBodyState State; uint64 Segment = 0;
    TestEqual(TEXT("Resized history does not expose old entries"), ViewA->Query(0.02,State,Segment), EUBotBodyStateQueryResult::Unavailable);
    A->Destroy();
    TestTrue(TEXT("Other world survives producer destruction"), ViewB->GetRange(After));
    TestEqual(TEXT("Other generation unchanged"), After.Generation, BeforeB.Generation);
    TestEqual(TEXT("Other completed time unchanged"), After.NewestSeconds, BeforeB.NewestSeconds);
    return true;
}
#endif
