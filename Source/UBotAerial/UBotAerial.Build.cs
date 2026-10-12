using UnrealBuildTool;

public class UBotAerial : ModuleRules
{
    public UBotAerial(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new[]
        {
            "Core",
            "CoreUObject",
            "Engine",
            "EnhancedInput",
            "UBotCore"
        });

        PrivateDependencyModuleNames.AddRange(new[]
        {
            "InputCore"
        });
    }
}
