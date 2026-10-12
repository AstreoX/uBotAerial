#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Modules/ModuleManager.h"
#include "UBotAerial.h"
#include "UBotPackageRegistry.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialModuleLoadsTest, "UBotAerial.Module.Loads", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialModuleLoadsTest::RunTest(const FString& Parameters)
{
    TestTrue(TEXT("The UBotAerial module is loaded"), FModuleManager::Get().IsModuleLoaded(TEXT("UBotAerial")));

    FUBotPackageRegistry Registry;
    Registry.Refresh();
    const FUBotPackageInfo* Package = Registry.FindPackage(TEXT("UBotAerial"));
    if (!TestNotNull(TEXT("The package registry finds UBotAerial"), Package))
    {
        return false;
    }

    TestEqual(TEXT("Layer"), Package->Layer, EUBotPackageLayer::Capability);
    TestEqual(TEXT("Provides"), FString::Join(Package->Provides, TEXT(",")),
        FString(TEXT("Actuation.Velocity,Actuation.Attitude,Actuation.Motor,Body.Multirotor,Aerial.Wind")));
    TestEqual(TEXT("Engine plugins"), FString::Join(Package->ExternalRequires, TEXT(",")), FString(TEXT("EnhancedInput")));
    if (TestEqual(TEXT("One requirement"), Package->Requires.Num(), 1))
    {
        TestEqual(TEXT("Requires UBotCore"), Package->Requires[0].Name, FString(TEXT("UBotCore")));
    }
    TestTrue(TEXT("The descriptor has no problems"), Package->Problems.IsEmpty());
    return true;
}

#endif
