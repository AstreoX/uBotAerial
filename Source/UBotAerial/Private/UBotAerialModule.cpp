#include "UBotAerial.h"

#include "Modules/ModuleManager.h"
#include "UBotBuiltInMultirotorBackend.h"
#include "UBotExtensionRegistry.h"

#define LOCTEXT_NAMESPACE "UBotAerial"

DEFINE_LOG_CATEGORY(LogUBotAerial);

void FUBotAerialModule::StartupModule()
{
    FUBotExtensionRegistry& Registry = FUBotExtensionRegistry::Get();
    Registry.RegisterExtensionPoint(UBot::Aerial::DynamicsBackendExtensionPoint, UBot::Aerial::PackageName,
        LOCTEXT("DynamicsBackendPoint", "Dynamics backends of physics multirotors (IUBotMultirotorBackendFactory). Experimental."));
    Registry.RegisterExtension(UBot::Aerial::DynamicsBackendExtensionPoint, UBot::Aerial::PackageName,
        MakeShared<FUBotBuiltInMultirotorBackendFactory>());
}

void FUBotAerialModule::ShutdownModule()
{
    FUBotExtensionRegistry::Get().UnregisterAllFromPackage(UBot::Aerial::PackageName);
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FUBotAerialModule, UBotAerial)
