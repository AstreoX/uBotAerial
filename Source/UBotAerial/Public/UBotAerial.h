#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleInterface.h"

UBOTAERIAL_API DECLARE_LOG_CATEGORY_EXTERN(LogUBotAerial, Log, All);

namespace UBot::Aerial
{
    /** Package name used for extension registrations of this plugin. */
    inline const FName PackageName(TEXT("UBotAerial"));

    /** Extension point for multirotor dynamics backends (IUBotMultirotorBackendFactory). Experimental. */
    inline const FName DynamicsBackendExtensionPoint(TEXT("UBotAerial.DynamicsBackend"));

    /** Name of the dynamics backend built into this plugin. */
    inline const FName BuiltInBackendName(TEXT("BuiltIn"));
}

class FUBotAerialModule final : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};
