#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "UBotAerialTypes.h"
#include "UBotWind.generated.h"

class UArrowComponent;

/**
 * Wind for uBot multirotors. Place one in the level: its wind blows everywhere (the field is uniform),
 * and several actors add up. The arrow shows the steady wind's direction in the editor.
 */
UCLASS(meta = (DisplayName = "uBot Wind"))
class UBOTAERIAL_API AUBotWind : public AActor
{
    GENERATED_BODY()

public:
    AUBotWind();

    /** Whether this wind blows. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Wind")
    bool bEnabled = true;

    /** Steady wind and gusts. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Wind")
    FUBotWindSettings Settings;

    /** Changes the wind from now on. */
    UFUNCTION(BlueprintCallable, Category = "uBot|Aerial|Wind", meta = (DisplayName = "Set Wind Settings"))
    void SetWindSettings(const FUBotWindSettings& NewSettings);

    /** Turns this wind on or off. */
    UFUNCTION(BlueprintCallable, Category = "uBot|Aerial|Wind", meta = (DisplayName = "Set Wind Enabled"))
    void SetWindEnabled(bool bNewEnabled);

    virtual void OnConstruction(const FTransform& Transform) override;

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
    void PointArrow();

    UPROPERTY(VisibleAnywhere, Category = "Wind")
    TObjectPtr<UArrowComponent> Arrow;
};
