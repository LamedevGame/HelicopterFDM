#pragma once

#include "CoreMinimal.h"
#include "HelicopterFDMTypes.generated.h"

// ============================================================================
// Blueprint-exposed configuration and data structs
// ============================================================================

USTRUCT(BlueprintType)
struct HELIFDM_API FRotorConfig
{
	GENERATED_BODY()

	/** Hub position in local space (cm, UE coordinate convention) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rotor")
	FVector Position = FVector::ZeroVector;

	/** Blade tip radius (m) — used for ground effect */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rotor", meta=(ClampMin="0.1"))
	double Radius = 7.3;
};

USTRUCT(BlueprintType)
struct HELIFDM_API FHelicopterControls
{
	GENERATED_BODY()

	/** Cyclic longitudinal / elevator [-1, 1] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Controls")
	float Elevator = 0.0f;

	/** Cyclic lateral / aileron [-1, 1] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Controls")
	float Aileron = 0.0f;

	/** Collective / throttle [0, 1] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Controls")
	float Throttle = 0.0f;

	/** Pedals / rudder [-1, 1] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Controls")
	float Rudder = 0.0f;
};

