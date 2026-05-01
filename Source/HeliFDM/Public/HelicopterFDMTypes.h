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

	/** Hub position in local space (cm, UE coordinate convention).
	 *  Main rotor: only Z is read (cyclic moment arm above CoG).
	 *  Tail rotor: only X is read (yaw moment arm from CoG). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rotor")
	FVector Position = FVector::ZeroVector;

	/** Blade tip radius (m). Main: ground effect, tip speed, torque arm. Tail: physical thrust formula. */
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

USTRUCT(BlueprintType)
struct HELIFDM_API FInputTuning
{
	GENERATED_BODY()

	/** Cyclic pitch sensitivity (forward/backward) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Input", meta=(ClampMin="0.01", ClampMax="2.0"))
	float CyclicSensitivityPitch = 1.0f;

	/** Cyclic roll sensitivity (left/right) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Input", meta=(ClampMin="0.01", ClampMax="2.0"))
	float CyclicSensitivityRoll = 1.0f;

	/** Pedal input sensitivity */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Input", meta=(ClampMin="0.01", ClampMax="2.0"))
	float PedalSensitivity = 1.0f;

	/** Collective input sensitivity */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Input", meta=(ClampMin="0.01", ClampMax="2.0"))
	float CollectiveSensitivity = 1.0f;

	/** Pitch attack — how slowly nose lifts/dips when stick is pressed (s). Higher = more gradual. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Input", meta=(ClampMin="0.01", ClampMax="2.0"))
	float SmoothingPitch = 0.80f;

	/** Pitch release — how quickly nose returns to neutral when stick released (s). Lower = snappier. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Input", meta=(ClampMin="0.01", ClampMax="1.0"))
	float SmoothingPitchRelease = 0.12f;

	/** Smoothing time constant for roll, yaw, and collective inputs (s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Input", meta=(ClampMin="0.01", ClampMax="1.0"))
	float SmoothingOther = 0.12f;
};

