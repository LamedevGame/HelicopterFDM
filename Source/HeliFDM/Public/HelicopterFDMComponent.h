#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "HelicopterFDMTypes.h"
#include "HelicopterFDMComponent.generated.h"

/**
 * UHelicopterFDMComponent - Helicopter Flight Dynamic Model
 *
 * Game-oriented helicopter physics: rotor thrust, cyclic/collective control,
 * engine governor, hull drag, and stability augmentation.
 * All configuration is done via Blueprint properties.
 */
UCLASS(ClassGroup=(Simulation), meta=(BlueprintSpawnableComponent, DisplayName="Helicopter FDM"))
class HELIFDM_API UHelicopterFDMComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	UHelicopterFDMComponent();

protected:
	virtual void BeginPlay() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// ============================================
	// GENERAL
	// ============================================

	/** Enable FDM simulation */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM")
	bool bEnabled = false;

	/** Airport field elevation in feet MSL — altimeter reads this on the ground */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM")
	float FieldElevationFeet = 0.0f;

	// ============================================
	// PHYSICS
	// ============================================

	/** Diagonal inertia tensor Ixx, Iyy, Izz (kg*m^2) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Physics")
	FVector InertiaTensor = FVector(2200.0, 7500.0, 6800.0);

	// ============================================
	// MAIN ROTOR
	// ============================================

	/** Main rotor position and radius */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Main Rotor")
	FRotorConfig MainRotor;

	/** Thrust at full collective as multiplier of weight (1.0 = hover, 1.3 = can climb) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Main Rotor", meta=(ClampMin="0.5", ClampMax="3.0"))
	double ThrustMultiplier = 1.25;

	/** Fraction of thrust that becomes reactive torque (0.05-0.15) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Main Rotor", meta=(ClampMin="0.01", ClampMax="0.3"))
	double TorqueFraction = 0.07;

	/** Thrust response time (seconds, lower = snappier, 0 = instant) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Main Rotor", meta=(ClampMin="0", ClampMax="0.5"))
	double ThrustResponseTime = 0.05;

	/** Maximum disc tilt from cyclic input (degrees) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Main Rotor")
	double CyclicMaxTilt = 10.0;

	/** Nominal main rotor RPM (governor setpoint) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Main Rotor", meta=(ClampMin="1"))
	double MainRotorRPMNominal = 394.0;

	/** Main rotor direction: +1 = CCW from above (US), -1 = CW */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Main Rotor", meta=(ClampMin="-1", ClampMax="1"))
	double MainRotorDirection = -1.0;

	// ============================================
	// TAIL ROTOR
	// ============================================

	/** Tail rotor position (X = moment arm for yaw) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Tail Rotor")
	FRotorConfig TailRotor;

	/** Tail rotor gear ratio (TailRPM = MainRPM * ratio) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Tail Rotor", meta=(ClampMin="0.1"))
	double TailRotorGearRatio = 5.32;

	/** Pedal yaw authority — how much yaw force pedals produce (multiplier) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Tail Rotor", meta=(ClampMin="0.1", ClampMax="5.0"))
	double PedalYawAuthority = 1.2;

	// ============================================
	// ENGINE
	// ============================================

	/** Engine idle RPM */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Engine", meta=(ClampMin="0"))
	double EngineIdleRPM = 550.0;

	/** Engine max RPM */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Engine", meta=(ClampMin="1"))
	double EngineMaxRPM = 2500.0;

	/** Engine max power (HP) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Engine", meta=(ClampMin="1"))
	double EngineMaxPowerHP = 315.0;

	/** Engine RPM / Main rotor RPM */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Engine", meta=(ClampMin="0.1"))
	double EngineToRotorRatio = 7.05;

	/** Rotational inertia of engine + rotor system (kg*m^2, higher = slower spool) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Engine", meta=(ClampMin="1"))
	double EngineMomentOfInertia = 950.0;

	/** Governor base throttle at nominal RPM with zero collective (0.3-0.6) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Engine", meta=(ClampMin="0.1", ClampMax="0.8"))
	double GovernorBaseThrottle = 0.35;

	/** Time to spool up from zero to nominal (seconds) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Engine", meta=(ClampMin="0.1", ClampMax="60"))
	float EngineSpoolUpTime = 12.0f;

	/** Time to spool down from nominal to zero (seconds) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Engine", meta=(ClampMin="0.1", ClampMax="60"))
	float EngineSpoolDownTime = 8.0f;

	// ============================================
	// HULL AERODYNAMICS
	// ============================================

	/** Fuselage reference area for drag (m^2) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Hull", meta=(ClampMin="0"))
	double HullReferenceArea = 5.0;

	/** Fuselage parasitic drag coefficient */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Hull", meta=(ClampMin="0"))
	double HullDragCoefficient = 0.35;

	/** Landing gear drag coefficient */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Hull", meta=(ClampMin="0"))
	double LandingGearDragCoefficient = 0.25;

	/** Landing gear reference area (m^2) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Hull", meta=(ClampMin="0"))
	double LandingGearReferenceArea = 1.2;

	// ============================================
	// SAS (Stability Augmentation System)
	// ============================================

	/** Pitch rate damping gain */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|SAS", meta=(ClampMin="0", ClampMax="20"))
	float SAS_RateDampingPitch = 6.0f;

	/** Roll rate damping gain */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|SAS", meta=(ClampMin="0", ClampMax="20"))
	float SAS_RateDampingRoll = 5.0f;

	/** Yaw rate damping gain */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|SAS", meta=(ClampMin="0", ClampMax="20"))
	float SAS_RateDampingYaw = 6.0f;

	/** Attitude hold strength — returns to level when cyclic released */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|SAS", meta=(ClampMin="0", ClampMax="10"))
	float SAS_AttitudeHoldStrength = 2.5f;

	/** Cyclic input deadzone for attitude hold activation */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|SAS", meta=(ClampMin="0", ClampMax="0.5"))
	float SAS_AttitudeHoldDeadzone = 0.1f;

	/** Progressive gain — stronger correction at larger angles */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|SAS", meta=(ClampMin="0", ClampMax="5"))
	float SAS_AttitudeHoldProgressive = 1.5f;

	/** Minimum cyclic authority as fraction of hover thrust [0-0.3] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|SAS", meta=(ClampMin="0", ClampMax="0.3"))
	float MinCyclicAuthorityFraction = 0.15f;

	/** Maximum cyclic angular acceleration (rad/s^2) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|SAS", meta=(ClampMin="0.5", ClampMax="10"))
	float MaxCyclicAcceleration = 3.0f;

	// ============================================
	// VELOCITY LIMITS
	// ============================================

	/** Maximum climb rate before damping kicks in (m/s) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Velocity Limits", meta=(ClampMin="1", ClampMax="30"))
	float MaxClimbRate = 8.0f;

	/** Maximum descent rate before damping kicks in (m/s) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Velocity Limits", meta=(ClampMin="1", ClampMax="30"))
	float MaxDescentRate = 12.0f;

	/** Vertical speed damping strength (multiplier on mass) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Velocity Limits", meta=(ClampMin="0", ClampMax="10"))
	float VerticalDampingGain = 2.0f;

	/** Lateral drift damping strength (multiplier on mass) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Velocity Limits", meta=(ClampMin="0", ClampMax="10"))
	float LateralDampingGain = 2.0f;

	/** Backward flight damping strength (multiplier on mass) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Velocity Limits", meta=(ClampMin="0", ClampMax="10"))
	float BackwardDampingGain = 1.5f;

	// ============================================
	// ENVELOPE PROTECTION
	// ============================================

	/** Maximum allowed pitch angle (degrees) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Envelope", meta=(ClampMin="10", ClampMax="80"))
	float EnvelopeMaxPitch = 30.0f;

	/** Maximum allowed roll angle (degrees) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Envelope", meta=(ClampMin="15", ClampMax="80"))
	float EnvelopeMaxRoll = 45.0f;

	/** Envelope pushback strength */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Envelope", meta=(ClampMin="0.5", ClampMax="20"))
	float EnvelopeStrength = 8.0f;

	// ============================================
	// INPUT
	// ============================================

	/** Cyclic pitch sensitivity (forward/backward) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Input", meta=(ClampMin="0.01", ClampMax="2.0"))
	float CyclicSensitivityPitch = 1.0f;

	/** Cyclic roll sensitivity (left/right) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Input", meta=(ClampMin="0.01", ClampMax="2.0"))
	float CyclicSensitivityRoll = 1.0f;

	/** Pedal input sensitivity */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Input", meta=(ClampMin="0.01", ClampMax="2.0"))
	float PedalSensitivity = 1.0f;

	/** Collective input sensitivity */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Input", meta=(ClampMin="0.01", ClampMax="2.0"))
	float CollectiveSensitivity = 1.0f;

	/** Cyclic input smoothing time constant (seconds, lower = more responsive) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Input", meta=(ClampMin="0.01", ClampMax="1.0"))
	float InputSmoothingCyclic = 0.12f;

	/** Pedal input smoothing time constant (seconds) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Input", meta=(ClampMin="0.01", ClampMax="1.0"))
	float InputSmoothingPedals = 0.2f;

	/** Collective input smoothing time constant (seconds) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Input", meta=(ClampMin="0.01", ClampMax="1.0"))
	float InputSmoothingCollective = 0.05f;

	// ============================================
	// OPTIMIZATION
	// ============================================

	/** How often to update ground height via line trace (seconds) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Optimization", meta=(ClampMin="0.01", ClampMax="1.0"))
	float GroundTraceInterval = 0.2f;

	// ============================================
	// ENGINE CONTROL API
	// ============================================

	/** Start the engine — RPM spools up to nominal */
	UFUNCTION(BlueprintCallable, Category = "HeliFDM|Engine")
	void StartEngine();

	/** Stop the engine — RPM drops to zero */
	UFUNCTION(BlueprintCallable, Category = "HeliFDM|Engine")
	void StopEngine();

	/** Is the engine currently running */
	UFUNCTION(BlueprintPure, Category = "HeliFDM|Engine")
	bool IsEngineRunning() const { return bEngineRunning; }

	// ============================================
	// CONTROL INPUT API
	// ============================================

	UPROPERTY(BlueprintReadWrite, Category = "HeliFDM|Controls")
	FHelicopterControls Controls;

	UFUNCTION(BlueprintCallable, Category = "HeliFDM|Controls")
	void SetControls(const FHelicopterControls& NewControls);

	UFUNCTION(BlueprintCallable, Category = "HeliFDM|Controls")
	void SetCyclicLongitudinal(float Value);

	UFUNCTION(BlueprintCallable, Category = "HeliFDM|Controls")
	void SetCyclicLateral(float Value);

	UFUNCTION(BlueprintCallable, Category = "HeliFDM|Controls")
	void SetCollective(float Value);

	UFUNCTION(BlueprintCallable, Category = "HeliFDM|Controls")
	void SetPedals(float Value);

	// ============================================
	// FLIGHT STATE API
	// ============================================

	/** Airspeed in knots */
	UFUNCTION(BlueprintPure, Category = "HeliFDM|State")
	float GetAirspeed() const;

	/** Altitude in feet MSL */
	UFUNCTION(BlueprintPure, Category = "HeliFDM|State")
	float GetAltitude() const;

	/** Vertical speed in feet/min */
	UFUNCTION(BlueprintPure, Category = "HeliFDM|State")
	float GetVerticalSpeed() const;

	/** Main rotor RPM as percentage of nominal (100% = normal) */
	UFUNCTION(BlueprintPure, Category = "HeliFDM|State")
	float GetRotorRPMPercent() const;

	/** Engine RPM as percentage of max (100% = redline) */
	UFUNCTION(BlueprintPure, Category = "HeliFDM|State")
	float GetEngineRPMPercent() const;

	/** Main rotor thrust in Newtons */
	UFUNCTION(BlueprintPure, Category = "HeliFDM|State")
	float GetMainRotorThrust() const;

	/** Tail rotor RPM as percentage of nominal */
	UFUNCTION(BlueprintPure, Category = "HeliFDM|State")
	float GetTailRotorRPMPercent() const;

	/** Raw main rotor RPM (for visual blade rotation) */
	UFUNCTION(BlueprintPure, Category = "HeliFDM|State")
	float GetMainRotorRPM() const;

	/** Raw tail rotor RPM (for visual blade rotation) */
	UFUNCTION(BlueprintPure, Category = "HeliFDM|State")
	float GetTailRotorRPM() const;

	/** Pitch angle in degrees */
	UFUNCTION(BlueprintPure, Category = "HeliFDM|State")
	float GetPitch() const;

	/** Roll angle in degrees */
	UFUNCTION(BlueprintPure, Category = "HeliFDM|State")
	float GetRoll() const;

	/** Heading 0-360 degrees */
	UFUNCTION(BlueprintPure, Category = "HeliFDM|State")
	float GetHeading() const;

	/** Height above ground in meters */
	UFUNCTION(BlueprintPure, Category = "HeliFDM|State")
	float GetGroundHeight() const;

protected:
	// ============================================
	// INTERNAL STATE
	// ============================================

	UPROPERTY()
	TObjectPtr<UPrimitiveComponent> PhysicsComponent;

	bool bPhysicsApplied = false;
	double StartingZCm = 0.0;
	double MassKg = 0.0;
	float CurrentDeltaTime = 0.0f;

	// Smoothed control inputs
	double SmoothedElevator = 0.0;
	double SmoothedAileron = 0.0;
	double SmoothedThrottle = 0.0;
	double SmoothedRudder = 0.0;

	// Environment
	static constexpr double AirDensity = 1.225;
	static constexpr double Gravity = 9.81;

	// Engine state
	bool bEngineRunning = false;
	bool bEngineSpooling = false;
	double EngineSpoolTimer = 0.0;
	double EngineSpoolStartRPM = 0.0;
	double EngineRPM_Internal = 0.0;

	// Governor PID state
	double GovernorIntegral = 0.0;

	// Main rotor state
	double MainRotorRPM_Internal = 0.0;
	double MainRotorThrust = 0.0;
	double MainRotorTorque = 0.0;
	double CyclicLon = 0.0;
	double CyclicLat = 0.0;

	// Ground effect
	double GroundHeight = 1000.0;
	float GroundTraceTimer = 0.0f;

	// Telemetry log timer
	double TelemetryTimer = 0.0;

	// ============================================
	// SIMULATION STEPS
	// ============================================

	void ResolvePhysicsBody();
	void ConfigureBodyInstance();
	void FilterControlInputs(float DeltaTime);
	void TickPowerplant(float DeltaTime);
	void SolveRotorDisc();
	void CommitForcesAndTorques();
	double EvaluateIGEFactor();
};
