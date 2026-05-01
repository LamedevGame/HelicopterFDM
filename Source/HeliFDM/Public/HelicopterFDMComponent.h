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

	/** Altitude (ft MSL) where rotor thrust + engine power drop to ~37% of sea-level value.
	 *  Defines natural service ceiling. Apache ≈ 15,000 ft, light heli ≈ 12,000 ft, heavy lift ≈ 10,000 ft. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM", meta=(ClampMin="3000", ClampMax="30000"))
	float ServiceCeilingFeet = 15000.0f;

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

	/** Thrust response time (seconds, lower = snappier, 0 = instant) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Main Rotor", meta=(ClampMin="0", ClampMax="0.5"))
	double ThrustResponseTime = 0.05;

	/** Maximum disc tilt from cyclic input (degrees) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Main Rotor")
	double CyclicMaxTilt = 10.0;

	/** Nominal main rotor RPM (governor setpoint) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Main Rotor", meta=(ClampMin="1"))
	double MainRotorRPMNominal = 394.0;

	/** Main rotor spin direction. true = CW from above (Mil/Eurocopter, default), false = CCW (US Bell/Sikorsky). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Main Rotor")
	bool bMainRotorClockwise = true;

	// ============================================
	// TAIL ROTOR
	// ============================================

	/** Tail rotor position (X = moment arm for yaw) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Tail Rotor")
	FRotorConfig TailRotor;

	/** Tail rotor gear ratio (TailRPM = MainRPM * ratio) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Tail Rotor", meta=(ClampMin="0.1"))
	double TailRotorGearRatio = 5.32;

	/** Pedal authority — fraction of available tail rotor thrust used at full pedal. 1.0 = full physical thrust. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Tail Rotor", meta=(ClampMin="0.1", ClampMax="2.0"))
	double PedalYawAuthority = 1.0;

	/** Enable translating tendency — tail rotor thrust pushes body sideways (realistic).
	 *  Disable for arcade feel without lateral drift on pedal input. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Tail Rotor")
	bool bEnableTranslatingTendency = true;

	// ============================================
	// FORWARD FLIGHT
	// ============================================

	/** Translational lift bonus at full ETL — peak thrust multiplier above hover (0.10–0.15 typical).
	 *  Felt as the "kick" of accelerating through ~15–25 knots. Set to 0 to disable. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Forward Flight", meta=(ClampMin="0", ClampMax="0.5"))
	float ETLBoost = 0.12f;

	/** Enable Vortex Ring State — fast descent at low forward speed loses thrust (settling with power).
	 *  Recovery: drop nose, gain forward speed. Realistic hazard, replaces arbitrary descent caps. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Forward Flight")
	bool bEnableVRS = true;

	/** Peak VRS thrust loss when fully developed (descending hard, near-zero forward speed). 0.3 = 30% drop. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Forward Flight", meta=(ClampMin="0", ClampMax="0.7"))
	float VRSStrength = 0.3f;

	// ============================================
	// ENGINE
	// ============================================

	/** Engine max power (HP) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Engine", meta=(ClampMin="1"))
	double EngineMaxPowerHP = 315.0;

	/** Engine RPM / Main rotor RPM */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Engine", meta=(ClampMin="0.1"))
	double EngineToRotorRatio = 7.05;

	/** Rotational inertia of engine + rotor system (kg*m^2, higher = slower spool) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Engine", meta=(ClampMin="1"))
	double EngineMomentOfInertia = 950.0;

	/** Time to spool up from zero to nominal (seconds) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Engine", meta=(ClampMin="0.1", ClampMax="60"))
	float EngineSpoolUpTime = 12.0f;

	/** Time to spool down from nominal to zero (seconds) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Engine", meta=(ClampMin="0.1", ClampMax="60"))
	float EngineSpoolDownTime = 8.0f;

	// ============================================
	// HULL AERODYNAMICS
	// ============================================

	/** Forward fuselage drag area Cd·A (m²). Drag = ½·ρ·V²·CdA. Typical light heli ≈ 1.5, Apache ≈ 6. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Hull", meta=(ClampMin="0"))
	double HullDragCdA = 1.75;

	/** Landing gear drag area Cd·A (m²). Set 0 for retractable gear up. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Hull", meta=(ClampMin="0"))
	double LandingGearDragCdA = 0.30;

	/** Lateral drag multiplier — fuselage is much draggier sideways than forward.
	 *  1.0 = isotropic. 2.5–3.5 typical for real fuselages. Drives skid arrest in pedal turns. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Hull", meta=(ClampMin="1.0", ClampMax="6.0"))
	float LateralDragMultiplier = 3.0f;

	/** Weather-cock yaw stability — restoring moment that aligns nose with airflow in forward flight.
	 *  Felt as nose pulling back to direction of motion during pedal-induced skids. 0 = disabled. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Hull", meta=(ClampMin="0", ClampMax="2"))
	float SideslipYawGain = 0.5f;

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

	/** Auto-compensate vertical lift loss when body is tilted (banking/pitching).
	 *  ON (default): heli holds altitude through banked turns automatically (arcade feel).
	 *  OFF: pilot must add collective during maneuvers (realistic). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|SAS")
	bool bAutoCollectiveCompensation = true;

	/** Auto-trim tail rotor to balance main rotor reactive torque (eliminates need for constant pedal).
	 *  ON: hovers without pedal input, but rapid collective pulls still cause brief yaw kick (realistic).
	 *  OFF: pilot must hold pedal continuously to counter reactive torque (hardcore). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|SAS")
	bool bEnableAutoTrim = true;

	// ============================================
	// VELOCITY LIMITS
	// ============================================

	/** Maximum climb rate before damping kicks in (m/s) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Velocity Limits", meta=(ClampMin="1", ClampMax="30"))
	float MaxClimbRate = 8.0f;

	// ============================================
	// ENVELOPE PROTECTION
	// ============================================

	/** Maximum allowed pitch angle (degrees) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Envelope", meta=(ClampMin="10", ClampMax="80"))
	float EnvelopeMaxPitch = 30.0f;

	/** Maximum allowed roll angle (degrees) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Envelope", meta=(ClampMin="15", ClampMax="80"))
	float EnvelopeMaxRoll = 45.0f;

	// ============================================
	// INPUT
	// ============================================

	/** Player input tuning — sensitivity and smoothing per axis. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HeliFDM|Input")
	FInputTuning InputTuning;

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

	/** Advance ratio (V_forward / V_tip) — useful for HUD and tuning forward-flight effects */
	UFUNCTION(BlueprintPure, Category = "HeliFDM|State")
	float GetAdvanceRatio() const { return static_cast<float>(AdvanceRatioCached); }

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

	// Forward-flight cache
	double AdvanceRatioCached = 0.0;

	// Yaw auto-trim — moment (N·m) that cancels reactive torque in steady state
	double TailTrimYawMoment = 0.0;

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
	double GetAltitudeFactor() const;
};
