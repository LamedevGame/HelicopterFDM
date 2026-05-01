#include "HelicopterFDMComponent.h"
#include "Components/PrimitiveComponent.h"
#include "GameFramework/Actor.h"
#include "Engine/World.h"

DEFINE_LOG_CATEGORY_STATIC(LogHelicopterFDM, Log, All);

namespace
{
	constexpr double HDegToRad = UE_DOUBLE_PI / 180.0;

	// Universal physics constants — same for any helicopter, no need to expose to BP.
	constexpr double ETLOnsetMu                 = 0.04;  // ~15 knots
	constexpr double ETLFullMu                  = 0.10;  // ~30 knots
	constexpr double TailAutoTrimResponseTime   = 0.4;   // s — yaw trim catch-up time
	constexpr double TrimDecayTime              = 0.5;   // s — trim decay when engine off
	constexpr double VRSDescentThreshold        = 7.0;   // m/s — descent rate where VRS starts (~1400 ft/min)
	constexpr double VRSDescentSaturation       = 12.0;  // m/s — descent rate of full VRS severity
	constexpr double VRSForwardSafeSpeed        = 8.0;   // m/s — forward speed that breaks VRS

	// Damping gains — game-feel, not vehicle-specific.
	constexpr double VerticalDampingGain        = 2.0;
	constexpr double LateralDampingGain         = 2.0;
	constexpr double BackwardDampingGain        = 1.5;

	// SAS internals — produce stable behavior across the supported airframe range.
	constexpr double MinCyclicAuthorityFraction = 0.15;  // floor on cyclic at low thrust
	constexpr double MaxCyclicAcceleration      = 3.0;   // rad/s² cap on cyclic-induced angular accel
	constexpr double AttitudeHoldDeadzone       = 0.1;   // cyclic input deadzone for attitude hold
	constexpr double AttitudeHoldProgressive    = 1.5;   // quadratic restoring gain for large angles
	constexpr double EnvelopeStrength           = 8.0;   // envelope-protection cubic pushback gain

	// Universal physical constants — same for any helicopter.
	constexpr double TorqueFraction             = 0.07;  // fraction of main thrust as reactive torque
	constexpr double TailRotorThrustCoefficient = 0.008; // CT for tail rotor thrust formula

	// Ground line-trace cadence.
	constexpr float  GroundTraceInterval        = 0.2f;
}

// ========================================================================
// Constructor
// ========================================================================

UHelicopterFDMComponent::UHelicopterFDMComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

// ========================================================================
// Lifecycle
// ========================================================================

void UHelicopterFDMComponent::BeginPlay()
{
	Super::BeginPlay();

	ResolvePhysicsBody();

	if (PhysicsComponent)
	{
		StartingZCm = PhysicsComponent->GetComponentLocation().Z;
	}

	bEngineRunning = false;
	EngineRPM_Internal = 0.0;
	MainRotorRPM_Internal = 0.0;
	GovernorIntegral = 0.0;
	TailTrimYawMoment = 0.0;
	AdvanceRatioCached = 0.0;
}

void UHelicopterFDMComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (bEnabled && !bPhysicsApplied && PhysicsComponent)
	{
		if (!PhysicsComponent->IsSimulatingPhysics())
		{
			PhysicsComponent->SetSimulatePhysics(true);
		}
		ConfigureBodyInstance();
		bPhysicsApplied = true;
	}

	if (bEnabled && PhysicsComponent && PhysicsComponent->IsSimulatingPhysics())
	{
		CurrentDeltaTime = DeltaTime;
		FilterControlInputs(DeltaTime);
		TickPowerplant(DeltaTime);
		SolveRotorDisc();
		CommitForcesAndTorques();
	}
}

// ========================================================================
// Initialization
// ========================================================================

void UHelicopterFDMComponent::ResolvePhysicsBody()
{
	AActor* Owner = GetOwner();
	if (!Owner) return;

	PhysicsComponent = Cast<UPrimitiveComponent>(Owner->GetRootComponent());

	if (!PhysicsComponent)
	{
		UE_LOG(LogHelicopterFDM, Error, TEXT("Root component is not a UPrimitiveComponent — FDM disabled"));
	}
}

void UHelicopterFDMComponent::ConfigureBodyInstance()
{
	if (!PhysicsComponent) return;

	MassKg = static_cast<double>(PhysicsComponent->GetMass());

	FBodyInstance* BodyInstance = PhysicsComponent->GetBodyInstance();
	if (BodyInstance)
	{
		// Chaos inertia is in kg*cm^2, our InertiaTensor is in kg*m^2
		FVector DesiredInertiaCm2(
			InertiaTensor.X * 10000.0,
			InertiaTensor.Y * 10000.0,
			InertiaTensor.Z * 10000.0
		);

		FVector CurrentInertia = BodyInstance->GetBodyInertiaTensor();

		FVector Scale(
			(CurrentInertia.X > 0.01) ? DesiredInertiaCm2.X / CurrentInertia.X : 1.0,
			(CurrentInertia.Y > 0.01) ? DesiredInertiaCm2.Y / CurrentInertia.Y : 1.0,
			(CurrentInertia.Z > 0.01) ? DesiredInertiaCm2.Z / CurrentInertia.Z : 1.0
		);

		BodyInstance->InertiaTensorScale = Scale;
		BodyInstance->UpdateMassProperties();
	}
}

// ========================================================================
// Engine Control API
// ========================================================================

void UHelicopterFDMComponent::StartEngine()
{
	if (!bEngineRunning)
	{
		bEngineRunning = true;
		bEngineSpooling = true;
		EngineSpoolTimer = 0.0;
		EngineSpoolStartRPM = EngineRPM_Internal;
		GovernorIntegral = 0.0;

		if (PhysicsComponent)
		{
			StartingZCm = PhysicsComponent->GetComponentLocation().Z;
		}
	}
}

void UHelicopterFDMComponent::StopEngine()
{
	if (bEngineRunning)
	{
		bEngineRunning = false;
		bEngineSpooling = true;
		EngineSpoolTimer = 0.0;
		EngineSpoolStartRPM = EngineRPM_Internal;
	}
}

// ========================================================================
// Control Input API
// ========================================================================

void UHelicopterFDMComponent::SetControls(const FHelicopterControls& NewControls)
{
	Controls.Elevator = FMath::Clamp(NewControls.Elevator * InputTuning.CyclicSensitivityPitch, -1.0f, 1.0f);
	Controls.Aileron  = FMath::Clamp(NewControls.Aileron  * InputTuning.CyclicSensitivityRoll,  -1.0f, 1.0f);
	Controls.Throttle = FMath::Clamp(NewControls.Throttle * InputTuning.CollectiveSensitivity,    0.0f, 1.0f);
	Controls.Rudder   = FMath::Clamp(NewControls.Rudder   * InputTuning.PedalSensitivity,       -1.0f, 1.0f);
}

void UHelicopterFDMComponent::SetCyclicLongitudinal(float Value)
{
	Controls.Elevator = FMath::Clamp(Value * InputTuning.CyclicSensitivityPitch, -1.0f, 1.0f);
}

void UHelicopterFDMComponent::SetCyclicLateral(float Value)
{
	Controls.Aileron = FMath::Clamp(Value * InputTuning.CyclicSensitivityRoll, -1.0f, 1.0f);
}

void UHelicopterFDMComponent::SetCollective(float Value)
{
	Controls.Throttle = FMath::Clamp(Value * InputTuning.CollectiveSensitivity, 0.0f, 1.0f);
}

void UHelicopterFDMComponent::SetPedals(float Value)
{
	Controls.Rudder = FMath::Clamp(Value * InputTuning.PedalSensitivity, -1.0f, 1.0f);
}

void UHelicopterFDMComponent::FilterControlInputs(float DeltaTime)
{
	double dt = static_cast<double>(DeltaTime);

	auto AlphaFromTau = [dt](float Tau) {
		return 1.0 - FMath::Exp(-dt / static_cast<double>(FMath::Max(Tau, 0.01f)));
	};

	// Pitch: asymmetric — slow attack, fast release. Other axes share one symmetric tau.
	double TargetElev = static_cast<double>(Controls.Elevator);
	bool   bReleasingPitch = FMath::Abs(TargetElev) < FMath::Abs(SmoothedElevator);
	double AlphaElevator = AlphaFromTau(bReleasingPitch ? InputTuning.SmoothingPitchRelease : InputTuning.SmoothingPitch);
	double AlphaOther    = AlphaFromTau(InputTuning.SmoothingOther);

	SmoothedElevator += (TargetElev                             - SmoothedElevator) * AlphaElevator;
	SmoothedAileron  += (static_cast<double>(Controls.Aileron)  - SmoothedAileron)  * AlphaOther;
	SmoothedThrottle += (static_cast<double>(Controls.Throttle) - SmoothedThrottle) * AlphaOther;
	SmoothedRudder   += (static_cast<double>(Controls.Rudder)   - SmoothedRudder)   * AlphaOther;
}

// ========================================================================
// Flight State API
// ========================================================================

float UHelicopterFDMComponent::GetAirspeed() const
{
	if (!PhysicsComponent) return 0.0f;
	FVector V = PhysicsComponent->GetPhysicsLinearVelocity() / 100.0f;
	return static_cast<float>(V.Size() * 1.94384);
}

float UHelicopterFDMComponent::GetAltitude() const
{
	if (!PhysicsComponent) return FieldElevationFeet;
	return FieldElevationFeet + static_cast<float>((PhysicsComponent->GetComponentLocation().Z - StartingZCm) / 30.48);
}

float UHelicopterFDMComponent::GetVerticalSpeed() const
{
	if (!PhysicsComponent) return 0.0f;
	return static_cast<float>(PhysicsComponent->GetPhysicsLinearVelocity().Z / 100.0 * 196.85);
}

float UHelicopterFDMComponent::GetRotorRPMPercent() const
{
	return (MainRotorRPMNominal > 0.0) ? static_cast<float>(MainRotorRPM_Internal / MainRotorRPMNominal * 100.0) : 0.0f;
}

float UHelicopterFDMComponent::GetEngineRPMPercent() const
{
	double NominalEngineRPM = MainRotorRPMNominal * EngineToRotorRatio;
	return (NominalEngineRPM > 0.0) ? static_cast<float>(EngineRPM_Internal / NominalEngineRPM * 100.0) : 0.0f;
}

float UHelicopterFDMComponent::GetMainRotorThrust() const
{
	return static_cast<float>(MainRotorThrust);
}

float UHelicopterFDMComponent::GetTailRotorRPMPercent() const
{
	double NominalTailRPM = MainRotorRPMNominal * TailRotorGearRatio;
	double CurrentTailRPM = MainRotorRPM_Internal * TailRotorGearRatio;
	return (NominalTailRPM > 0.0) ? static_cast<float>(CurrentTailRPM / NominalTailRPM * 100.0) : 0.0f;
}

float UHelicopterFDMComponent::GetMainRotorRPM() const
{
	return static_cast<float>(MainRotorRPM_Internal);
}

float UHelicopterFDMComponent::GetTailRotorRPM() const
{
	return static_cast<float>(MainRotorRPM_Internal * TailRotorGearRatio);
}

float UHelicopterFDMComponent::GetPitch() const
{
	if (!PhysicsComponent) return 0.0f;
	return PhysicsComponent->GetComponentRotation().Pitch;
}

float UHelicopterFDMComponent::GetRoll() const
{
	if (!PhysicsComponent) return 0.0f;
	return -PhysicsComponent->GetComponentRotation().Roll;
}

float UHelicopterFDMComponent::GetHeading() const
{
	if (!PhysicsComponent) return 0.0f;
	float Yaw = PhysicsComponent->GetComponentRotation().Yaw;
	return (Yaw < 0.0f) ? Yaw + 360.0f : Yaw;
}

float UHelicopterFDMComponent::GetGroundHeight() const
{
	return static_cast<float>(GroundHeight);
}

// ========================================================================
// Engine Simulation & Governor
// ========================================================================

void UHelicopterFDMComponent::TickPowerplant(float DeltaTime)
{
	double NominalEngineRPM = MainRotorRPMNominal * EngineToRotorRatio;
	double dt = static_cast<double>(DeltaTime);

	// Spool up/down: smoothstep curves
	if (bEngineSpooling)
	{
		double SpoolTime = bEngineRunning
			? FMath::Max(static_cast<double>(EngineSpoolUpTime), 0.1)
			: FMath::Max(static_cast<double>(EngineSpoolDownTime), 0.1);

		EngineSpoolTimer += dt;
		double T = FMath::Clamp(EngineSpoolTimer / SpoolTime, 0.0, 1.0);

		if (bEngineRunning)
		{
			// Spool up: smoothstep 3T²-2T³
			double S = T * T * (3.0 - 2.0 * T);
			double TargetRPM = NominalEngineRPM;
			EngineRPM_Internal = EngineSpoolStartRPM + (TargetRPM - EngineSpoolStartRPM) * S;
		}
		else
		{
			// Spool down: 1-smoothstep
			double S = T * T * (3.0 - 2.0 * T);
			EngineRPM_Internal = EngineSpoolStartRPM * (1.0 - S);
		}

		if (T >= 1.0)
		{
			bEngineSpooling = false;
		}
	}

	// Governor: PID-style with collective feedforward
	if (bEngineRunning && !bEngineSpooling)
	{
		double EngineOmega = EngineRPM_Internal * UE_DOUBLE_PI / 30.0;
		if (EngineOmega > 1.0)
		{
			double RotorRPM = EngineRPM_Internal / EngineToRotorRatio;
			double Error = 1.0 - (RotorRPM / MainRotorRPMNominal);
			GovernorIntegral += Error * dt;
			GovernorIntegral = FMath::Clamp(GovernorIntegral, -2.0, 2.0);

			constexpr double Kp = 3.0;
			constexpr double Ki = 0.8;
			constexpr double CollectiveFeedforward = 0.5;
			double GovernorThrottle = FMath::Clamp(Kp * Error + Ki * GovernorIntegral + CollectiveFeedforward * SmoothedThrottle, 0.0, 1.0);

			double EngTorque = EngineMaxPowerHP * 746.0 * GovernorThrottle / EngineOmega * GetAltitudeFactor();
			double RotorLoadTorque = FMath::Abs(MainRotorTorque / EngineToRotorRatio);
			double NetTorque = EngTorque - RotorLoadTorque;

			EngineRPM_Internal += (NetTorque / EngineMomentOfInertia) * dt * 30.0 / UE_DOUBLE_PI;
		}
	}

	EngineRPM_Internal = FMath::Clamp(EngineRPM_Internal, 0.0, NominalEngineRPM * 1.1);
	MainRotorRPM_Internal = EngineRPM_Internal / EngineToRotorRatio;
}

// ========================================================================
// Main Rotor — Thrust & Torque
// ========================================================================

void UHelicopterFDMComponent::SolveRotorDisc()
{
	double RpmFactor = FMath::Clamp(MainRotorRPM_Internal / MainRotorRPMNominal, 0.0, 1.2);

	if (RpmFactor < 0.01)
	{
		MainRotorThrust = 0.0;
		MainRotorTorque = 0.0;
		AdvanceRatioCached = 0.0;
		return;
	}

	// Cyclic disc tilt — compute early so auto-comp can account for cyclic-induced disc tilt.
	double MaxTilt = CyclicMaxTilt * HDegToRad;
	CyclicLon = SmoothedElevator * MaxTilt;
	CyclicLat = SmoothedAileron * MaxTilt;

	// Cache velocity components — used by ETL, VRS, and stored for later steps.
	FVector LinVel = PhysicsComponent ? PhysicsComponent->GetPhysicsLinearVelocity() : FVector::ZeroVector;
	FVector BodyFwdW = GetOwner() ? GetOwner()->GetActorRotation().RotateVector(FVector::ForwardVector) : FVector::ForwardVector;
	double VelFwdMs    = static_cast<double>(FVector::DotProduct(LinVel, BodyFwdW)) / 100.0;
	double VHorizMs    = FMath::Sqrt(LinVel.X * LinVel.X + LinVel.Y * LinVel.Y) / 100.0;
	double VDescentMs  = FMath::Max(-static_cast<double>(LinVel.Z) / 100.0, 0.0);

	// ETL: rotor escapes own downwash at ~15 knots → +10–15% thrust available.
	// Felt as a satisfying acceleration kick when transitioning from hover to forward flight.
	double TipSpeed = (MainRotorRPM_Internal / 60.0) * 2.0 * UE_DOUBLE_PI * MainRotor.Radius;
	AdvanceRatioCached = (TipSpeed > 1.0) ? (VelFwdMs / TipSpeed) : 0.0;

	double ETLFactor = 1.0;
	if (AdvanceRatioCached > ETLOnsetMu)
	{
		double T = FMath::Clamp((AdvanceRatioCached - ETLOnsetMu) / (ETLFullMu - ETLOnsetMu), 0.0, 1.0);
		double Curve = T * T * (3.0 - 2.0 * T);
		ETLFactor = 1.0 + static_cast<double>(ETLBoost) * Curve;
	}

	// Vortex Ring State: descending fast at low forward speed → rotor in own wake → thrust collapses.
	// Recovery: drop nose, gain forward speed past VRSForwardSafeSpeed.
	double VRSFactor = 1.0;
	if (bEnableVRS && VDescentMs > VRSDescentThreshold && VHorizMs < VRSForwardSafeSpeed)
	{
		double SeverityT = FMath::Clamp((VDescentMs - VRSDescentThreshold) / (VRSDescentSaturation - VRSDescentThreshold), 0.0, 1.0);
		double IsolationT = 1.0 - VHorizMs / VRSForwardSafeSpeed;
		VRSFactor = 1.0 - static_cast<double>(VRSStrength) * SeverityT * IsolationT;
	}

	double Weight = MassKg * Gravity;
	double RawThrust = SmoothedThrottle * RpmFactor * RpmFactor * Weight * ThrustMultiplier;
	RawThrust *= EvaluateIGEFactor() * GetAltitudeFactor() * ETLFactor * VRSFactor;

	// Note: auto-collective compensation is NOT applied here. It's applied at the moment of force
	// application in CommitForcesAndTorques [A] — that way it reacts instantly to body orientation
	// changes without lag from ThrustResponseTime smoothing.

	if (ThrustResponseTime > 0.001)
	{
		double SmoothAlpha = 1.0 - FMath::Exp(-static_cast<double>(CurrentDeltaTime) / ThrustResponseTime);
		MainRotorThrust += (RawThrust - MainRotorThrust) * SmoothAlpha;
	}
	else
	{
		MainRotorThrust = RawThrust;
	}

	MainRotorTorque = MainRotorThrust * TorqueFraction * MainRotor.Radius;
}

// ========================================================================
// Force Application
// ========================================================================

void UHelicopterFDMComponent::CommitForcesAndTorques()
{
	if (!PhysicsComponent || !PhysicsComponent->IsSimulatingPhysics()) return;

	FBodyInstance* BodyInst = PhysicsComponent->GetBodyInstance();
	if (!BodyInst) return;

	FVector LinVelocity = PhysicsComponent->GetPhysicsLinearVelocity();
	FVector AngVelocity = PhysicsComponent->GetPhysicsAngularVelocityInRadians();

	FRotator ActorRot = GetOwner()->GetActorRotation();
	FVector BodyFwd   = ActorRot.RotateVector(FVector::ForwardVector);
	FVector BodyRight = ActorRot.RotateVector(FVector::RightVector);
	FVector BodyUp    = ActorRot.RotateVector(FVector::UpVector);

	FVector BodyInertia = BodyInst->GetBodyInertiaTensor();
	double Ixx = FMath::Max(static_cast<double>(BodyInertia.X) / 10000.0, 100.0);
	double Iyy = FMath::Max(static_cast<double>(BodyInertia.Y) / 10000.0, 100.0);
	double Izz = FMath::Max(static_cast<double>(BodyInertia.Z) / 10000.0, 100.0);

	double RpmFactor = FMath::Clamp(MainRotorRPM_Internal / MainRotorRPMNominal, 0.0, 1.2);
	double Thrust = MainRotorThrust;

	FVector TotalForce = FVector::ZeroVector;
	FVector TotalTorque = FVector::ZeroVector;

	double VzMs = static_cast<double>(LinVelocity.Z) / 100.0;

	double PitchRate = static_cast<double>(FVector::DotProduct(AngVelocity, BodyRight));
	double RollRate  = static_cast<double>(FVector::DotProduct(AngVelocity, BodyFwd));
	double YawRate   = static_cast<double>(FVector::DotProduct(AngVelocity, BodyUp));

	FRotator YawOnly(0.0f, ActorRot.Yaw, 0.0f);
	FVector YawForward = YawOnly.RotateVector(FVector::ForwardVector);
	FVector YawRight   = YawOnly.RotateVector(FVector::RightVector);

	double PitchSin = FMath::Clamp( static_cast<double>(FVector::DotProduct(BodyUp, YawForward)), -1.0, 1.0);
	double RollSin  = FMath::Clamp(-static_cast<double>(FVector::DotProduct(BodyUp, YawRight)),   -1.0, 1.0);
	double CurrentPitch = FMath::Asin(PitchSin);
	double CurrentRoll  = FMath::Asin(RollSin);

	// [A] Disc thrust + cyclic tilt moment
	{
		double CosLon = FMath::Cos(CyclicLon);
		double CosLat = FMath::Cos(CyclicLat);
		double Nx = FMath::Sin(CyclicLon);
		double Ny = FMath::Sin(CyclicLat);
		double Nz = CosLon * CosLat;

		double Norm = FMath::Sqrt(Nx * Nx + Ny * Ny + Nz * Nz);
		if (Norm > 0.001)
		{
			Nx /= Norm;
			Ny /= Norm;
			Nz /= Norm;
		}

		FVector DiscNormalWorld =
			BodyFwd   * static_cast<float>(Nx) +
			BodyRight * static_cast<float>(Ny) +
			BodyUp    * static_cast<float>(Nz);

		// Auto-collective compensation: scale thrust so vertical force ≥ weight when pilot holds
		// hover throttle, regardless of body+cyclic tilt or ETL/VRS dips during turns.
		// Applied here (not in SolveRotorDisc) for instant response without ThrustResponseTime lag.
		double AppliedThrust = Thrust;
		if (bAutoCollectiveCompensation)
		{
			double VertFactor = FMath::Max(static_cast<double>(DiscNormalWorld.Z), 0.3);
			double TargetVertical = Thrust;

			// Gate on pilot's *commanded* thrust (throttle × multiplier × altitude — no rpm²),
			// so RPM droop from a weak engine doesn't lock auto-comp out. The cap below
			// (MaxAvailable, with rpm²) honors real physics: if the rotor literally can't
			// produce enough thrust, we don't fake it.
			double Weight = MassKg * Gravity;
			double CommandedThrust = SmoothedThrottle * ThrustMultiplier * GetAltitudeFactor() * Weight;
			if (CommandedThrust >= Weight * 0.85)
			{
				double MaxAvailable = Weight * ThrustMultiplier * GetAltitudeFactor() * RpmFactor * RpmFactor;
				double Floor = FMath::Min(Weight, MaxAvailable);
				TargetVertical = FMath::Max(TargetVertical, Floor);
			}

			AppliedThrust = TargetVertical / VertFactor;
		}

		TotalForce += DiscNormalWorld * static_cast<float>(AppliedThrust * 100.0);

		double HubZ = MainRotor.Position.Z / 100.0;
		if (FMath::Abs(HubZ) > 0.01)
		{
			double Weight = MassKg * Gravity;
			double MinCyclicThrust = static_cast<double>(MinCyclicAuthorityFraction) * Weight * RpmFactor;
			double CyclicThrust = FMath::Max(Thrust, MinCyclicThrust);

			// Sign verified: W (cyclic forward, +CyclicLon) → +TauPitch → +Y torque → nose down (matches real heli).
			// Convention check: attitude hold for nose-up state produces +Y torque to restore level → so +Y = nose down.
			double TauRoll  = HubZ * CyclicThrust * FMath::Sin(CyclicLat);
			double TauPitch = HubZ * CyclicThrust * FMath::Sin(CyclicLon);

			double MaxAccel = static_cast<double>(MaxCyclicAcceleration);
			double MaxTauPitch = MaxAccel * Iyy;
			double MaxTauRoll  = MaxAccel * Ixx;
			if (MaxTauPitch > 0.01) TauPitch = MaxTauPitch * FMath::Tanh(TauPitch / MaxTauPitch);
			if (MaxTauRoll  > 0.01) TauRoll  = MaxTauRoll  * FMath::Tanh(TauRoll  / MaxTauRoll);

			TotalTorque +=
				BodyFwd   * static_cast<float>(TauRoll  * 10000.0) +
				BodyRight * static_cast<float>(TauPitch * 10000.0);
		}
	}

	// [A2] Climb rate soft cap — power-equivalent limit (descent is now governed by VRS instead).
	{
		if (VzMs > static_cast<double>(MaxClimbRate))
		{
			double VerticalDamping = -(VzMs - static_cast<double>(MaxClimbRate)) * static_cast<double>(VerticalDampingGain) * MassKg;
			TotalForce.Z += static_cast<float>(VerticalDamping * 100.0);
		}
	}

	// [A3] Lateral + backward velocity damping — projected to world-horizontal so
	// that vertical velocity in a banked attitude doesn't leak into "lateral slip"
	// and pull the heli down through banked maneuvers.
	{
		FVector LinVelHoriz(LinVelocity.X, LinVelocity.Y, 0.0f);
		double VelLatHoriz = static_cast<double>(FVector::DotProduct(LinVelHoriz, YawRight))   / 100.0;
		double VelFwdHoriz = static_cast<double>(FVector::DotProduct(LinVelHoriz, YawForward)) / 100.0;

		double LatForce = -VelLatHoriz * static_cast<double>(LateralDampingGain) * MassKg;

		double BackForce = 0.0;
		if (VelFwdHoriz < -1.0)
		{
			BackForce = -VelFwdHoriz * static_cast<double>(BackwardDampingGain) * MassKg;
		}

		TotalForce +=
			YawRight   * static_cast<float>(LatForce  * 100.0) +
			YawForward * static_cast<float>(BackForce * 100.0);
	}

	// [Hull] Parasitic drag — body-frame anisotropic + aerodynamic yaw stability.
	// Fuselage produces ~3× more drag laterally than forward; weather-cock effect aligns nose
	// with airflow in forward flight (felt as skid arrest during pedal turns).
	{
		double VelFwdB   = static_cast<double>(FVector::DotProduct(LinVelocity, BodyFwd))   / 100.0;
		double VelRightB = static_cast<double>(FVector::DotProduct(LinVelocity, BodyRight)) / 100.0;
		double VelUpB    = static_cast<double>(FVector::DotProduct(LinVelocity, BodyUp))    / 100.0;

		double CdA_Total = HullDragCdA + LandingGearDragCdA;
		double LatMult = static_cast<double>(LateralDragMultiplier);

		// F = -0.5·ρ·v·|v|·CdA per body axis (signed quadratic preserves direction).
		double DragFwd   = -0.5 * AirDensity * VelFwdB   * FMath::Abs(VelFwdB)   * CdA_Total;
		double DragRight = -0.5 * AirDensity * VelRightB * FMath::Abs(VelRightB) * CdA_Total * LatMult;
		double DragUp    = -0.5 * AirDensity * VelUpB    * FMath::Abs(VelUpB)    * CdA_Total;

		TotalForce +=
			BodyFwd   * static_cast<float>(DragFwd   * 100.0) +
			BodyRight * static_cast<float>(DragRight * 100.0) +
			BodyUp    * static_cast<float>(DragUp    * 100.0);

		// Weather-cock yaw — restoring moment ∝ (forward speed × lateral velocity).
		// Signed VelFwdB gives correct sign for both forward and tail-first flight.
		// Geometry term R · TailArm proxies fin area × moment arm.
		if (SideslipYawGain > 0.0f && FMath::Abs(VelFwdB) > 1.0)
		{
			double TailArmM = FMath::Abs(TailRotor.Position.X) / 100.0;
			double SideslipMoment = 0.5 * AirDensity * VelFwdB * VelRightB *
				static_cast<double>(SideslipYawGain) * MainRotor.Radius * TailArmM;
			TotalTorque += BodyUp * static_cast<float>(SideslipMoment * 10000.0);
		}
	}

	// [B+C] Reactive torque + auto-trim + pedal yaw + translating tendency
	{
		double TailArm = FMath::Abs(TailRotor.Position.X) / 100.0;
		double TrimDt = static_cast<double>(CurrentDeltaTime);

		// Newton's 3rd: body experiences torque opposite the engine torque on the rotor.
		// CW main rotor (Mil/Eurocopter) → body gets +Z reactive (right yaw).
		// CCW main rotor (Bell/US)       → body gets -Z reactive (left yaw).
		const double DirSign = bMainRotorClockwise ? +1.0 : -1.0;
		double ReactiveYawMoment = DirSign * MainRotorTorque * RpmFactor;

		// Auto-trim moment converges to whatever cancels the reactive torque in steady state.
		// τ ~0.4s leaves a brief, visible yaw transient on rapid collective changes.
		if (bEnableAutoTrim && RpmFactor > 0.1)
		{
			double TrimAlpha = 1.0 - FMath::Exp(-TrimDt / TailAutoTrimResponseTime);
			double DesiredTrim = -ReactiveYawMoment;
			TailTrimYawMoment += (DesiredTrim - TailTrimYawMoment) * TrimAlpha;
		}
		else
		{
			TailTrimYawMoment *= FMath::Exp(-TrimDt / TrimDecayTime);
		}

		// Pilot pedal — physical tail rotor thrust: T = ρ · A_disc · V_tip² · CT · pedal · authority.
		// Yaw moment and translating tendency both scale from the same true thrust, so falling RPM
		// (V_tip² drop) attenuates both naturally without ad-hoc RpmFactor multipliers.
		double TailOmega    = MainRotorRPM_Internal * TailRotorGearRatio * UE_DOUBLE_PI / 30.0;
		double TailTipSpeed = TailOmega * TailRotor.Radius;
		double TailDiscArea = UE_DOUBLE_PI * TailRotor.Radius * TailRotor.Radius;
		double TailMaxThrust = AirDensity * TailDiscArea * TailTipSpeed * TailTipSpeed * TailRotorThrustCoefficient;
		double TailThrust    = SmoothedRudder * PedalYawAuthority * TailMaxThrust;

		double PilotMoment = TailThrust * TailArm;

		double TotalYawMoment = ReactiveYawMoment + TailTrimYawMoment + PilotMoment;
		TotalTorque += BodyUp * static_cast<float>(TotalYawMoment * 10000.0);

		// CW main → tail pushes body RIGHT for right pedal. CCW → pushes LEFT.
		double LateralForceN = bEnableTranslatingTendency ? (TailThrust * DirSign) : 0.0;
		TotalForce += BodyRight * static_cast<float>(LateralForceN * 100.0);
	}

	// [D] SAS rate damping
	{
		double TauPitch = -static_cast<double>(SAS_RateDampingPitch) * PitchRate * Iyy * RpmFactor;
		double TauRoll  = -static_cast<double>(SAS_RateDampingRoll)  * RollRate  * Ixx * RpmFactor;
		double TauYaw   = -static_cast<double>(SAS_RateDampingYaw)   * YawRate   * Izz * RpmFactor;

		TotalTorque +=
			BodyRight * static_cast<float>(TauPitch * 10000.0) +
			BodyFwd   * static_cast<float>(TauRoll  * 10000.0) +
			BodyUp    * static_cast<float>(TauYaw   * 10000.0);
	}

	// [E] SAS attitude hold + [F] Envelope protection (shared pitch/roll angles)
	{
		double AttTauPitch = 0.0;
		double AttTauRoll  = 0.0;

		// Attitude hold split into two layers:
		//   1. Base (linear in angle) — fades with stick input. "Auto-level when neutral."
		//   2. Progressive (quadratic in angle) — always active. "The more tilted, the stronger the pull back."
		// Result: pilot holds attitude with stick, but big tilts still get progressive resistance.
		constexpr double AttitudeHoldFadeRange = 0.4;
		double CyclicMag = FMath::Max(FMath::Abs(SmoothedElevator), FMath::Abs(SmoothedAileron));
		double FadeT = FMath::Clamp(
			(CyclicMag - AttitudeHoldDeadzone) / AttitudeHoldFadeRange,
			0.0, 1.0);
		double HoldWeight = 1.0 - FadeT * FadeT * (3.0 - 2.0 * FadeT);  // 1.0 at neutral → 0.0 at full stick

		if (RpmFactor > 0.1)
		{
			double K = static_cast<double>(SAS_AttitudeHoldStrength);
			double Progressive = AttitudeHoldProgressive;

			// Base — linear restoring torque, fades with stick
			AttTauPitch = -K * CurrentPitch * Iyy * RpmFactor * HoldWeight;
			AttTauRoll  = -K * CurrentRoll  * Ixx * RpmFactor * HoldWeight;

			// Progressive — quadratic in angle (|x|·x is signed-quadratic), always active.
			// At small angles negligible; at large angles dominates and resists further tilt.
			AttTauPitch += -K * Progressive * FMath::Abs(CurrentPitch) * CurrentPitch * Iyy * RpmFactor;
			AttTauRoll  += -K * Progressive * FMath::Abs(CurrentRoll)  * CurrentRoll  * Ixx * RpmFactor;
		}

		// Envelope protection: onset at 80%, cubic pushback T³
		double MaxPitchRad = static_cast<double>(EnvelopeMaxPitch) * HDegToRad;
		double MaxRollRad  = static_cast<double>(EnvelopeMaxRoll)  * HDegToRad;
		double EnvK = EnvelopeStrength;
		constexpr double OnsetFrac = 0.8;

		double AbsPitch = FMath::Abs(CurrentPitch);
		double PitchOnset = MaxPitchRad * OnsetFrac;
		if (AbsPitch > PitchOnset)
		{
			double T = FMath::Clamp((AbsPitch - PitchOnset) / (MaxPitchRad - PitchOnset), 0.0, 2.0);
			double Sign = (CurrentPitch > 0.0) ? 1.0 : -1.0;
			AttTauPitch += -Sign * EnvK * T * T * T * Iyy * FMath::Max(RpmFactor, 0.3);
		}

		double AbsRoll = FMath::Abs(CurrentRoll);
		double RollOnset = MaxRollRad * OnsetFrac;
		if (AbsRoll > RollOnset)
		{
			double T = FMath::Clamp((AbsRoll - RollOnset) / (MaxRollRad - RollOnset), 0.0, 2.0);
			double Sign = (CurrentRoll > 0.0) ? 1.0 : -1.0;
			AttTauRoll += -Sign * EnvK * T * T * T * Ixx * FMath::Max(RpmFactor, 0.3);
		}

		TotalTorque +=
			BodyRight * static_cast<float>(AttTauPitch * 10000.0) +
			BodyFwd   * static_cast<float>(AttTauRoll  * 10000.0);
	}

	// Apply all accumulated forces and torques in one call each
	PhysicsComponent->AddForce(TotalForce);
	PhysicsComponent->AddTorqueInRadians(TotalTorque);
}

// ========================================================================
// Atmosphere — exponential thrust/power decay with altitude
// ========================================================================

double UHelicopterFDMComponent::GetAltitudeFactor() const
{
	double AltitudeFt = static_cast<double>(FieldElevationFeet);
	if (PhysicsComponent)
	{
		AltitudeFt += (PhysicsComponent->GetComponentLocation().Z - StartingZCm) / 30.48;
	}
	double Ceiling = FMath::Max(static_cast<double>(ServiceCeilingFeet), 100.0);
	// exp(-h / ceiling): 1.0 at sea level → 0.37 at ceiling → asymptotic decay above
	return FMath::Exp(-AltitudeFt / Ceiling);
}

// ========================================================================
// Ground Effect — Cheeseman-Bennett (1955)
// ========================================================================

double UHelicopterFDMComponent::EvaluateIGEFactor()
{
	GroundTraceTimer += CurrentDeltaTime;
	if (GroundTraceTimer >= GroundTraceInterval && GetWorld() && PhysicsComponent)
	{
		GroundTraceTimer = 0.0f;

		FVector Start = PhysicsComponent->GetComponentLocation();
		FVector End = Start - FVector(0.0f, 0.0f, 15000.0f);
		FHitResult Hit;
		FCollisionQueryParams Params;
		Params.AddIgnoredActor(GetOwner());

		if (GetWorld()->LineTraceSingleByChannel(Hit, Start, End, ECC_WorldStatic, Params))
		{
			GroundHeight = Hit.Distance / 100.0;
		}
		else
		{
			GroundHeight = 1000.0;
		}
	}

	double H = GroundHeight;
	double R = MainRotor.Radius;

	// Cheeseman-Bennett: T_IGE/T_OGE = 1 / (1 - (R/(4H))^2)
	if (H < 0.1 || R < 0.1)
	{
		return 1.0;
	}

	double Ratio = R / (4.0 * H);
	double RatioSq = Ratio * Ratio;

	if (RatioSq >= 1.0)
	{
		return 1.5;
	}

	double GeFactor = 1.0 / (1.0 - RatioSq);

	return FMath::Clamp(GeFactor, 1.0, 1.5);
}
