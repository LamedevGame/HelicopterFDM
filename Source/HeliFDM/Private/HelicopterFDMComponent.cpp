#include "HelicopterFDMComponent.h"
#include "Components/PrimitiveComponent.h"
#include "GameFramework/Actor.h"
#include "Engine/World.h"

DEFINE_LOG_CATEGORY_STATIC(LogHelicopterFDM, Log, All);

namespace
{
	constexpr double HDegToRad = UE_DOUBLE_PI / 180.0;
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

	UE_LOG(LogHelicopterFDM, Log, TEXT("BeginPlay on %s"), *GetOwner()->GetName());

	ResolvePhysicsBody();

	if (PhysicsComponent)
	{
		StartingZCm = PhysicsComponent->GetComponentLocation().Z;
		UE_LOG(LogHelicopterFDM, Log, TEXT("  PhysicsComponent: %s (SimPhysics=%d)"),
			*PhysicsComponent->GetName(), PhysicsComponent->IsSimulatingPhysics());
	}
	else
	{
		UE_LOG(LogHelicopterFDM, Error, TEXT("  No physics component found!"));
	}

	// Engine starts off
	bEngineRunning = false;
	EngineRPM_Internal = 0.0;
	MainRotorRPM_Internal = 0.0;
	GovernorIntegral = 0.0;

	UE_LOG(LogHelicopterFDM, Log, TEXT("  MainRotor R=%.1f m, Engine MaxHP=%.0f"),
		MainRotor.Radius, EngineMaxPowerHP);
}

void UHelicopterFDMComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (bEnabled && !bPhysicsApplied && PhysicsComponent)
	{
		if (!PhysicsComponent->IsSimulatingPhysics())
		{
			UE_LOG(LogHelicopterFDM, Log, TEXT("Enabling physics simulation"));
			PhysicsComponent->SetSimulatePhysics(true);
		}
		ConfigureBodyInstance();
		bPhysicsApplied = true;
		UE_LOG(LogHelicopterFDM, Log, TEXT("Physics applied: Mass=%.0f kg"), MassKg);
	}

	if (bEnabled && PhysicsComponent && PhysicsComponent->IsSimulatingPhysics())
	{
		CurrentDeltaTime = DeltaTime;
		FilterControlInputs(DeltaTime);
		TickPowerplant(DeltaTime);
		SolveRotorDisc();
		CommitForcesAndTorques();

		// Periodic telemetry
		TelemetryTimer += DeltaTime;
		if (TelemetryTimer >= 3.0)
		{
			TelemetryTimer = 0.0;
			UE_LOG(LogHelicopterFDM, Log,
				TEXT("RPM=%.0f | Thrust=%.0fN | Collective=%.2f | Cyclic=(%.2f,%.2f) | Pedal=%.2f | AGL=%.1fm"),
				MainRotorRPM_Internal, MainRotorThrust,
				SmoothedThrottle, SmoothedElevator, SmoothedAileron,
				SmoothedRudder, GroundHeight);
		}
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

		UE_LOG(LogHelicopterFDM, Log, TEXT("Engine starting"));
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
		UE_LOG(LogHelicopterFDM, Log, TEXT("Engine stopping"));
	}
}

// ========================================================================
// Control Input API
// ========================================================================

void UHelicopterFDMComponent::SetControls(const FHelicopterControls& NewControls)
{
	Controls.Elevator = FMath::Clamp(NewControls.Elevator * CyclicSensitivityPitch, -1.0f, 1.0f);
	Controls.Aileron  = FMath::Clamp(NewControls.Aileron  * CyclicSensitivityRoll,  -1.0f, 1.0f);
	Controls.Throttle = FMath::Clamp(NewControls.Throttle * CollectiveSensitivity,    0.0f, 1.0f);
	Controls.Rudder   = FMath::Clamp(NewControls.Rudder   * PedalSensitivity,       -1.0f, 1.0f);
}

void UHelicopterFDMComponent::SetCyclicLongitudinal(float Value)
{
	Controls.Elevator = FMath::Clamp(Value * CyclicSensitivityPitch, -1.0f, 1.0f);
}

void UHelicopterFDMComponent::SetCyclicLateral(float Value)
{
	Controls.Aileron = FMath::Clamp(Value * CyclicSensitivityRoll, -1.0f, 1.0f);
}

void UHelicopterFDMComponent::SetCollective(float Value)
{
	Controls.Throttle = FMath::Clamp(Value * CollectiveSensitivity, 0.0f, 1.0f);
}

void UHelicopterFDMComponent::SetPedals(float Value)
{
	Controls.Rudder = FMath::Clamp(Value * PedalSensitivity, -1.0f, 1.0f);
}

void UHelicopterFDMComponent::FilterControlInputs(float DeltaTime)
{
	double dt = static_cast<double>(DeltaTime);

	double alpha_cyclic    = 1.0 - FMath::Exp(-dt / static_cast<double>(FMath::Max(InputSmoothingCyclic, 0.01f)));
	double alpha_pedals    = 1.0 - FMath::Exp(-dt / static_cast<double>(FMath::Max(InputSmoothingPedals, 0.01f)));
	double alpha_collective = 1.0 - FMath::Exp(-dt / static_cast<double>(FMath::Max(InputSmoothingCollective, 0.01f)));

	SmoothedElevator += (static_cast<double>(Controls.Elevator) - SmoothedElevator) * alpha_cyclic;
	SmoothedAileron  += (static_cast<double>(Controls.Aileron)  - SmoothedAileron)  * alpha_cyclic;
	SmoothedThrottle += (static_cast<double>(Controls.Throttle) - SmoothedThrottle) * alpha_collective;
	SmoothedRudder   += (static_cast<double>(Controls.Rudder)   - SmoothedRudder)   * alpha_pedals;
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
	if (!PhysicsComponent || !bEngineRunning) return 0.0f;
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
	return (EngineMaxRPM > 0.0) ? static_cast<float>(EngineRPM_Internal / EngineMaxRPM * 100.0) : 0.0f;
}

float UHelicopterFDMComponent::GetMainRotorThrust() const
{
	return static_cast<float>(MainRotorThrust);
}

float UHelicopterFDMComponent::GetTailRotorRPMPercent() const
{
	return (MainRotorRPMNominal > 0.0) ? static_cast<float>(MainRotorRPM_Internal / MainRotorRPMNominal * 100.0) : 0.0f;
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

			double EngTorque = EngineMaxPowerHP * 746.0 * GovernorThrottle / EngineOmega;
			double RotorLoadTorque = FMath::Abs(MainRotorTorque / EngineToRotorRatio);
			double NetTorque = EngTorque - RotorLoadTorque;

			EngineRPM_Internal += (NetTorque / EngineMomentOfInertia) * dt * 30.0 / UE_DOUBLE_PI;
		}
	}

	EngineRPM_Internal = FMath::Clamp(EngineRPM_Internal, 0.0, EngineMaxRPM * 1.1);
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
		return;
	}

	double Weight = MassKg * Gravity;
	double RawThrust = SmoothedThrottle * RpmFactor * RpmFactor * Weight * ThrustMultiplier;
	RawThrust *= EvaluateIGEFactor();

	// Smoothing
	if (ThrustResponseTime > 0.001)
	{
		double SmoothAlpha = 1.0 - FMath::Exp(-static_cast<double>(CurrentDeltaTime) / ThrustResponseTime);
		MainRotorThrust += (RawThrust - MainRotorThrust) * SmoothAlpha;
	}
	else
	{
		MainRotorThrust = RawThrust;
	}

	// Torque
	MainRotorTorque = MainRotorThrust * TorqueFraction * MainRotor.Radius;

	// Cyclic disc tilt
	double MaxTilt = CyclicMaxTilt * HDegToRad;
	CyclicLon = SmoothedElevator * MaxTilt;
	CyclicLat = SmoothedAileron * MaxTilt;
}

// ========================================================================
// Force Application
// ========================================================================

void UHelicopterFDMComponent::CommitForcesAndTorques()
{
	if (!PhysicsComponent || !PhysicsComponent->IsSimulatingPhysics()) return;

	FBodyInstance* BodyInst = PhysicsComponent->GetBodyInstance();
	if (!BodyInst) return;

	// Cache physics state once
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

	// Accumulators
	FVector TotalForce = FVector::ZeroVector;
	FVector TotalTorque = FVector::ZeroVector;

	// Velocity in m/s
	double VzMs = static_cast<double>(LinVelocity.Z) / 100.0;
	double VelFwdComp = static_cast<double>(FVector::DotProduct(LinVelocity, BodyFwd)) / 100.0;
	double VelLatComp = static_cast<double>(FVector::DotProduct(LinVelocity, BodyRight)) / 100.0;

	// Angular rates in body frame
	double PitchRate = static_cast<double>(FVector::DotProduct(AngVelocity, BodyRight));
	double RollRate  = static_cast<double>(FVector::DotProduct(AngVelocity, BodyFwd));
	double YawRate   = static_cast<double>(FVector::DotProduct(AngVelocity, BodyUp));

	// Shared: Yaw-only rotation for attitude/envelope (computed once)
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

		TotalForce += DiscNormalWorld * static_cast<float>(Thrust * 100.0);

		double HubZ = MainRotor.Position.Z / 100.0;
		if (FMath::Abs(HubZ) > 0.01)
		{
			double Weight = MassKg * Gravity;
			double MinCyclicThrust = static_cast<double>(MinCyclicAuthorityFraction) * Weight * RpmFactor;
			double CyclicThrust = FMath::Max(Thrust, MinCyclicThrust);

			double TauRoll  =  HubZ * CyclicThrust * FMath::Sin(CyclicLat);
			double TauPitch = -HubZ * CyclicThrust * FMath::Sin(CyclicLon);

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

	// [A2] Vertical speed damping
	{
		double VerticalDamping = 0.0;
		if (VzMs > static_cast<double>(MaxClimbRate))
		{
			VerticalDamping = -(VzMs - static_cast<double>(MaxClimbRate)) * static_cast<double>(VerticalDampingGain) * MassKg;
		}
		else if (VzMs < -static_cast<double>(MaxDescentRate))
		{
			VerticalDamping = (-VzMs - static_cast<double>(MaxDescentRate)) * static_cast<double>(VerticalDampingGain) * MassKg;
		}

		if (FMath::Abs(VerticalDamping) > 0.1)
		{
			TotalForce.Z += static_cast<float>(VerticalDamping * 100.0);
		}
	}

	// [A3] Lateral + backward velocity damping
	{
		double LatForce = -VelLatComp * static_cast<double>(LateralDampingGain) * MassKg;

		double BackForce = 0.0;
		if (VelFwdComp < -1.0)
		{
			BackForce = -VelFwdComp * static_cast<double>(BackwardDampingGain) * MassKg;
		}

		TotalForce +=
			BodyRight * static_cast<float>(LatForce * 100.0) +
			BodyFwd   * static_cast<float>(BackForce * 100.0);
	}

	// [Hull] Parasitic drag
	{
		double GVx = LinVelocity.X / 100.0;
		double GVy = LinVelocity.Y / 100.0;
		double GVz = LinVelocity.Z / 100.0;
		double AirspeedMag = FMath::Sqrt(GVx * GVx + GVy * GVy + GVz * GVz);

		if (AirspeedMag > 0.01)
		{
			double InvMag = 1.0 / AirspeedMag;
			double Q = 0.5 * AirDensity * AirspeedMag * AirspeedMag;
			double DTotal = Q * (HullReferenceArea * HullDragCoefficient + LandingGearReferenceArea * LandingGearDragCoefficient);

			TotalForce += FVector(
				static_cast<float>(-GVx * InvMag * DTotal * 100.0),
				static_cast<float>(-GVy * InvMag * DTotal * 100.0),
				static_cast<float>(-GVz * InvMag * DTotal * 100.0)
			);
		}
	}

	// [B+C] Pedal yaw (reactive torque auto-trimmed by tail rotor)
	{
		double TailArm = FMath::Abs(TailRotor.Position.X) / 100.0;
		double PedalMoment = SmoothedRudder * PedalYawAuthority * MassKg * Gravity * TailArm * RpmFactor;
		TotalTorque += BodyUp * static_cast<float>(PedalMoment * 10000.0);
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

		// Attitude hold — active when cyclic input below deadzone
		double CyclicMag = FMath::Max(FMath::Abs(SmoothedElevator), FMath::Abs(SmoothedAileron));
		if (CyclicMag < static_cast<double>(SAS_AttitudeHoldDeadzone) && RpmFactor > 0.1)
		{
			double K = static_cast<double>(SAS_AttitudeHoldStrength);
			double Progressive = static_cast<double>(SAS_AttitudeHoldProgressive);

			double KPitch = K * (1.0 + Progressive * FMath::Abs(CurrentPitch));
			double KRoll  = K * (1.0 + Progressive * FMath::Abs(CurrentRoll));

			AttTauPitch = -KPitch * CurrentPitch * Iyy * RpmFactor;
			AttTauRoll  = -KRoll  * CurrentRoll  * Ixx * RpmFactor;
		}

		// Envelope protection: onset at 80%, cubic pushback T³
		double MaxPitchRad = static_cast<double>(EnvelopeMaxPitch) * HDegToRad;
		double MaxRollRad  = static_cast<double>(EnvelopeMaxRoll)  * HDegToRad;
		double EnvK = static_cast<double>(EnvelopeStrength);
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
