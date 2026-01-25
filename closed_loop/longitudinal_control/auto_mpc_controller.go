package control

import (
	"math"
)

// ============================================================================
// AUTO-MPC DIAGNOSTICS (specific to this controller)
// ============================================================================

// AutoMPCDiagnostics contains internal state for monitoring
type AutoMPCDiagnostics struct {
	EstimatedMass      float64
	EstimatedDrag      float64
	EstimatedRolling   float64
	EstimatedMaxTorque float64
	EstimatedMaxBrake  float64
	MassConfidence     float64
	DragConfidence     float64
	LimitConfidence    float64
	AdaptiveKp         float64
	AdaptiveKi         float64
	AdaptiveKd         float64
	IterationCount     int
	WheelRadiusM       float64
	GearRatio          float64
}

// ============================================================================
// IMPROVED AUTO-MPC CONTROLLER - APPLYING MPC TUNING LESSONS
// ============================================================================
// Changes from original:
// 1. FIXED mass adaptation sign (was going wrong direction)
// 2. Higher sensitivity for faster convergence
// 3. Better initial estimates (start at 200 tons, not 220)
// 4. Improved force conversion with realistic geometry
// 5. Better integral anti-windup
// 6. Adaptive deadband for integral term
// ============================================================================

// AutoMPCController with fixes from MPC tuning experience
type AutoMPCController struct {
	cfg AutoMPCConfig

	// Estimated vehicle parameters (learned online)
	estimatedMass      float64 // kg
	estimatedDrag      float64 // N/(m/s)²
	estimatedRolling   float64 // N
	estimatedMaxTorque float64 // Nm
	estimatedMaxBrake  float64 // N

	// Vehicle geometry
	wheelRadiusM float64
	gearRatio    float64

	// Confidence tracking
	massConfidence  float64 // 0-1
	dragConfidence  float64 // 0-1
	limitConfidence float64 // 0-1

	// Adaptive control gains
	adaptiveKp float64
	adaptiveKi float64
	adaptiveKd float64

	// State tracking
	velocityHistory []float64
	accelHistory    []float64
	controlHistory  []ControlOutput
	errorIntegral   float64
	prevError       float64

	// Overshoot protection
	overshootDuration float64

	iterationCount int
}

// NewAutoMPCController creates improved Auto-MPC
func NewAutoMPCController(cfg AutoMPCConfig) *AutoMPCController {
	if cfg.LearningRate == 0 {
		cfg.LearningRate = 0.05 // Increased from 0.01 for faster learning
	}

	return &AutoMPCController{
		cfg: cfg,

		// LESSON 1: Start at 200 tons, not 220 (room to adapt UP)
		estimatedMass:      200000.0, // 200 tons (will adapt to 220)
		estimatedDrag:      3.5,      // Realistic initial estimate
		estimatedRolling:   8000.0,   // ~4% grade resistance
		estimatedMaxTorque: 145000.0, // Known vehicle limit
		estimatedMaxBrake:  180000.0, // 180 kN brake force

		// Vehicle geometry
		wheelRadiusM: 1.95,
		gearRatio:    28.0,

		// LESSON 2: Start with medium confidence (not low)
		massConfidence:  0.3, // Was 0.05, now 0.3
		dragConfidence:  0.3, // Was 0.05, now 0.3
		limitConfidence: 0.3, // Was 0.05, now 0.3

		// LESSON 3: Start with stronger initial gains
		adaptiveKp: 8000.0, // Much higher starting point
		adaptiveKi: 400.0,  // Much higher starting point
		adaptiveKd: 1500.0, // Much higher starting point

		velocityHistory: make([]float64, 0, 200),
		accelHistory:    make([]float64, 0, 200),
		controlHistory:  make([]ControlOutput, 0, 200),
	}
}

// Update computes optimal control
func (amc *AutoMPCController) Update(currentVelocity float64, dt float64) ControlOutput {
	amc.iterationCount++

	// Step 1: Learn vehicle parameters (FIXED adaptation)
	amc.adaptVehicleModel(currentVelocity, dt)

	// Step 2: Auto-tune gains
	amc.autoTuneGains(currentVelocity, dt)

	// Step 3: Compute control with improved anti-windup
	output := amc.computeAdaptiveControl(currentVelocity, dt)

	// Step 4: Detect saturation
	amc.detectSaturation(output)

	// Step 5: Store history
	amc.updateHistory(currentVelocity, dt, output)

	return output
}

// adaptVehicleModel - FIXED VERSION with correct sign
func (amc *AutoMPCController) adaptVehicleModel(velocity float64, dt float64) {
	if len(amc.velocityHistory) < 3 || len(amc.controlHistory) < 2 {
		return
	}

	vPrev := amc.velocityHistory[len(amc.velocityHistory)-2]
	prevControl := amc.controlHistory[len(amc.controlHistory)-2]

	measuredAccel := (velocity - vPrev) / dt

	// Only adapt when significant control applied
	if math.Abs(prevControl.TorqueNm) < 1000 && math.Abs(prevControl.BrakePct) < 5 {
		return
	}

	// Estimate forces
	driveForce := amc.torqueToForce(prevControl.TorqueNm)
	brakeForce := -amc.brakeToForce(prevControl.BrakePct)
	netControlForce := driveForce + brakeForce

	// Resistive forces
	dragForce := amc.estimatedDrag * vPrev * math.Abs(vPrev)
	if vPrev < 0 {
		dragForce = -dragForce
	}
	rollingForce := amc.estimatedRolling
	if vPrev < 0 {
		rollingForce = -rollingForce
	}

	// Predicted vs measured acceleration
	predictedAccel := (netControlForce - dragForce - rollingForce) / amc.estimatedMass
	accelError := measuredAccel - predictedAccel

	// Adaptive learning rate
	alpha := amc.cfg.LearningRate * (1.0 - amc.massConfidence*0.5)

	// CRITICAL FIX: Mass adaptation with CORRECT SIGN
	// If accelError < 0 → vehicle accelerating slower → mass too LOW → INCREASE mass
	// If accelError > 0 → vehicle accelerating faster → mass too HIGH → DECREASE mass
	if math.Abs(netControlForce) > 2000 {
		// FIXED: Negative sign to invert relationship
		massUpdate := -alpha * accelError * math.Abs(netControlForce) * 0.02 // Increased sensitivity
		amc.estimatedMass += massUpdate
		amc.estimatedMass = ClampFloat(amc.estimatedMass, 50000, 400000)
		amc.massConfidence = math.Min(0.99, amc.massConfidence+0.003) // Faster confidence gain
	}

	// Drag coefficient (higher speeds)
	if math.Abs(vPrev) > 2.0 {
		dragUpdate := -alpha * accelError * vPrev * math.Abs(vPrev) * 0.08 // Increased sensitivity
		amc.estimatedDrag += dragUpdate
		amc.estimatedDrag = ClampFloat(amc.estimatedDrag, 0.5, 25.0)
		amc.dragConfidence = math.Min(0.99, amc.dragConfidence+0.002)
	}

	// Rolling resistance (lower speeds)
	if math.Abs(vPrev) > 0.5 && math.Abs(vPrev) < 3.0 {
		sign := 1.0
		if vPrev < 0 {
			sign = -1.0
		}
		rollingUpdate := -alpha * accelError * sign * 300.0 // Increased sensitivity
		amc.estimatedRolling += rollingUpdate
		amc.estimatedRolling = ClampFloat(amc.estimatedRolling, 100, 20000)
	}
}

// autoTuneGains - Improved version with better mass scaling
func (amc *AutoMPCController) autoTuneGains(velocity float64, dt float64) {
	if len(amc.velocityHistory) < 20 {
		return
	}

	// Check for oscillations
	recentVel := amc.velocityHistory[len(amc.velocityHistory)-20:]
	variance := 0.0
	mean := 0.0
	for _, v := range recentVel {
		mean += v
	}
	mean /= float64(len(recentVel))

	for _, v := range recentVel {
		variance += (v - mean) * (v - mean)
	}
	variance /= float64(len(recentVel))

	// Adjust for oscillations
	if variance > 1.0 && amc.massConfidence > 0.3 {
		amc.adaptiveKp *= 0.95 // More aggressive reduction
		amc.adaptiveKd *= 1.05
	} else if variance < 0.05 && amc.massConfidence > 0.5 {
		amc.adaptiveKp *= 1.02
		amc.adaptiveKi *= 1.01
	}

	// LESSON: Better gain scaling based on mass
	// For 220-ton vehicle, we found Kp=15000, Ki=800, Kd=3000 works well
	// Scale proportionally with estimated mass
	massRatio := amc.estimatedMass / 220000.0
	targetKp := 15000.0 * massRatio
	targetKi := 800.0 * massRatio
	targetKd := 3000.0 * massRatio

	// Apply aggressive tuning if enabled
	if amc.cfg.AggressiveTuning {
		targetKp *= 1.3
		targetKi *= 1.2
		targetKd *= 1.3
	}

	// Converge to target gains
	amc.adaptiveKp += (targetKp - amc.adaptiveKp) * 0.02 // Faster convergence
	amc.adaptiveKi += (targetKi - amc.adaptiveKi) * 0.02
	amc.adaptiveKd += (targetKd - amc.adaptiveKd) * 0.02

	// Realistic bounds for 200+ ton vehicles
	amc.adaptiveKp = ClampFloat(amc.adaptiveKp, 1000, 50000)
	amc.adaptiveKi = ClampFloat(amc.adaptiveKi, 100, 10000)
	amc.adaptiveKd = ClampFloat(amc.adaptiveKd, 500, 20000)
}

// computeAdaptiveControl - Improved with better anti-windup
func (amc *AutoMPCController) computeAdaptiveControl(velocity float64, dt float64) ControlOutput {
	error := amc.cfg.TargetVelocityMPS - velocity

	// LESSON: Overshoot protection from PID tuning
	if error < 0 {
		amc.overshootDuration += dt
		if amc.overshootDuration > 1.5 {
			// Aggressive integral reset
			amc.errorIntegral *= 0.3
		}
	} else {
		amc.overshootDuration = 0.0
	}

	// P term
	pTerm := amc.adaptiveKp * error

	// I term with adaptive deadband
	deadband := 0.1 // Don't integrate tiny errors
	if math.Abs(error) > deadband {
		amc.errorIntegral += error * dt
	}

	// LESSON: Reset integral when crossing setpoint
	if (amc.prevError > 0 && error < 0) || (amc.prevError < 0 && error > 0) {
		amc.errorIntegral *= 0.15 // Aggressive reset
	}

	// Anti-windup
	maxIntegral := amc.estimatedMaxTorque / (amc.adaptiveKi + 1.0)
	amc.errorIntegral = ClampFloat(amc.errorIntegral, -maxIntegral, maxIntegral)
	iTerm := amc.adaptiveKi * amc.errorIntegral

	// D term
	dTerm := 0.0
	if dt > 0 && amc.iterationCount > 1 {
		dTerm = amc.adaptiveKd * (error - amc.prevError) / dt
	}
	amc.prevError = error

	// Total control torque
	controlTorque := pTerm + iTerm + dTerm

	// Feedforward compensation for resistive forces
	controlForce := amc.torqueToForce(controlTorque)
	if controlForce > 0 {
		dragComp := amc.estimatedDrag * velocity * math.Abs(velocity)
		rollingComp := amc.estimatedRolling
		if velocity < 0 {
			dragComp = -dragComp
			rollingComp = -rollingComp
		}
		controlForce += dragComp + rollingComp
	}

	return amc.forceToActuators(controlForce)
}

// forceToActuators - Same as before
func (amc *AutoMPCController) forceToActuators(force float64) ControlOutput {
	var output ControlOutput
	output.Confidence = (amc.massConfidence + amc.dragConfidence + amc.limitConfidence) / 3.0

	if force > 0 {
		torque := amc.forceToTorque(force)
		output.TorqueNm = ClampFloat(torque, 0, amc.estimatedMaxTorque)
		output.BrakePct = 0
		output.IsAccel = true
		output.IsBrake = false
	} else {
		brakeForce := -force
		brakePct := amc.forceToBrake(brakeForce)
		output.TorqueNm = 0
		output.BrakePct = ClampFloat(brakePct, 0, 100)
		output.IsAccel = false
		output.IsBrake = true
	}

	return output
}

// detectSaturation - Same as before
func (amc *AutoMPCController) detectSaturation(output ControlOutput) {
	if output.TorqueNm >= amc.estimatedMaxTorque*0.99 {
		amc.estimatedMaxTorque *= 1.05
		amc.estimatedMaxTorque = math.Min(amc.estimatedMaxTorque, 200000)
	}

	if output.BrakePct >= 98.0 {
		amc.estimatedMaxBrake *= 1.05
		amc.estimatedMaxBrake = math.Min(amc.estimatedMaxBrake, 500000)
	}

	amc.limitConfidence = math.Min(0.99, amc.limitConfidence+0.001)
}

// Conversion helpers
func (amc *AutoMPCController) torqueToForce(torque float64) float64 {
	return torque * amc.gearRatio / amc.wheelRadiusM
}

func (amc *AutoMPCController) forceToTorque(force float64) float64 {
	return force * amc.wheelRadiusM / amc.gearRatio
}

func (amc *AutoMPCController) brakeToForce(brakePct float64) float64 {
	return (brakePct / 100.0) * amc.estimatedMaxBrake
}

func (amc *AutoMPCController) forceToBrake(force float64) float64 {
	return (force / amc.estimatedMaxBrake) * 100.0
}

// updateHistory - Same as before
func (amc *AutoMPCController) updateHistory(velocity float64, dt float64, output ControlOutput) {
	amc.velocityHistory = append(amc.velocityHistory, velocity)
	if len(amc.velocityHistory) > 200 {
		amc.velocityHistory = amc.velocityHistory[1:]
	}

	if len(amc.velocityHistory) >= 2 {
		vPrev := amc.velocityHistory[len(amc.velocityHistory)-2]
		accel := (velocity - vPrev) / dt
		amc.accelHistory = append(amc.accelHistory, accel)
		if len(amc.accelHistory) > 200 {
			amc.accelHistory = amc.accelHistory[1:]
		}
	}

	amc.controlHistory = append(amc.controlHistory, output)
	if len(amc.controlHistory) > 200 {
		amc.controlHistory = amc.controlHistory[1:]
	}
}

// GetDiagnostics - Same interface
func (amc *AutoMPCController) GetDiagnostics() AutoMPCDiagnostics {
	return AutoMPCDiagnostics{
		EstimatedMass:      amc.estimatedMass,
		EstimatedDrag:      amc.estimatedDrag,
		EstimatedRolling:   amc.estimatedRolling,
		EstimatedMaxTorque: amc.estimatedMaxTorque,
		EstimatedMaxBrake:  amc.estimatedMaxBrake,
		MassConfidence:     amc.massConfidence,
		DragConfidence:     amc.dragConfidence,
		LimitConfidence:    amc.limitConfidence,
		AdaptiveKp:         amc.adaptiveKp,
		AdaptiveKi:         amc.adaptiveKi,
		AdaptiveKd:         amc.adaptiveKd,
		IterationCount:     amc.iterationCount,
		WheelRadiusM:       amc.wheelRadiusM,
		GearRatio:          amc.gearRatio,
	}
}

// GetTargetVelocity - Same interface
func (amc *AutoMPCController) GetTargetVelocity() float64 {
	return amc.cfg.TargetVelocityMPS
}

// Reset - Updated with new initial values
func (amc *AutoMPCController) Reset() {
	amc.velocityHistory = amc.velocityHistory[:0]
	amc.accelHistory = amc.accelHistory[:0]
	amc.controlHistory = amc.controlHistory[:0]
	amc.errorIntegral = 0
	amc.prevError = 0
	amc.overshootDuration = 0
	amc.massConfidence = 0.3
	amc.dragConfidence = 0.3
	amc.limitConfidence = 0.3

	amc.estimatedMass = 200000.0
	amc.estimatedDrag = 3.5
	amc.estimatedRolling = 8000.0
	amc.estimatedMaxTorque = 145000.0
	amc.estimatedMaxBrake = 180000.0

	amc.adaptiveKp = 8000.0
	amc.adaptiveKi = 400.0
	amc.adaptiveKd = 1500.0
}
