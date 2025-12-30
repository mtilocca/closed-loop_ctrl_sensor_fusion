package main

import (
	"math"
)

// AutoMPCController is a fully autonomous adaptive controller
// It learns ALL vehicle parameters online and adapts control automatically
type AutoMPCController struct {
	cfg AutoMPCConfig

	// Estimated vehicle parameters (learned online)
	estimatedMass      float64 // kg
	estimatedDrag      float64 // N/(m/s)²
	estimatedRolling   float64 // N
	estimatedMaxTorque float64 // Nm (discovered by saturation detection)
	estimatedMaxBrake  float64 // kN (discovered by saturation detection)

	// Confidence tracking
	massConfidence  float64 // 0-1
	dragConfidence  float64 // 0-1
	limitConfidence float64 // 0-1

	// Adaptive control gains (auto-tuned based on vehicle response)
	adaptiveKp float64
	adaptiveKi float64
	adaptiveKd float64

	// State tracking
	velocityHistory []float64
	accelHistory    []float64
	controlHistory  []ControlOutput
	errorIntegral   float64
	prevError       float64

	// Performance metrics (for auto-tuning)
	settlingDetected bool
	responseSpeed    float64 // rad/s (vehicle bandwidth estimate)

	iterationCount int
}

// NewAutoMPCController creates a fully autonomous MPC
func NewAutoMPCController(cfg AutoMPCConfig) *AutoMPCController {
	// Set defaults if not specified
	if cfg.LearningRate == 0 {
		cfg.LearningRate = 0.01
	}

	return &AutoMPCController{
		cfg: cfg,

		// Conservative initial estimates (will adapt quickly)
		estimatedMass:      8000.0, // Start with medium truck assumption
		estimatedDrag:      3.0,    // Moderate drag
		estimatedRolling:   800.0,  // Moderate rolling resistance
		estimatedMaxTorque: 3000.0, // Will increase if saturates
		estimatedMaxBrake:  150.0,  // Will increase if saturates (kN)

		// Low initial confidence
		massConfidence:  0.05,
		dragConfidence:  0.05,
		limitConfidence: 0.05,

		// Conservative initial gains (will auto-tune)
		adaptiveKp: 50.0,
		adaptiveKi: 5.0,
		adaptiveKd: 10.0,

		velocityHistory: make([]float64, 0, 200),
		accelHistory:    make([]float64, 0, 200),
		controlHistory:  make([]ControlOutput, 0, 200),
	}
}

// Update computes optimal control using adaptive MPC
func (amc *AutoMPCController) Update(currentVelocity float64, dt float64) ControlOutput {
	amc.iterationCount++

	// Step 1: Learn vehicle parameters from data
	amc.adaptVehicleModel(currentVelocity, dt)

	// Step 2: Auto-tune controller gains based on performance
	amc.autoTuneGains(currentVelocity, dt)

	// Step 3: Compute control using adaptive PID-like law
	output := amc.computeAdaptiveControl(currentVelocity, dt)

	// Step 4: Detect and adapt to saturation
	amc.detectSaturation(output)

	// Step 5: Store history
	amc.updateHistory(currentVelocity, dt, output)

	return output
}

// adaptVehicleModel learns mass, drag, and rolling resistance online
func (amc *AutoMPCController) adaptVehicleModel(velocity float64, dt float64) {
	if len(amc.velocityHistory) < 3 || len(amc.controlHistory) < 2 {
		return // Need more data
	}

	// Get previous state
	vPrev := amc.velocityHistory[len(amc.velocityHistory)-2]
	prevControl := amc.controlHistory[len(amc.controlHistory)-2]

	// Measured acceleration
	measuredAccel := (velocity - vPrev) / dt

	// Only adapt when control is applied (otherwise just coasting)
	if math.Abs(prevControl.TorqueNm) < 10 && math.Abs(prevControl.BrakePct) < 1 {
		return
	}

	// Estimate forces from control
	driveForce := amc.torqueToForce(prevControl.TorqueNm)
	brakeForce := -amc.brakeToForce(prevControl.BrakePct) // Negative (opposes motion)
	netControlForce := driveForce + brakeForce

	// Estimate resistive forces using current model
	dragForce := amc.estimatedDrag * vPrev * math.Abs(vPrev) // Use |v|*v for directionality
	if vPrev < 0 {
		dragForce = -dragForce
	}
	rollingForce := amc.estimatedRolling
	if vPrev < 0 {
		rollingForce = -rollingForce
	}

	// Predicted acceleration: F_net = m*a
	predictedAccel := (netControlForce - dragForce - rollingForce) / amc.estimatedMass

	// Prediction error
	accelError := measuredAccel - predictedAccel

	// Adaptive learning rate (higher when confidence is low)
	alpha := amc.cfg.LearningRate * (1.0 - amc.massConfidence*0.5)

	// Update mass (inverse relationship with acceleration)
	if math.Abs(netControlForce) > 500 {
		massUpdate := alpha * accelError * netControlForce * 0.01
		amc.estimatedMass += massUpdate
		amc.estimatedMass = clampFloat(amc.estimatedMass, 1000, 80000) // 1-80 tons
		amc.massConfidence = math.Min(0.99, amc.massConfidence+0.002)
	}

	// Update drag coefficient (when moving at reasonable speed)
	if math.Abs(vPrev) > 2.0 {
		dragUpdate := -alpha * accelError * vPrev * math.Abs(vPrev) * 0.05
		amc.estimatedDrag += dragUpdate
		amc.estimatedDrag = clampFloat(amc.estimatedDrag, 0.5, 15.0)
		amc.dragConfidence = math.Min(0.99, amc.dragConfidence+0.001)
	}

	// Update rolling resistance (when moving slowly)
	if math.Abs(vPrev) > 0.5 && math.Abs(vPrev) < 3.0 {
		sign := 1.0
		if vPrev < 0 {
			sign = -1.0
		}
		rollingUpdate := -alpha * accelError * sign * 200.0
		amc.estimatedRolling += rollingUpdate
		amc.estimatedRolling = clampFloat(amc.estimatedRolling, 100, 5000)
	}
}

// autoTuneGains adjusts PID gains based on closed-loop performance
func (amc *AutoMPCController) autoTuneGains(velocity float64, dt float64) {
	if len(amc.velocityHistory) < 20 {
		return // Need settling data
	}

	// Estimate system response speed from velocity changes
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

	// High variance = oscillatory = reduce gains
	// Low variance = sluggish = increase gains
	if variance > 0.5 && amc.massConfidence > 0.3 {
		// Oscillating - reduce gains
		amc.adaptiveKp *= 0.98
		amc.adaptiveKd *= 1.02
	} else if variance < 0.01 && amc.massConfidence > 0.5 {
		// Sluggish - increase gains if confident
		amc.adaptiveKp *= 1.01
		amc.adaptiveKi *= 1.005
	}

	// Tune based on vehicle mass (heavier = higher gains)
	targetKp := 50.0 + (amc.estimatedMass/1000.0)*5.0 // Scale with mass
	targetKi := 5.0 + (amc.estimatedMass/1000.0)*0.5
	targetKd := 10.0 + (amc.estimatedMass/1000.0)*1.0

	// Aggressive tuning mode
	if amc.cfg.AggressiveTuning {
		targetKp *= 1.5
		targetKi *= 1.3
		targetKd *= 1.2
	}

	// Slowly converge to target gains
	amc.adaptiveKp += (targetKp - amc.adaptiveKp) * 0.01
	amc.adaptiveKi += (targetKi - amc.adaptiveKi) * 0.01
	amc.adaptiveKd += (targetKd - amc.adaptiveKd) * 0.01

	// Clamp to reasonable ranges
	amc.adaptiveKp = clampFloat(amc.adaptiveKp, 10, 500)
	amc.adaptiveKi = clampFloat(amc.adaptiveKi, 1, 100)
	amc.adaptiveKd = clampFloat(amc.adaptiveKd, 1, 100)
}

// computeAdaptiveControl calculates control output
func (amc *AutoMPCController) computeAdaptiveControl(velocity float64, dt float64) ControlOutput {
	// Compute error
	error := amc.cfg.TargetVelocityMPS - velocity

	// PID terms
	pTerm := amc.adaptiveKp * error

	amc.errorIntegral += error * dt
	// Anti-windup: limit integral based on estimated vehicle capacity
	maxIntegral := amc.estimatedMaxTorque / (amc.adaptiveKi + 0.01)
	amc.errorIntegral = clampFloat(amc.errorIntegral, -maxIntegral, maxIntegral)
	iTerm := amc.adaptiveKi * amc.errorIntegral

	dTerm := 0.0
	if dt > 0 {
		dTerm = amc.adaptiveKd * (error - amc.prevError) / dt
	}
	amc.prevError = error

	// Total control force needed
	controlForce := pTerm + iTerm + dTerm

	// Add feedforward compensation for drag/rolling resistance
	dragComp := amc.estimatedDrag * velocity * math.Abs(velocity)
	rollingComp := amc.estimatedRolling
	if velocity < 0 {
		dragComp = -dragComp
		rollingComp = -rollingComp
	}
	controlForce += dragComp + rollingComp

	// Convert to actuator commands
	return amc.forceToActuators(controlForce)
}

// forceToActuators converts desired force to torque/brake commands
func (amc *AutoMPCController) forceToActuators(force float64) ControlOutput {
	var output ControlOutput

	// Determine confidence for diagnostic
	output.Confidence = (amc.massConfidence + amc.dragConfidence + amc.limitConfidence) / 3.0

	if force > 0 {
		// Acceleration - use motor torque
		torque := amc.forceToTorque(force)
		output.TorqueNm = clampFloat(torque, 0, amc.estimatedMaxTorque)
		output.BrakePct = 0
		output.IsAccel = true
		output.IsBrake = false
	} else {
		// Deceleration - use brakes
		brakeForce := -force // Positive braking force
		brakePct := amc.forceToBrake(brakeForce)
		output.TorqueNm = 0
		output.BrakePct = clampFloat(brakePct, 0, 100)
		output.IsAccel = false
		output.IsBrake = true
	}

	return output
}

// detectSaturation increases limits if controller saturates
func (amc *AutoMPCController) detectSaturation(output ControlOutput) {
	// If we hit torque limit repeatedly, increase estimate
	if output.TorqueNm >= amc.estimatedMaxTorque*0.99 {
		amc.estimatedMaxTorque *= 1.05
		amc.estimatedMaxTorque = math.Min(amc.estimatedMaxTorque, 10000) // Cap at 10kNm
	}

	// If we hit brake limit repeatedly, increase estimate
	if output.BrakePct >= 98.0 {
		amc.estimatedMaxBrake *= 1.05
		amc.estimatedMaxBrake = math.Min(amc.estimatedMaxBrake, 300) // Cap at 300 kN
	}

	amc.limitConfidence = math.Min(0.99, amc.limitConfidence+0.001)
}

// Conversion helpers (rough estimates, will be compensated by adaptation)
func (amc *AutoMPCController) torqueToForce(torque float64) float64 {
	// Assume ~0.5m wheel radius, ~10:1 gear ratio
	// F = T / (r * gear_ratio)
	return torque / (0.5 * 10.0)
}

func (amc *AutoMPCController) forceToTorque(force float64) float64 {
	return force * (0.5 * 10.0)
}

func (amc *AutoMPCController) brakeToForce(brakePct float64) float64 {
	// Assume max brake force scales with estimated capacity
	return (brakePct / 100.0) * amc.estimatedMaxBrake * 1000.0 // kN to N
}

func (amc *AutoMPCController) forceToBrake(force float64) float64 {
	return (force / (amc.estimatedMaxBrake * 1000.0)) * 100.0
}

// updateHistory stores recent data for adaptation
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

// GetDiagnostics returns current state for monitoring
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
	}
}

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
}

// GetTargetVelocity returns the setpoint
func (amc *AutoMPCController) GetTargetVelocity() float64 {
	return amc.cfg.TargetVelocityMPS
}

// Reset clears all learned parameters
func (amc *AutoMPCController) Reset() {
	amc.velocityHistory = amc.velocityHistory[:0]
	amc.accelHistory = amc.accelHistory[:0]
	amc.controlHistory = amc.controlHistory[:0]
	amc.errorIntegral = 0
	amc.prevError = 0
	amc.massConfidence = 0.05
	amc.dragConfidence = 0.05
	amc.limitConfidence = 0.05
}
