package main

import (
	"math"
)

// VehicleModel represents estimated vehicle dynamics
type VehicleModel struct {
	Mass          float64 // kg (estimated online)
	DragCoeff     float64 // N/(m/s)² (estimated online)
	RollingResist float64 // N (estimated online)

	// Confidence in estimates (0-1)
	Confidence float64
}

// MPCController implements Model Predictive Control for velocity tracking
type MPCController struct {
	cfg   MPCConfig
	model VehicleModel

	// State history for adaptation
	velocityHistory []float64
	accelHistory    []float64
	torqueHistory   []float64

	// Previous control for rate limiting
	prevTorque float64
	prevBrake  float64

	// Statistics
	iterationCount int
}

// NewMPCController creates a new MPC controller
func NewMPCController(cfg MPCConfig) *MPCController {
	// Initialize with conservative vehicle model estimates
	model := VehicleModel{
		Mass:          5000.0, // Start with medium truck assumption
		DragCoeff:     2.5,    // Conservative drag
		RollingResist: 500.0,  // Conservative rolling resistance
		Confidence:    0.1,    // Low initial confidence
	}

	return &MPCController{
		cfg:             cfg,
		model:           model,
		velocityHistory: make([]float64, 0, 100),
		accelHistory:    make([]float64, 0, 100),
		torqueHistory:   make([]float64, 0, 100),
	}
}

// Update computes optimal control using MPC
func (mpc *MPCController) Update(currentVelocity float64, dt float64) ControlOutput {
	mpc.iterationCount++

	// Update model with new data (online adaptation)
	if mpc.cfg.EnableAdaptation && len(mpc.velocityHistory) > 0 {
		mpc.adaptModel(currentVelocity, dt)
	}

	// Store history
	mpc.velocityHistory = append(mpc.velocityHistory, currentVelocity)
	if len(mpc.velocityHistory) > 100 {
		mpc.velocityHistory = mpc.velocityHistory[1:]
	}

	// Compute tracking error
	error := mpc.cfg.TargetVelocityMPS - currentVelocity

	// Predict future trajectory and optimize control
	output := mpc.solveOptimization(currentVelocity, error, dt)

	// Store control history for adaptation
	mpc.torqueHistory = append(mpc.torqueHistory, output.TorqueNm)
	if len(mpc.torqueHistory) > 100 {
		mpc.torqueHistory = mpc.torqueHistory[1:]
	}

	mpc.prevTorque = output.TorqueNm
	mpc.prevBrake = output.BrakePct

	return output
}

// solveOptimization performs simplified MPC optimization
func (mpc *MPCController) solveOptimization(v0 float64, error float64, dt float64) ControlOutput {
	// Simplified single-step MPC (can be extended to full horizon)
	// For real-time embedded systems, full QP solver is too heavy
	// Use gradient-based approach instead

	// Predict required acceleration to reach target
	horizonTime := float64(mpc.cfg.PredictionHorizon) * dt
	requiredAccel := error / horizonTime // Simple P-like prediction

	// Add damping based on current velocity error rate
	if len(mpc.velocityHistory) >= 2 {
		velocityRate := (v0 - mpc.velocityHistory[len(mpc.velocityHistory)-2]) / dt
		requiredAccel -= 0.5 * velocityRate // Damping term
	}

	// Clamp to physical limits
	requiredAccel = clampFloat(requiredAccel, -mpc.cfg.MaxDecel, mpc.cfg.MaxAccel)

	// Convert acceleration to control commands
	return mpc.accelToControl(requiredAccel, v0)
}

// accelToControl converts desired acceleration to torque/brake commands
func (mpc *MPCController) accelToControl(accel float64, velocity float64) ControlOutput {
	// Estimate resistive forces
	dragForce := mpc.model.DragCoeff * velocity * velocity
	rollingForce := mpc.model.RollingResist
	resistiveForce := dragForce + rollingForce

	// Required force: F = m*a + F_resist
	requiredForce := mpc.model.Mass*accel + resistiveForce

	var output ControlOutput
	output.Confidence = mpc.model.Confidence

	if requiredForce > 0 {
		// Acceleration mode - use motor torque
		// Assume wheel radius ~0.5m, gear ratio varies
		// Torque = Force * wheel_radius / gear_ratio
		// Simplified: torque proportional to force
		torque := requiredForce * 0.3 // Rough conversion factor

		output.TorqueNm = clampFloat(torque, 0, mpc.cfg.MaxTorque)
		output.BrakePct = 0
		output.IsAccel = true
		output.IsBrake = false
	} else {
		// Deceleration mode - use brakes
		brakingForce := -requiredForce // Positive braking force

		// Convert force to brake percentage
		// Assume max brake force corresponds to 100%
		maxBrakeForceN := mpc.cfg.MaxBrakeForce * 1000 // kN to N
		brakePct := (brakingForce / maxBrakeForceN) * 100.0

		output.TorqueNm = 0
		output.BrakePct = clampFloat(brakePct, 0, 100)
		output.IsAccel = false
		output.IsBrake = true
	}

	return output
}

// adaptModel updates vehicle model parameters online using recursive least squares
func (mpc *MPCController) adaptModel(currentVelocity float64, dt float64) {
	if len(mpc.velocityHistory) < 3 || len(mpc.torqueHistory) < 2 {
		return // Need more data
	}

	// Get previous states
	v_prev := mpc.velocityHistory[len(mpc.velocityHistory)-2]
	torque_prev := mpc.torqueHistory[len(mpc.torqueHistory)-2]

	// Measured acceleration
	measuredAccel := (currentVelocity - v_prev) / dt

	// Predicted acceleration using current model
	dragForce := mpc.model.DragCoeff * v_prev * v_prev
	rollingForce := mpc.model.RollingResist
	driveForce := torque_prev / 0.3 // Inverse of conversion factor

	predictedAccel := (driveForce - dragForce - rollingForce) / mpc.model.Mass

	// Prediction error
	accelError := measuredAccel - predictedAccel

	// Update parameters using gradient descent
	alpha := mpc.cfg.AdaptationRate

	// Update mass (inverse relationship with accel)
	if math.Abs(driveForce) > 100 { // Only adapt when force is significant
		mpc.model.Mass += alpha * accelError * driveForce * 0.001
		mpc.model.Mass = clampFloat(mpc.model.Mass, 1000, 50000) // 1-50 tons
	}

	// Update drag coefficient (proportional to v²)
	if v_prev > 1.0 { // Only adapt at reasonable speeds
		mpc.model.DragCoeff -= alpha * accelError * v_prev * v_prev * 0.01
		mpc.model.DragCoeff = clampFloat(mpc.model.DragCoeff, 0.5, 10.0)
	}

	// Increase confidence gradually
	mpc.model.Confidence += 0.001
	mpc.model.Confidence = clampFloat(mpc.model.Confidence, 0, 1)
}

// Reset clears controller state
func (mpc *MPCController) Reset() {
	mpc.velocityHistory = mpc.velocityHistory[:0]
	mpc.accelHistory = mpc.accelHistory[:0]
	mpc.torqueHistory = mpc.torqueHistory[:0]
	mpc.prevTorque = 0
	mpc.prevBrake = 0
	mpc.iterationCount = 0
	mpc.model.Confidence = 0.1
}

// GetDiagnostics returns current state for logging
func (mpc *MPCController) GetDiagnostics() MPCDiagnostics {
	return MPCDiagnostics{
		EstimatedMass:    mpc.model.Mass,
		EstimatedDrag:    mpc.model.DragCoeff,
		EstimatedRolling: mpc.model.RollingResist,
		ModelConfidence:  mpc.model.Confidence,
		IterationCount:   mpc.iterationCount,
	}
}

// MPCDiagnostics contains MPC internal state for monitoring
type MPCDiagnostics struct {
	EstimatedMass    float64
	EstimatedDrag    float64
	EstimatedRolling float64
	ModelConfidence  float64
	IterationCount   int
}

// GetTargetVelocity returns the current setpoint
func (mpc *MPCController) GetTargetVelocity() float64 {
	return mpc.cfg.TargetVelocityMPS
}
