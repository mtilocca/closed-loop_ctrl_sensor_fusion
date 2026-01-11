package control

import "math"

// PIDController implements a discrete PID controller for velocity tracking
// with automatic brake conversion for heavy vehicles
type PIDController struct {
	cfg PIDConfig

	// State
	integral     float64
	prevError    float64
	prevVelocity float64
	initialized  bool

	// Vehicle-specific brake conversion parameters
	maxBrakeTorqueNm float64 // Equivalent torque for max brake force

	// Overshoot protection
	overshootDuration float64 // How long we've been above target (seconds)
}

// NewPIDController creates a new PID controller with given configuration
func NewPIDController(cfg PIDConfig) *PIDController {
	// Calculate max brake torque equivalent for heavy vehicles
	// For XCMG XDE360: 180 kN brake force * 1.95m radius / 28 gear ratio = 12536 Nm
	// This is the torque equivalent when brakes are at 100%
	maxBrakeTorqueNm := calculateMaxBrakeTorque()

	return &PIDController{
		cfg:              cfg,
		initialized:      false,
		maxBrakeTorqueNm: maxBrakeTorqueNm,
	}
}

// calculateMaxBrakeTorque computes the torque equivalent of max brake force
// This is vehicle-specific and should match your vehicle YAML config
func calculateMaxBrakeTorque() float64 {
	// XCMG XDE360 parameters
	const (
		maxBrakeForceN = 180000.0 // 180 kN max brake force
		wheelRadiusM   = 1.95     // 1.95m wheel radius
		gearRatio      = 28.0     // 28:1 gear ratio
	)

	// Brake torque = brake_force * wheel_radius / gear_ratio
	return maxBrakeForceN * wheelRadiusM / gearRatio
}

// Reset clears the PID state
func (pid *PIDController) Reset() {
	pid.integral = 0.0
	pid.prevError = 0.0
	pid.prevVelocity = 0.0
	pid.initialized = false
	pid.overshootDuration = 0.0
}

// Update computes the PID control output given current velocity and time delta
//
// Returns: ControlOutput with torque and brake commands
func (pid *PIDController) Update(currentVelocity float64, dt float64) ControlOutput {
	// Initialize on first call
	if !pid.initialized {
		pid.prevVelocity = currentVelocity
		pid.prevError = pid.cfg.TargetVelocityMPS - currentVelocity
		pid.initialized = true
		// Return a reasonable initial torque (50% of max)
		return ControlOutput{
			TorqueNm:   pid.cfg.MaxTorqueNm * 0.5,
			BrakePct:   0.0,
			IsAccel:    true,
			IsBrake:    false,
			Confidence: 1.0, // PID is always "confident"
		}
	}

	// Compute error
	error := pid.cfg.TargetVelocityMPS - currentVelocity

	// CRITICAL OVERSHOOT PROTECTION: Track how long we've been above target
	if error < 0 {
		// We're overshooting (velocity > target)
		pid.overshootDuration += dt

		// If overshooting for more than 2 seconds, IMMEDIATELY cut torque to zero
		// and zero out integral to force braking
		if pid.overshootDuration > 2.0 {
			pid.integral = 0.0 // Kill integral immediately

			// Return immediate brake command
			return ControlOutput{
				TorqueNm:   0.0,
				BrakePct:   math.Min(math.Abs(error)*20.0, 100.0), // Aggressive braking
				IsAccel:    false,
				IsBrake:    true,
				Confidence: 1.0,
			}
		}
	} else {
		// Below target - reset overshoot timer
		pid.overshootDuration = 0.0
	}

	// Proportional term
	p := pid.cfg.Kp * error

	// Integral term with anti-windup and deadband
	// DEADBAND: Don't accumulate integral if error is tiny (within ±0.05 m/s)
	if math.Abs(error) > 0.05 {
		pid.integral += error * dt
	}

	// CRITICAL: Reset integral when crossing setpoint to prevent overshoot
	// If error changed sign, we've crossed the setpoint - reduce integral aggressively
	if (pid.prevError > 0 && error < 0) || (pid.prevError < 0 && error > 0) {
		pid.integral *= 0.1 // Keep only 10% when crossing setpoint
	}

	// Clamp integral
	if pid.integral > pid.cfg.IntegralLimit {
		pid.integral = pid.cfg.IntegralLimit
	} else if pid.integral < -pid.cfg.IntegralLimit {
		pid.integral = -pid.cfg.IntegralLimit
	}
	i := pid.cfg.Ki * pid.integral

	// Derivative term (using error derivative to avoid derivative kick)
	var d float64
	if dt > 0 {
		d = pid.cfg.Kd * (error - pid.prevError) / dt
	}

	// Compute total control output (can be positive or negative)
	controlTorque := p + i + d

	// Apply saturation limits
	if controlTorque > pid.cfg.MaxTorqueNm {
		controlTorque = pid.cfg.MaxTorqueNm
		// Anti-windup: back-calculate integral
		pid.integral = (controlTorque - p - d) / pid.cfg.Ki
	} else if controlTorque < pid.cfg.MinTorqueNm {
		controlTorque = pid.cfg.MinTorqueNm
		// Anti-windup: back-calculate integral
		pid.integral = (controlTorque - p - d) / pid.cfg.Ki
	}

	// Update state for next iteration
	pid.prevError = error
	pid.prevVelocity = currentVelocity

	// ========= BRAKE CONVERSION LOGIC =========
	// Convert control torque to actuator commands
	return pid.torqueToActuators(controlTorque)
}

// torqueToActuators converts PID output torque to motor/brake commands
// Positive torque → motor acceleration
// Negative torque → brake application
func (pid *PIDController) torqueToActuators(controlTorque float64) ControlOutput {
	var output ControlOutput
	output.Confidence = 1.0 // PID is always confident in its output

	if controlTorque >= 0 {
		// Accelerating - use motor torque
		output.TorqueNm = controlTorque
		output.BrakePct = 0.0
		output.IsAccel = true
		output.IsBrake = false
	} else {
		// Braking - convert negative torque to brake percentage
		output.TorqueNm = 0.0 // CRITICAL: No motor torque when braking!

		// Convert torque magnitude to brake percentage
		brakeTorqueMagnitude := math.Abs(controlTorque)
		output.BrakePct = (brakeTorqueMagnitude / pid.maxBrakeTorqueNm) * 100.0

		// Clamp to 0-100% range
		if output.BrakePct > 100.0 {
			output.BrakePct = 100.0
		}

		output.IsAccel = false
		output.IsBrake = true
	}

	return output
}

// GetDiagnostics returns current PID state for logging/debugging
func (pid *PIDController) GetDiagnostics() PIDDiagnostics {
	return PIDDiagnostics{
		Error:    pid.prevError,
		Integral: pid.integral,
		P:        pid.cfg.Kp * pid.prevError,
		I:        pid.cfg.Ki * pid.integral,
	}
}

// PIDDiagnostics contains PID internal state for monitoring
type PIDDiagnostics struct {
	Error    float64
	Integral float64
	P        float64
	I        float64
}

// GetTargetVelocity returns the current target velocity
func (pid *PIDController) GetTargetVelocity() float64 {
	return pid.cfg.TargetVelocityMPS
}

// SetTargetVelocity updates the target velocity (useful for adaptive scenarios)
func (pid *PIDController) SetTargetVelocity(target float64) {
	pid.cfg.TargetVelocityMPS = target
}

// GetError returns the most recent velocity error
func (pid *PIDController) GetError() float64 {
	return pid.prevError
}

// GetIntegral returns the current integral term value
func (pid *PIDController) GetIntegral() float64 {
	return pid.integral
}
