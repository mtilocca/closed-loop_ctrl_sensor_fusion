package main

// PIDConfig holds PID controller parameters
type PIDConfig struct {
	TargetVelocityMPS float64 `json:"target_velocity_mps"`
	Kp                float64 `json:"kp"`
	Ki                float64 `json:"ki"`
	Kd                float64 `json:"kd"`
	MaxTorqueNm       float64 `json:"max_torque_nm"`
	MinTorqueNm       float64 `json:"min_torque_nm"`
	IntegralLimit     float64 `json:"integral_limit"`
}

// PIDController implements a discrete PID controller for velocity tracking
type PIDController struct {
	cfg PIDConfig

	// State
	integral     float64
	prevError    float64
	prevVelocity float64
	initialized  bool
}

// NewPIDController creates a new PID controller with given configuration
func NewPIDController(cfg PIDConfig) *PIDController {
	return &PIDController{
		cfg:         cfg,
		initialized: false,
	}
}

// Reset clears the PID state
func (pid *PIDController) Reset() {
	pid.integral = 0.0
	pid.prevError = 0.0
	pid.prevVelocity = 0.0
	pid.initialized = false
}

// Update computes the PID control output given current velocity and time delta
//
// Returns: torque command (Nm)
func (pid *PIDController) Update(currentVelocity float64, dt float64) float64 {
	// Initialize on first call
	if !pid.initialized {
		pid.prevVelocity = currentVelocity
		pid.prevError = pid.cfg.TargetVelocityMPS - currentVelocity
		pid.initialized = true
		// Return a reasonable initial torque
		return pid.cfg.MaxTorqueNm * 0.5
	}

	// Compute error
	error := pid.cfg.TargetVelocityMPS - currentVelocity

	// Proportional term
	p := pid.cfg.Kp * error

	// Integral term with anti-windup
	pid.integral += error * dt
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

	// Compute total control output
	torque := p + i + d

	// Apply saturation limits
	if torque > pid.cfg.MaxTorqueNm {
		torque = pid.cfg.MaxTorqueNm
		// Anti-windup: back-calculate integral
		pid.integral = (torque - p - d) / pid.cfg.Ki
	} else if torque < pid.cfg.MinTorqueNm {
		torque = pid.cfg.MinTorqueNm
		// Anti-windup: back-calculate integral
		pid.integral = (torque - p - d) / pid.cfg.Ki
	}

	// Update state for next iteration
	pid.prevError = error
	pid.prevVelocity = currentVelocity

	return torque
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
