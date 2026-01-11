package control

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

// AdaptivePIDConfig holds adaptive PID parameters
type AdaptivePIDConfig struct {
	TargetVelocityMPS float64 `json:"target_velocity_mps"`
	VehicleMassKg     float64 `json:"vehicle_mass_kg"`
	InitialKp         float64 `json:"initial_kp"`
	InitialKi         float64 `json:"initial_ki"`
	InitialKd         float64 `json:"initial_kd"`
	AdaptationRate    float64 `json:"adaptation_rate"`
}

// MPCConfig holds MPC controller parameters
type MPCConfig struct {
	TargetVelocityMPS float64 `json:"target_velocity_mps"`
	PredictionHorizon int     `json:"prediction_horizon"`
	ControlHorizon    int     `json:"control_horizon"`
	TimeStep          float64 `json:"time_step"`

	WeightVelocity float64 `json:"weight_velocity"`
	WeightAccel    float64 `json:"weight_accel"`
	WeightJerk     float64 `json:"weight_jerk"`

	MaxTorque     float64 `json:"max_torque"`
	MaxBrakeForce float64 `json:"max_brake_force"`
	MaxAccel      float64 `json:"max_accel"`
	MaxDecel      float64 `json:"max_decel"`

	EnableAdaptation bool    `json:"enable_adaptation"`
	AdaptationRate   float64 `json:"adaptation_rate"`
}

// AutoMPCConfig holds minimal autonomous MPC parameters
type AutoMPCConfig struct {
	TargetVelocityMPS float64 `json:"target_velocity_mps"`
	AggressiveTuning  bool    `json:"aggressive_tuning"`
	LearningRate      float64 `json:"learning_rate"`
}
