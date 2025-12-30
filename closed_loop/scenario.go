package main

import (
	"encoding/json"
	"fmt"
	"os"
)

// Scenario defines a complete test scenario
type Scenario struct {
	Meta              ScenarioMeta       `json:"meta"`
	Timing            ScenarioTiming     `json:"timing"`
	Defaults          ActuatorCmd        `json:"defaults"`
	Segments          []ScenarioSegment  `json:"segments"`
	PIDConfig         *PIDConfig         `json:"pid_config,omitempty"`          // Optional PID config
	AdaptivePIDConfig *AdaptivePIDConfig `json:"adaptive_pid_config,omitempty"` // Optional Adaptive PID config
	MPCConfig         *MPCConfig         `json:"mpc_config,omitempty"`          // Optional MPC config
	AutoMPCConfig     *AutoMPCConfig     `json:"auto_mpc_config,omitempty"`     // Optional Auto-MPC config
}

// ScenarioMeta contains scenario metadata
type ScenarioMeta struct {
	Name        string `json:"name"`
	Version     int    `json:"version"`
	Description string `json:"description"`
	ControlMode string `json:"control_mode,omitempty"` // "open_loop", "velocity_pid", "adaptive_velocity_pid", "velocity_mpc", or "auto_mpc"
}

// ScenarioTiming defines timing parameters
type ScenarioTiming struct {
	DtS          float64 `json:"dt_s"`
	DurationS    float64 `json:"duration_s"`
	LogHz        float64 `json:"log_hz"`
	RealTimeMode bool    `json:"real_time_mode"`
}

// ScenarioSegment defines a time segment with actuator commands
type ScenarioSegment struct {
	T0       float64 `json:"t0"`
	T1       float64 `json:"t1"`
	SteerDeg float64 `json:"steer_cmd_deg,omitempty"`
	TorqueNm float64 `json:"drive_torque_cmd_nm,omitempty"`
	BrakePct float64 `json:"brake_cmd_pct,omitempty"`
	Comment  string  `json:"comment,omitempty"`
}

// ActuatorCmd represents a complete actuator command set
type ActuatorCmd struct {
	SystemEnable bool    `json:"system_enable"`
	Mode         float64 `json:"mode"`
	SteerDeg     float64 `json:"steer_cmd_deg"`
	TorqueNm     float64 `json:"drive_torque_cmd_nm"`
	BrakePct     float64 `json:"brake_cmd_pct"`
}

// LoadScenario loads a scenario from JSON file
func LoadScenario(path string) (Scenario, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return Scenario{}, fmt.Errorf("read file: %w", err)
	}

	var scen Scenario
	if err := json.Unmarshal(data, &scen); err != nil {
		return Scenario{}, fmt.Errorf("unmarshal: %w", err)
	}

	// Validate
	if scen.Timing.DurationS <= 0 {
		return Scenario{}, fmt.Errorf("invalid duration_s: %f", scen.Timing.DurationS)
	}

	// Set default control mode if not specified
	if scen.Meta.ControlMode == "" {
		scen.Meta.ControlMode = "open_loop"
	}

	// Validate PID config if in velocity_pid mode
	if scen.Meta.ControlMode == "velocity_pid" {
		if scen.PIDConfig == nil {
			return Scenario{}, fmt.Errorf("velocity_pid mode requires pid_config")
		}
		if scen.PIDConfig.TargetVelocityMPS <= 0 {
			return Scenario{}, fmt.Errorf("invalid target_velocity_mps: %f", scen.PIDConfig.TargetVelocityMPS)
		}
	}

	// Validate adaptive PID config if in adaptive_velocity_pid mode
	if scen.Meta.ControlMode == "adaptive_velocity_pid" {
		if scen.AdaptivePIDConfig == nil {
			return Scenario{}, fmt.Errorf("adaptive_velocity_pid mode requires adaptive_pid_config")
		}
		if scen.AdaptivePIDConfig.TargetVelocityMPS <= 0 {
			return Scenario{}, fmt.Errorf("invalid target_velocity_mps: %f", scen.AdaptivePIDConfig.TargetVelocityMPS)
		}
		if scen.AdaptivePIDConfig.VehicleMassKg <= 0 {
			return Scenario{}, fmt.Errorf("invalid vehicle_mass_kg: %f", scen.AdaptivePIDConfig.VehicleMassKg)
		}
	}

	return scen, nil
}

// EvalActCmd evaluates the scenario at time t and returns actuator commands
func EvalActCmd(scen *Scenario, t float64) ActuatorCmd {
	cmd := scen.Defaults

	// Find active segment
	for _, seg := range scen.Segments {
		t1 := seg.T1
		if t1 < 0 {
			t1 = scen.Timing.DurationS
		}

		if t >= seg.T0 && t < t1 {
			// Override with segment values (only if explicitly set in JSON)
			cmd.SteerDeg = seg.SteerDeg
			if seg.TorqueNm != 0 || scen.Meta.ControlMode != "velocity_pid" {
				cmd.TorqueNm = seg.TorqueNm
			}
			cmd.BrakePct = seg.BrakePct
			break
		}
	}

	return cmd
}
