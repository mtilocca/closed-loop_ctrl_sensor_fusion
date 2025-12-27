package main

import (
	"encoding/json"
	"os"
)

type Scenario struct {
	Meta struct {
		Name        string `json:"name"`
		Version     int    `json:"version"`
		Description string `json:"description"`
	} `json:"meta"`

	Timing struct {
		DtS          float64 `json:"dt_s"`
		DurationS    float64 `json:"duration_s"`
		LogHz        float64 `json:"log_hz"`
		RealTimeMode bool    `json:"real_time_mode"`
	} `json:"timing"`

	Defaults map[string]any `json:"defaults"`

	Segments []struct {
		T0      float64 `json:"t0"`
		T1      float64 `json:"t1"`
		Comment string  `json:"comment"`

		SystemEnable     *bool    `json:"system_enable,omitempty"`
		Mode             *float64 `json:"mode,omitempty"`
		SteerCmdDeg      *float64 `json:"steer_cmd_deg,omitempty"`
		DriveTorqueCmdNm *float64 `json:"drive_torque_cmd_nm,omitempty"`
		BrakeCmdPct      *float64 `json:"brake_cmd_pct,omitempty"`
	} `json:"segments"`
}

type ActCmd struct {
	SystemEnable bool
	Mode         float64
	SteerDeg     float64
	TorqueNm     float64
	BrakePct     float64
}

func LoadScenario(path string) (Scenario, error) {
	b, err := os.ReadFile(path)
	if err != nil {
		return Scenario{}, err
	}
	var s Scenario
	if err := json.Unmarshal(b, &s); err != nil {
		return Scenario{}, err
	}
	return s, nil
}

func EvalActCmd(s *Scenario, t float64) ActCmd {
	cmd := ActCmd{
		SystemEnable: getBoolDefault(s.Defaults, "system_enable", true),
		Mode:         getNumDefault(s.Defaults, "mode", 0),
		SteerDeg:     getNumDefault(s.Defaults, "steer_cmd_deg", 0),
		TorqueNm:     getNumDefault(s.Defaults, "drive_torque_cmd_nm", 0),
		BrakePct:     getNumDefault(s.Defaults, "brake_cmd_pct", 0),
	}

	for _, seg := range s.Segments {
		t1 := seg.T1
		if t1 < 0 {
			t1 = s.Timing.DurationS
		}
		if t >= seg.T0 && t < t1 {
			if seg.SystemEnable != nil {
				cmd.SystemEnable = *seg.SystemEnable
			}
			if seg.Mode != nil {
				cmd.Mode = *seg.Mode
			}
			if seg.SteerCmdDeg != nil {
				cmd.SteerDeg = *seg.SteerCmdDeg
			}
			if seg.DriveTorqueCmdNm != nil {
				cmd.TorqueNm = *seg.DriveTorqueCmdNm
			}
			if seg.BrakeCmdPct != nil {
				cmd.BrakePct = *seg.BrakeCmdPct
			}
			return cmd
		}
	}
	return cmd
}

func getNumDefault(m map[string]any, k string, def float64) float64 {
	v, ok := m[k]
	if !ok {
		return def
	}
	switch x := v.(type) {
	case float64:
		return x
	case int:
		return float64(x)
	case bool:
		if x {
			return 1
		}
		return 0
	default:
		return def
	}
}

func getBoolDefault(m map[string]any, k string, def bool) bool {
	v, ok := m[k]
	if !ok {
		return def
	}
	if b, ok := v.(bool); ok {
		return b
	}
	if f, ok := v.(float64); ok {
		return f != 0
	}
	return def
}

func boolToFloat(b bool) float64 {
	if b {
		return 1
	}
	return 0
}
