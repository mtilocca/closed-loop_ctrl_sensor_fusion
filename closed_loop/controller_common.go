package main

// ControlOutput contains both throttle and brake commands
// Used by MPC and Auto-MPC controllers
type ControlOutput struct {
	TorqueNm   float64
	BrakePct   float64
	IsAccel    bool
	IsBrake    bool
	Confidence float64 // Model confidence (0-1)
}

// clampFloat clamps value between min and max
func clampFloat(value, min, max float64) float64 {
	if value < min {
		return min
	}
	if value > max {
		return max
	}
	return value
}

// boolToFloat converts bool to float64 (for CAN encoding)
func boolToFloat(b bool) float64 {
	if b {
		return 1.0
	}
	return 0.0
}

// boolToInt converts bool to int (for CSV logging)
func boolToInt(b bool) int {
	if b {
		return 1
	}
	return 0
}

// getControlModeStr returns a string describing the control mode
func getControlModeStr(output ControlOutput) string {
	if output.IsAccel {
		return "[ACCEL]"
	} else if output.IsBrake {
		return "[BRAKE]"
	}
	return "[COAST]"
}
