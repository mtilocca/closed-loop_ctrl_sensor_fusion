package control

// ControlOutput contains both throttle and brake commands
// Used by MPC and Auto-MPC controllers
type ControlOutput struct {
	TorqueNm   float64
	BrakePct   float64
	IsAccel    bool
	IsBrake    bool
	Confidence float64 // Model confidence (0-1)
}

// ClampFloat clamps value between min and max
func ClampFloat(value, min, max float64) float64 {
	if value < min {
		return min
	}
	if value > max {
		return max
	}
	return value
}

// BoolToFloat converts bool to float64 (for CAN encoding)
func BoolToFloat(b bool) float64 {
	if b {
		return 1.0
	}
	return 0.0
}

// BoolToInt converts bool to int (for CSV logging)
func BoolToInt(b bool) int {
	if b {
		return 1
	}
	return 0
}

// GetControlModeStr returns a string describing the control mode
func GetControlModeStr(output ControlOutput) string {
	if output.IsAccel {
		return "[ACCEL]"
	} else if output.IsBrake {
		return "[BRAKE]"
	}
	return "[COAST]"
}
