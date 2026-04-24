package utils

import (
	"context"
	"time"
)

// ActuatorValues is the transport-agnostic actuator command.
type ActuatorValues struct {
	SystemEnable bool
	GearPosition int     // 0=N 1=F 2=R
	Mode         float64 // reserved, typically 0
	TorqueNm     float64
	SteerDeg     float64
	BrakePct     float64
}

// SensorFeedback contains decoded vehicle state from any transport.
type SensorFeedback struct {
	VelocityMPS float64
	YawRateRPS  float64
	PosX        float64
	PosY        float64
	YawDeg      float64
	HasVelocity bool // true when VelocityMPS is a fresh reading
	HasPosition bool // true when PosX/PosY are valid
	Timestamp   time.Time
}

// Transport is the seam between the runner and the physical bus.
// Both CAN and MQTT implement this interface.
type Transport interface {
	// Start launches the receive goroutine. Call once before reading Feedback().
	Start(ctx context.Context)
	// SendActuator publishes one actuator command.
	SendActuator(ctx context.Context, v ActuatorValues) error
	// SendShutdown sends a safe zero-command to the vehicle.
	// CAN: 10 frames spaced 10 ms apart. MQTT: one zero-command message.
	SendShutdown(ctx context.Context) error
	// Feedback returns the channel delivering decoded vehicle state.
	Feedback() <-chan SensorFeedback
	// Close tears down connections gracefully.
	Close() error
}
