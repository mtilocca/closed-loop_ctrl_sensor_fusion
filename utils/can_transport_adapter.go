//go:build linux || darwin
// +build linux darwin

package utils

import (
	"context"
	"fmt"
	"time"

	"go.einride.tech/can"
)

// CANTransport implements Transport over SocketCAN (J1939).
type CANTransport struct {
	writer          CANWriter
	reader          CANReader
	cmap            *CANMap
	fd              *FrameDef
	vehicleStateID  uint32
	positionStateID uint32
	orientStateID   uint32
	fbChan          chan SensorFeedback
	log             *Logger
	txCount         uint64
}

// Compile-time interface check.
var _ Transport = (*CANTransport)(nil)

// NewCANTransport opens SocketCAN connections and returns a ready-to-use CANTransport.
func NewCANTransport(
	ctx context.Context,
	iface string,
	cmap *CANMap,
	fd *FrameDef,
	vehicleStateID, positionStateID, orientStateID uint32,
	log *Logger,
) (*CANTransport, error) {
	writer, err := NewSocketCANWriter(ctx, iface)
	if err != nil {
		return nil, fmt.Errorf("can writer: %w", err)
	}
	reader, err := NewSocketCANReader(ctx, iface)
	if err != nil {
		_ = writer.Close()
		return nil, fmt.Errorf("can reader: %w", err)
	}
	return &CANTransport{
		writer:          writer,
		reader:          reader,
		cmap:            cmap,
		fd:              fd,
		vehicleStateID:  vehicleStateID,
		positionStateID: positionStateID,
		orientStateID:   orientStateID,
		fbChan:          make(chan SensorFeedback, 100),
		log:             log,
	}, nil
}

// Start launches the background CAN receive goroutine.
func (c *CANTransport) Start(ctx context.Context) {
	c.log.Debug("CAN RX loop started")
	go c.receiveLoop(ctx)
}

// Feedback returns the channel delivering decoded vehicle state.
func (c *CANTransport) Feedback() <-chan SensorFeedback {
	return c.fbChan
}

// SendActuator encodes an actuator command and transmits it as a CAN frame.
func (c *CANTransport) SendActuator(ctx context.Context, v ActuatorValues) error {
	enable := 0.0
	if v.SystemEnable {
		enable = 1.0
	}
	values := map[string]float64{
		"system_enable":       enable,
		"mode":                v.Mode,
		"steer_cmd_deg":       v.SteerDeg,
		"drive_torque_cmd_nm": v.TorqueNm,
		"brake_cmd_pct":       v.BrakePct,
		"gear_position":       float64(v.GearPosition),
	}
	frame, err := c.cmap.EncodeEinrideFrame(c.fd.Name, values)
	if err != nil {
		return fmt.Errorf("encode: %w", err)
	}
	if err := c.writer.WriteFrame(ctx, frame); err != nil {
		return fmt.Errorf("write: %w", err)
	}
	c.txCount++
	if c.txCount%100 == 0 {
		c.log.Trace("CAN TX #%d id=0x%08X bytes=[%02X %02X %02X %02X %02X %02X %02X %02X] gear=%d torque=%.0f brake=%.1f steer=%.1f",
			c.txCount, uint32(frame.ID),
			frame.Data[0], frame.Data[1], frame.Data[2], frame.Data[3],
			frame.Data[4], frame.Data[5], frame.Data[6], frame.Data[7],
			v.GearPosition, v.TorqueNm, v.BrakePct, v.SteerDeg)
	}
	return nil
}

// SendShutdown transmits 10 zero-command frames spaced 10 ms apart to safely stop the vehicle.
func (c *CANTransport) SendShutdown(ctx context.Context) error {
	c.log.Info("CAN: sending shutdown commands (10 zero-torque frames)...")
	values := map[string]float64{
		"system_enable":       0.0,
		"mode":                0.0,
		"steer_cmd_deg":       0.0,
		"drive_torque_cmd_nm": 0.0,
		"brake_cmd_pct":       0.0,
		"gear_position":       0.0,
	}
	for i := 0; i < 10; i++ {
		frame, err := c.cmap.EncodeEinrideFrame(c.fd.Name, values)
		if err != nil {
			return fmt.Errorf("shutdown encode: %w", err)
		}
		shutCtx, cancel := context.WithTimeout(context.Background(), 100*time.Millisecond)
		err = c.writer.WriteFrame(shutCtx, frame)
		cancel()
		if err != nil {
			return fmt.Errorf("shutdown write: %w", err)
		}
		time.Sleep(10 * time.Millisecond)
	}
	c.log.Info("CAN: shutdown complete (10 frames sent)")
	return nil
}

// Close shuts down the CAN writer and reader connections.
func (c *CANTransport) Close() error {
	var errs []error
	if c.reader != nil {
		if err := c.reader.Close(); err != nil {
			errs = append(errs, err)
		}
	}
	if c.writer != nil {
		if err := c.writer.Close(); err != nil {
			errs = append(errs, err)
		}
	}
	if len(errs) > 0 {
		return fmt.Errorf("close errors: %v", errs)
	}
	return nil
}

// receiveLoop reads CAN frames and decodes vehicle state into fbChan.
func (c *CANTransport) receiveLoop(ctx context.Context) {
	defer c.log.Debug("CAN RX loop stopped")
	for {
		select {
		case <-ctx.Done():
			return
		default:
			frame, err := c.reader.ReadFrame(ctx)
			if err != nil {
				if ctx.Err() != nil {
					return
				}
				c.log.Error("CAN RX error: %v", err)
				continue
			}
			c.dispatchFrame(frame)
		}
	}
}

func (c *CANTransport) dispatchFrame(frame can.Frame) {
	switch frame.ID {
	case c.vehicleStateID:
		// vehicle_speed_mps: start_bit=0, length=16, signed, factor=0.01
		velocity := decodeCANSignal(frame.Data[:], 0, 16, true, 0.01, 0.0)
		c.sendFeedback(SensorFeedback{VelocityMPS: velocity, HasVelocity: true, Timestamp: time.Now()})

	case c.positionStateID:
		// pos_x_m @ bit0 32-bit, pos_y_m @ bit32 32-bit, factor=0.01
		posX := decodeCANSignal(frame.Data[:], 0, 32, true, 0.01, 0.0)
		posY := decodeCANSignal(frame.Data[:], 32, 32, true, 0.01, 0.0)
		c.sendFeedback(SensorFeedback{PosX: posX, PosY: posY, HasPosition: true, Timestamp: time.Now()})

	case c.orientStateID:
		// yaw_deg @ bit0 16-bit signed, factor=0.1
		yawDeg := decodeCANSignal(frame.Data[:], 0, 16, true, 0.1, 0.0)
		c.sendFeedback(SensorFeedback{YawDeg: yawDeg, Timestamp: time.Now()})
	}
}

func (c *CANTransport) sendFeedback(fb SensorFeedback) {
	select {
	case c.fbChan <- fb:
	default:
		// Channel full — drop oldest implicitly (non-blocking send).
	}
}

// decodeCANSignal extracts a signal value from CAN data (little-endian, DBC parameters).
func decodeCANSignal(data []byte, startBit, bitLength int, isSigned bool, factor, offset float64) float64 {
	var rawValue int64
	startByte := startBit / 8
	startBitInByte := startBit % 8

	if bitLength <= 16 && startBitInByte == 0 {
		switch bitLength {
		case 8:
			rawValue = int64(data[startByte])
		case 16:
			rawValue = int64(data[startByte]) | (int64(data[startByte+1]) << 8)
		}
	} else if bitLength == 32 && startBitInByte == 0 {
		rawValue = int64(data[startByte]) |
			(int64(data[startByte+1]) << 8) |
			(int64(data[startByte+2]) << 16) |
			(int64(data[startByte+3]) << 24)
	}

	if isSigned {
		signBit := int64(1) << (bitLength - 1)
		if rawValue&signBit != 0 {
			rawValue |= ^((int64(1) << bitLength) - 1)
		}
	}
	return float64(rawValue)*factor + offset
}
