package main

import (
	"context"
	"fmt"
	"time"

	"go.einride.tech/can"

	"dds-fusion-core/utils"
)

type RunnerConfig struct {
	Interface    string
	MapPath      string
	ScenarioPath string
	FrameName    string
}

type Runner struct {
	cfg    RunnerConfig
	log    *utils.Logger
	cmap   *utils.CANMap
	scen   Scenario
	writer CANWriter
	reader CANReader // NEW: for receiving sensor feedback
	fd     *utils.FrameDef
	pid    *PIDController // NEW: PID controller for velocity mode
}

func NewRunner(ctx context.Context, cfg RunnerConfig, log *utils.Logger) (*Runner, error) {
	cmap, err := utils.LoadCANMap(cfg.MapPath)
	if err != nil {
		return nil, fmt.Errorf("load can map: %w", err)
	}

	scen, err := LoadScenario(cfg.ScenarioPath)
	if err != nil {
		return nil, fmt.Errorf("load scenario: %w", err)
	}

	fd, err := cmap.FrameByName(cfg.FrameName)
	if err != nil {
		return nil, fmt.Errorf("frame: %w", err)
	}
	if fd.CycleMS <= 0 {
		return nil, fmt.Errorf("frame %s has invalid cycle_ms %d", fd.Name, fd.CycleMS)
	}

	// Create CAN writer (TX)
	writer, err := NewSocketCANWriter(ctx, cfg.Interface)
	if err != nil {
		return nil, err
	}

	// Create CAN reader (RX) for sensor feedback
	reader, err := NewSocketCANReader(ctx, cfg.Interface)
	if err != nil {
		writer.Close()
		return nil, err
	}

	r := &Runner{
		cfg:    cfg,
		log:    log,
		cmap:   cmap,
		scen:   scen,
		writer: writer,
		reader: reader,
		fd:     fd,
	}

	// Initialize PID controller if in velocity_pid mode
	if scen.Meta.ControlMode == "velocity_pid" {
		if scen.PIDConfig == nil {
			return nil, fmt.Errorf("velocity_pid mode requires pid_config in scenario")
		}
		r.pid = NewPIDController(*scen.PIDConfig)
		log.Info("PID controller initialized: target=%.2f m/s, Kp=%.1f, Ki=%.1f, Kd=%.1f",
			scen.PIDConfig.TargetVelocityMPS,
			scen.PIDConfig.Kp,
			scen.PIDConfig.Ki,
			scen.PIDConfig.Kd)
	}

	return r, nil
}

func (r *Runner) Close() {
	if r.reader != nil {
		_ = r.reader.Close()
	}
	if r.writer != nil {
		_ = r.writer.Close()
	}
}

func (r *Runner) Run(ctx context.Context) error {
	controlModeStr := r.scen.Meta.ControlMode
	if controlModeStr == "" {
		controlModeStr = "open_loop"
	}

	r.log.Info("Starting TX: frame=%s id=0x%X dlc=%d cycle_ms=%d iface=%s scenario=%s duration=%.2fs mode=%s",
		r.fd.Name, r.fd.ID, r.fd.DLC, r.fd.CycleMS, r.cfg.Interface,
		r.scen.Meta.Name, r.scen.Timing.DurationS, controlModeStr)

	start := time.Now()
	ticker := time.NewTicker(time.Duration(r.fd.CycleMS) * time.Millisecond)
	defer ticker.Stop()

	endAfter := time.Duration(r.scen.Timing.DurationS * float64(time.Second))
	var sent uint64

	// Current velocity (updated from CAN RX)
	currentVelocity := 0.0
	lastRxTime := time.Now()

	// Start background RX goroutine
	rxChan := make(chan SensorFeedback, 100)
	go r.receiveLoop(ctx, rxChan)

	for {
		select {
		case <-ctx.Done():
			r.log.Warn("Context canceled; stopping TX")
			r.log.Info("Completed TX. frames_sent=%d", sent)
			return ctx.Err()

		case feedback := <-rxChan:
			// Update current velocity from sensor feedback
			currentVelocity = feedback.VelocityMPS
			lastRxTime = time.Now()
			r.log.Trace("RX velocity=%.3f m/s", currentVelocity)

		case now := <-ticker.C:
			elapsed := now.Sub(start)
			if elapsed > endAfter {
				r.log.Info("Completed TX. frames_sent=%d", sent)
				return nil
			}

			t := elapsed.Seconds()
			dt := float64(r.fd.CycleMS) / 1000.0

			// Check if we're receiving sensor data
			rxAge := now.Sub(lastRxTime)
			if rxAge > 500*time.Millisecond && r.scen.Meta.ControlMode == "velocity_pid" {
				r.log.Warn("No sensor feedback for %.1f ms - PID may be unreliable", rxAge.Seconds()*1000)
			}

			// Evaluate base command from scenario
			cmd := EvalActCmd(&r.scen, t)

			// Override torque with PID if in velocity_pid mode
			if r.scen.Meta.ControlMode == "velocity_pid" && r.pid != nil {
				pidTorque := r.pid.Update(currentVelocity, dt)
				cmd.TorqueNm = pidTorque

				// Log PID diagnostics periodically
				if sent%100 == 0 {
					diag := r.pid.GetDiagnostics()
					r.log.Debug("PID: v=%.2f err=%.3f torque=%.1f P=%.1f I=%.1f",
						currentVelocity, diag.Error, pidTorque, diag.P, diag.I)
				}
			}

			// Encode and transmit CAN frame
			values := map[string]float64{
				"system_enable":       boolToFloat(cmd.SystemEnable),
				"mode":                cmd.Mode,
				"steer_cmd_deg":       cmd.SteerDeg,
				"drive_torque_cmd_nm": cmd.TorqueNm,
				"brake_cmd_pct":       cmd.BrakePct,
			}

			frame, err := r.cmap.EncodeEinrideFrame(r.fd.Name, values)
			if err != nil {
				r.log.Error("Encode failed at t=%.3f: %v", t, err)
				return err
			}

			if err := r.writer.WriteFrame(ctx, frame); err != nil {
				r.log.Critical("Transmit failed at t=%.3f: %v", t, err)
				return err
			}

			sent++
			r.log.Trace("TX t=%.3f id=0x%X len=%d data=% X enable=%v mode=%.0f steer=%.2f torque=%.2f brake=%.2f",
				t, uint32(frame.ID), frame.Length, frame.Data[:frame.Length],
				cmd.SystemEnable, cmd.Mode, cmd.SteerDeg, cmd.TorqueNm, cmd.BrakePct)
		}
	}
}

// SensorFeedback contains decoded sensor data from CAN RX
type SensorFeedback struct {
	VelocityMPS float64
	YawRateRPS  float64
	Timestamp   time.Time
}

// receiveLoop continuously reads CAN frames and decodes sensor data
func (r *Runner) receiveLoop(ctx context.Context, feedback chan<- SensorFeedback) {
	r.log.Debug("RX loop started")
	defer r.log.Debug("RX loop stopped")

	for {
		select {
		case <-ctx.Done():
			return
		default:
			frame, err := r.reader.ReadFrame(ctx)
			if err != nil {
				if ctx.Err() != nil {
					return
				}
				r.log.Error("RX error: %v", err)
				continue
			}

			// Decode relevant sensor frames
			// Use VEHICLE_STATE_1 (0x300) for truth velocity feedback
			// This is more reliable than GNSS during PID tuning

			if frame.ID == 0x300 { // VEHICLE_STATE_1 frame contains truth velocity
				// Decode velocity from VEHICLE_STATE_1 frame
				// vehicle_speed_mps: start_bit=0, length=16, signed, factor=0.01

				velocity := r.decodeSignal(frame.Data[:], 0, 16, true, 0.01, 0.0)

				select {
				case feedback <- SensorFeedback{
					VelocityMPS: velocity,
					Timestamp:   time.Now(),
				}:
				default:
					// Channel full, skip
				}
			}

			r.log.Trace("RX id=0x%X len=%d data=% X", uint32(frame.ID), frame.Length, frame.Data[:frame.Length])
		}
	}
}

// decodeSignal extracts a signal value from CAN data using DBC parameters
func (r *Runner) decodeSignal(data []byte, startBit, bitLength int, isSigned bool, factor, offset float64) float64 {
	// Extract raw value (little-endian bit extraction)
	var rawValue int64

	startByte := startBit / 8
	startBitInByte := startBit % 8

	// Simple extraction for aligned signals
	if bitLength <= 16 && startBitInByte == 0 {
		if bitLength == 8 {
			rawValue = int64(data[startByte])
		} else if bitLength == 16 {
			rawValue = int64(data[startByte]) | (int64(data[startByte+1]) << 8)
		}
	} else if bitLength == 32 && startBitInByte == 0 {
		rawValue = int64(data[startByte]) |
			(int64(data[startByte+1]) << 8) |
			(int64(data[startByte+2]) << 16) |
			(int64(data[startByte+3]) << 24)
	}

	// Handle signed values
	if isSigned {
		signBit := int64(1) << (bitLength - 1)
		if rawValue&signBit != 0 {
			rawValue |= ^((int64(1) << bitLength) - 1)
		}
	}

	// Apply scaling
	return float64(rawValue)*factor + offset
}

// compile-time assurance the transmitted frame type is what we expect
var _ can.Frame
