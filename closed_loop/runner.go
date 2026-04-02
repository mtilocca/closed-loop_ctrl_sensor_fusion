package main

import (
	"context"
	"fmt"
	"math"
	"os"
	"time"

	control "closed_loop_ctrl_sensor_fusion/closed_loop/longitudinal_control"
	"closed_loop_ctrl_sensor_fusion/utils"

	"go.einride.tech/can"
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
	writer utils.CANWriter
	reader utils.CANReader
	fd     *utils.FrameDef

	// Controllers (only one active at a time)
	pid     *control.PIDController
	mpc     *control.MPCController
	autoMPC *control.AutoMPCController // Added Auto-MPC support

	// J1939 CAN IDs for RX frames (looked up from map at startup)
	vehicleStateID  uint32
	positionStateID uint32
	orientStateID   uint32

	// Waypoint tracking (waypoint_pid mode)
	wpIdx int
	posX  float64
	posY  float64
	posYawDeg float64

	// CSV logging
	csvFile *os.File
	csvPath string
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

	// Look up J1939 IDs for the RX feedback frames
	vsFrame, err := cmap.FrameByName("VEHICLE_STATE_1")
	if err != nil {
		return nil, fmt.Errorf("VEHICLE_STATE_1 not in CAN map: %w", err)
	}
	posFrame, err := cmap.FrameByName("POSITION_STATE")
	if err != nil {
		return nil, fmt.Errorf("POSITION_STATE not in CAN map: %w", err)
	}
	orientFrame, err := cmap.FrameByName("ORIENTATION_STATE")
	if err != nil {
		return nil, fmt.Errorf("ORIENTATION_STATE not in CAN map: %w", err)
	}

	// Create CAN writer (TX)
	writer, err := utils.NewSocketCANWriter(ctx, cfg.Interface)
	if err != nil {
		return nil, err
	}

	// Create CAN reader (RX) for sensor feedback
	reader, err := utils.NewSocketCANReader(ctx, cfg.Interface)
	if err != nil {
		writer.Close()
		return nil, err
	}

	// Generate CSV filename from scenario name
	csvPath := generateCSVFilename(cfg.ScenarioPath, scen.Meta.ControlMode)

	r := &Runner{
		cfg:             cfg,
		log:             log,
		cmap:            cmap,
		scen:            scen,
		writer:          writer,
		reader:          reader,
		fd:              fd,
		csvPath:         csvPath,
		vehicleStateID:  vsFrame.ID,
		positionStateID: posFrame.ID,
		orientStateID:   orientFrame.ID,
	}

	// Initialize appropriate controller based on control mode
	switch scen.Meta.ControlMode {
	case "velocity_pid":
		if scen.PIDConfig == nil {
			return nil, fmt.Errorf("velocity_pid mode requires pid_config in scenario")
		}
		r.pid = control.NewPIDController(*scen.PIDConfig)
		log.Info("PID controller initialized: target=%.2f m/s, Kp=%.1f, Ki=%.1f, Kd=%.1f",
			scen.PIDConfig.TargetVelocityMPS,
			scen.PIDConfig.Kp,
			scen.PIDConfig.Ki,
			scen.PIDConfig.Kd)

		// Create CSV log file for PID
		csvFile, err := os.Create(r.csvPath)
		if err != nil {
			return nil, fmt.Errorf("create CSV log: %w", err)
		}
		r.csvFile = csvFile

		// PID CSV header (unified format)
		_, err = csvFile.WriteString("time_s,target_velocity_mps,actual_velocity_mps,error_mps," +
			"torque_nm,brake_pct,p_term_nm,i_term_nm,d_term_nm,integral," +
			"est_mass_kg,est_drag,model_conf,steering_deg,gear_position\n")
		if err != nil {
			csvFile.Close()
			return nil, fmt.Errorf("write CSV header: %w", err)
		}
		log.Info("PID CSV logging to: %s", r.csvPath)

	case "velocity_mpc":
		if scen.MPCConfig == nil {
			return nil, fmt.Errorf("velocity_mpc mode requires mpc_config in scenario")
		}
		r.mpc = control.NewMPCController(*scen.MPCConfig)
		log.Info("MPC controller initialized: target=%.2f m/s, horizon=%d, adaptation=%v",
			scen.MPCConfig.TargetVelocityMPS,
			scen.MPCConfig.PredictionHorizon,
			scen.MPCConfig.EnableAdaptation)

		// Create CSV log file for MPC
		csvFile, err := os.Create(r.csvPath)
		if err != nil {
			return nil, fmt.Errorf("create CSV log: %w", err)
		}
		r.csvFile = csvFile

		// MPC CSV header (unified format)
		_, err = csvFile.WriteString("time_s,target_velocity_mps,actual_velocity_mps,error_mps," +
			"torque_nm,brake_pct,p_term_nm,i_term_nm,d_term_nm,integral," +
			"est_mass_kg,est_drag,model_conf,steering_deg,gear_position\n")
		if err != nil {
			csvFile.Close()
			return nil, fmt.Errorf("write CSV header: %w", err)
		}
		log.Info("MPC CSV logging to: %s", r.csvPath)

	case "auto_mpc":
		if scen.AutoMPCConfig == nil {
			return nil, fmt.Errorf("auto_mpc mode requires auto_mpc_config in scenario")
		}
		r.autoMPC = control.NewAutoMPCController(*scen.AutoMPCConfig)
		log.Info("Auto-MPC initialized: target=%.2f m/s, autonomous learning enabled",
			scen.AutoMPCConfig.TargetVelocityMPS)

		// CSV for auto-MPC
		csvFile, err := os.Create(r.csvPath)
		if err != nil {
			return nil, fmt.Errorf("create CSV log: %w", err)
		}
		r.csvFile = csvFile

		// Auto-MPC CSV header (unified format)
		_, err = csvFile.WriteString("time_s,target_velocity_mps,actual_velocity_mps,error_mps," +
			"torque_nm,brake_pct,p_term_nm,i_term_nm,d_term_nm,integral," +
			"est_mass_kg,est_drag,model_conf,steering_deg,gear_position\n")
		if err != nil {
			csvFile.Close()
			return nil, fmt.Errorf("write CSV header: %w", err)
		}
		log.Info("Auto-MPC CSV logging to: %s", r.csvPath)

	case "waypoint_pid":
		if len(scen.Waypoints) == 0 {
			return nil, fmt.Errorf("waypoint_pid mode requires at least one waypoint in scenario")
		}
		if scen.PIDConfig == nil {
			return nil, fmt.Errorf("waypoint_pid mode requires pid_config in scenario")
		}
		r.pid = control.NewPIDController(*scen.PIDConfig)
		log.Info("Waypoint-PID controller initialized: %d waypoints, Kp=%.1f, Ki=%.1f, Kd=%.1f",
			len(scen.Waypoints),
			scen.PIDConfig.Kp,
			scen.PIDConfig.Ki,
			scen.PIDConfig.Kd)

		csvFile, err := os.Create(r.csvPath)
		if err != nil {
			return nil, fmt.Errorf("create CSV log: %w", err)
		}
		r.csvFile = csvFile

		_, err = csvFile.WriteString("time_s,target_velocity_mps,actual_velocity_mps,error_mps," +
			"torque_nm,brake_pct,p_term_nm,i_term_nm,d_term_nm,integral," +
			"est_mass_kg,est_drag,model_conf,steering_deg,gear_position," +
			"waypoint_idx,cross_track_err_m,heading_err_deg\n")
		if err != nil {
			csvFile.Close()
			return nil, fmt.Errorf("write CSV header: %w", err)
		}
		log.Info("Waypoint-PID CSV logging to: %s", r.csvPath)

	case "open_loop", "":
		log.Info("Open-loop mode (no controller)")

	default:
		return nil, fmt.Errorf("unsupported control mode: %s", scen.Meta.ControlMode)
	}

	return r, nil
}

func (r *Runner) Close() {
	if r.csvFile != nil {
		r.csvFile.Close()
		r.log.Info("Controller CSV log saved to: %s", r.csvPath)
	}
	if r.reader != nil {
		_ = r.reader.Close()
	}
	if r.writer != nil {
		_ = r.writer.Close()
	}
}

// sendShutdownCommand sends zero torque/brake commands to safely stop the vehicle
func (r *Runner) sendShutdownCommand() {
	r.log.Info("Sending shutdown commands (zero torque/brake)...")

	// Create zero command
	values := map[string]float64{
		"system_enable":       0.0, // Disable system
		"mode":                0.0,
		"steer_cmd_deg":       0.0,
		"drive_torque_cmd_nm": 0.0,
		"brake_cmd_pct":       0.0,
		"gear_position":       0.0, // Neutral on shutdown
	}

	// Send multiple times to ensure delivery
	for i := 0; i < 10; i++ {
		frame, err := r.cmap.EncodeEinrideFrame(r.fd.Name, values)
		if err != nil {
			r.log.Error("Shutdown encode failed: %v", err)
			return
		}

		// Use background context since main context may be canceled
		shutdownCtx, cancel := context.WithTimeout(context.Background(), 100*time.Millisecond)
		if err := r.writer.WriteFrame(shutdownCtx, frame); err != nil {
			r.log.Error("Shutdown TX failed: %v", err)
			cancel()
			return
		}
		cancel()

		time.Sleep(10 * time.Millisecond) // 10ms between frames
	}

	r.log.Info("Shutdown commands sent (10 frames)")
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

	// Position/orientation for waypoint_pid mode
	currentX := 0.0
	currentY := 0.0
	currentYawDeg := 0.0

	// Gear state tracking for the safety guard
	currentGear := 1       // Start in Forward
	gearChangePending := false

	// Start background RX goroutine
	rxChan := make(chan SensorFeedback, 100)
	go r.receiveLoop(ctx, rxChan)

	for {
		select {
		case <-ctx.Done():
			r.log.Warn("Context canceled; stopping TX")
			r.sendShutdownCommand()
			r.log.Info("Completed TX. frames_sent=%d", sent)
			return ctx.Err()

		case feedback := <-rxChan:
			if !feedback.HasPosition && feedback.YawDeg == 0 {
				// Velocity-only update from 0x300
				currentVelocity = feedback.VelocityMPS
				lastRxTime = time.Now()
			}
			if feedback.HasPosition {
				currentX = feedback.PosX
				currentY = feedback.PosY
				r.posX = feedback.PosX
				r.posY = feedback.PosY
			}
			if !feedback.HasPosition && feedback.YawDeg != 0 {
				currentYawDeg = feedback.YawDeg
				r.posYawDeg = feedback.YawDeg
			}

		case now := <-ticker.C:
			elapsed := now.Sub(start)
			if elapsed > endAfter {
				r.sendShutdownCommand()
				r.log.Info("Completed TX. frames_sent=%d", sent)
				return nil
			}

			t := elapsed.Seconds()
			dt := float64(r.fd.CycleMS) / 1000.0

			// Check if we're receiving sensor data
			rxAge := now.Sub(lastRxTime)
			if rxAge > 500*time.Millisecond && (r.pid != nil || r.mpc != nil || r.autoMPC != nil) {
				r.log.Warn("No sensor feedback for %.1f ms - controller may be unreliable", rxAge.Seconds()*1000)
			}

			// Evaluate base command from scenario
			segEval := EvalSegment(&r.scen, t)
			cmd := segEval.Cmd

			// === GEAR-CHANGE SAFETY GUARD ===
			// Only allow gear change when vehicle is stopped; otherwise hold current gear and brake.
			const gearChangeThresholdMPS = 0.2
			desiredGear := cmd.GearPosition
			if desiredGear != currentGear {
				if math.Abs(currentVelocity) > gearChangeThresholdMPS {
					r.log.Warn("Gear change %d→%d inhibited: |v|=%.3f m/s - braking to stop",
						currentGear, desiredGear, currentVelocity)
					gearChangePending = true
					cmd.GearPosition = currentGear
					cmd.TorqueNm = 0.0
					cmd.BrakePct = 100.0
				} else {
					r.log.Info("Gear change: %d → %d (|v|=%.3f m/s)", currentGear, desiredGear, currentVelocity)
					currentGear = desiredGear
					cmd.GearPosition = currentGear
					gearChangePending = false
					if r.pid != nil {
						r.pid.Reset()
					}
				}
			} else {
				gearChangePending = false
				cmd.GearPosition = currentGear
			}
			// === END GEAR-CHANGE SAFETY GUARD ===

			// Apply controller based on mode (skipped while gear change is pending)
			switch r.scen.Meta.ControlMode {
			case "velocity_pid":
				if r.pid != nil && !gearChangePending {
					r.applyPID(&cmd, currentVelocity, dt, t, sent, segEval.TargetVelocityMPS)
				}

			case "velocity_mpc":
				if r.mpc != nil && !gearChangePending {
					r.applyMPC(&cmd, currentVelocity, dt, t, sent)
				}

			case "auto_mpc":
				if r.autoMPC != nil && !gearChangePending {
					r.applyAutoMPC(&cmd, currentVelocity, dt, t, sent)
				}

			case "waypoint_pid":
				if r.pid != nil && !gearChangePending {
					wpEval := EvalWaypoint(&r.scen, currentX, currentY, &r.wpIdx)
					if wpEval.Done {
						cmd.TorqueNm = 0.0
						cmd.BrakePct = 100.0
						r.log.Info("All waypoints reached — holding stop at t=%.2f", t)
					} else {
						cmd.GearPosition = wpEval.Waypoint.GearPos
						r.applyWaypointPID(&cmd, wpEval.Waypoint, currentVelocity, currentYawDeg, dt, t)
					}
				}

				// case "open_loop" - use cmd as-is from scenario
			}

			// Encode and transmit CAN frame
			values := map[string]float64{
				"system_enable":       control.BoolToFloat(cmd.SystemEnable),
				"mode":                cmd.Mode,
				"steer_cmd_deg":       cmd.SteerDeg,
				"drive_torque_cmd_nm": cmd.TorqueNm,
				"brake_cmd_pct":       cmd.BrakePct,
				"gear_position":       float64(cmd.GearPosition),
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
			// Trace: log raw bytes every 100 frames (~1 s) so the wire encoding
			// can be verified against the simulator's expected byte layout.
			if sent%100 == 0 {
				r.log.Trace("TX t=%.3f id=0x%08X bytes=[%02X %02X %02X %02X %02X %02X %02X %02X] gear=%d torque=%.0f brake=%.1f steer=%.1f",
					t, uint32(frame.ID),
					frame.Data[0], frame.Data[1], frame.Data[2], frame.Data[3],
					frame.Data[4], frame.Data[5], frame.Data[6], frame.Data[7],
					cmd.GearPosition, cmd.TorqueNm, cmd.BrakePct, cmd.SteerDeg)
			}
		}
	}
}

// applyPID updates command with PID controller output
// UPDATED applyPID FUNCTION FOR runner.go
// Replace your existing applyPID function with this one

// applyPID updates command with PID controller output
func (r *Runner) applyPID(cmd *ActuatorCmd, velocity float64, dt float64, t float64, iter uint64, segmentTargetVel *float64) {
	// Update target velocity if segment specifies one
	if segmentTargetVel != nil {
		r.pid.SetTargetVelocity(*segmentTargetVel)
	}

	// The sim reports speed as a positive magnitude regardless of gear direction.
	// The gear command sets the physical drive direction; the PID controls magnitude only.
	output := r.pid.Update(velocity, dt)

	// Apply motor torque and brake commands
	cmd.TorqueNm = output.TorqueNm
	cmd.BrakePct = output.BrakePct

	// Log diagnostics periodically
	if iter%100 == 0 {
		diag := r.pid.GetDiagnostics()
		controlMode := "ACCEL"
		if output.IsBrake {
			controlMode = "BRAKE"
		}
		r.log.Debug("PID: v=%.2f err=%.3f torque=%.1f brake=%.1f P=%.1f I=%.1f [%s]",
			velocity, diag.Error, output.TorqueNm, output.BrakePct, diag.P, diag.I, controlMode)
	}

	// Write to CSV every cycle (unified format)
	if r.csvFile != nil {
		diag := r.pid.GetDiagnostics()

		// Calculate D term by subtracting P and I from total output
		// Note: For brake commands, output.TorqueNm is 0, so we need to reconstruct
		var dTermApprox float64
		if output.IsAccel {
			dTermApprox = output.TorqueNm - diag.P - diag.I
		} else {
			// When braking, reconstruct negative torque then subtract P and I
			brakeTorqueEquiv := -(output.BrakePct / 100.0) * 12536.0 // Approximate
			dTermApprox = brakeTorqueEquiv - diag.P - diag.I
		}

		fmt.Fprintf(r.csvFile, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.3f,%.2f,%.2f,%d\n",
			t,
			r.pid.GetTargetVelocity(),
			velocity,
			diag.Error,
			output.TorqueNm, // Now 0 when braking
			output.BrakePct, // Now includes actual brake percentage!
			diag.P,
			diag.I,
			dTermApprox,
			diag.Integral,
			0.0, // est_mass (PID doesn't estimate)
			0.0, // est_drag (PID doesn't estimate)
			1.0, // model_conf (PID has no model)
			cmd.SteerDeg,
			cmd.GearPosition,
		)
	}
}

// applyMPC updates command with MPC controller output
func (r *Runner) applyMPC(cmd *ActuatorCmd, velocity float64, dt float64, t float64, iter uint64) {
	output := r.mpc.Update(velocity, dt)

	cmd.TorqueNm = output.TorqueNm
	cmd.BrakePct = output.BrakePct

	// Log diagnostics periodically
	if iter%100 == 0 {
		diag := r.mpc.GetDiagnostics()
		error := r.mpc.GetTargetVelocity() - velocity

		r.log.Debug("MPC: v=%.2f err=%.3f torque=%.1f brake=%.1f mass=%.0f conf=%.2f %s",
			velocity, error, output.TorqueNm, output.BrakePct,
			diag.EstimatedMass, diag.ModelConfidence,
			control.GetControlModeStr(output))
	}

	// Write to CSV every cycle (unified format)
	if r.csvFile != nil {
		diag := r.mpc.GetDiagnostics()
		error := r.mpc.GetTargetVelocity() - velocity

		fmt.Fprintf(r.csvFile, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.3f,%.2f,%.2f,%d\n",
			t,
			r.mpc.GetTargetVelocity(),
			velocity,
			error,
			output.TorqueNm,
			output.BrakePct,
			0.0, // p_term (MPC doesn't have PID terms)
			0.0, // i_term
			0.0, // d_term
			0.0, // integral
			diag.EstimatedMass,
			diag.EstimatedDrag,
			diag.ModelConfidence,
			cmd.SteerDeg,
			cmd.GearPosition,
		)
	}
}

// applyAutoMPC updates command with Auto-MPC controller output
func (r *Runner) applyAutoMPC(cmd *ActuatorCmd, velocity float64, dt float64, t float64, iter uint64) {
	output := r.autoMPC.Update(velocity, dt)

	cmd.TorqueNm = output.TorqueNm
	cmd.BrakePct = output.BrakePct

	if iter%100 == 0 {
		diag := r.autoMPC.GetDiagnostics()
		error := r.autoMPC.GetTargetVelocity() - velocity

		r.log.Debug("AUTO: v=%.2f err=%.3f torque=%.0f brake=%.1f mass=%.0f conf=%.2f Kp=%.1f %s",
			velocity, error, output.TorqueNm, output.BrakePct,
			diag.EstimatedMass, diag.MassConfidence, diag.AdaptiveKp,
			control.GetControlModeStr(output))
	}

	if r.csvFile != nil {
		diag := r.autoMPC.GetDiagnostics()
		error := r.autoMPC.GetTargetVelocity() - velocity

		// Calculate PID-like terms for comparison
		pTerm := diag.AdaptiveKp * error
		iTerm := diag.AdaptiveKi * error * dt    // Approximate
		dTerm := output.TorqueNm - pTerm - iTerm // Residual

		fmt.Fprintf(r.csvFile, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.3f,%.2f,%.2f,%d\n",
			t,
			r.autoMPC.GetTargetVelocity(),
			velocity,
			error,
			output.TorqueNm,
			output.BrakePct,
			pTerm, // Adaptive P term
			iTerm, // Adaptive I term
			dTerm, // Adaptive D term
			0.0,   // integral (not directly available)
			diag.EstimatedMass,
			diag.EstimatedDrag,
			diag.MassConfidence,
			cmd.SteerDeg,
			cmd.GearPosition,
		)
	}
}

// applyWaypointPID computes steering via Pure Pursuit and longitudinal torque/brake via PID.
func (r *Runner) applyWaypointPID(cmd *ActuatorCmd, wp Waypoint, velocity, yawDeg, dt, t float64) {
	// --- Pure Pursuit lateral control ---
	const (
		lookaheadM = 10.0 // tunable: larger = smoother, slower response
		wheelbaseM = 5.5  // XCMG XDE360
		maxSteerDeg = 15.0
	)
	dx := wp.X - r.posX
	dy := wp.Y - r.posY
	yawRad := yawDeg * math.Pi / 180.0
	// Transform waypoint into vehicle frame
	localX :=  math.Cos(yawRad)*dx + math.Sin(yawRad)*dy
	localY := -math.Sin(yawRad)*dx + math.Cos(yawRad)*dy
	dist := math.Sqrt(dx*dx + dy*dy)
	ld := math.Max(dist, lookaheadM)
	curvature := 2.0 * localY / (ld * ld)
	steerRad := math.Atan(curvature * wheelbaseM)
	steerDeg := steerRad * 180.0 / math.Pi
	// Reverse: negate steer because vehicle heading convention flips
	if cmd.GearPosition == 2 {
		steerDeg = -steerDeg
	}
	cmd.SteerDeg = control.ClampFloat(steerDeg, -maxSteerDeg, maxSteerDeg)

	// --- PID longitudinal ---
	r.pid.SetTargetVelocity(wp.SpeedMPS)
	output := r.pid.Update(velocity, dt)
	cmd.TorqueNm = output.TorqueNm
	cmd.BrakePct = output.BrakePct

	// --- CSV logging ---
	if r.csvFile != nil {
		diag := r.pid.GetDiagnostics()
		var dTermApprox float64
		if output.IsAccel {
			dTermApprox = output.TorqueNm - diag.P - diag.I
		} else {
			dTermApprox = -(output.BrakePct/100.0)*12536.0 - diag.P - diag.I
		}
		crossTrackErr := localY
		headingErrDeg := math.Atan2(localY, localX) * 180.0 / math.Pi
		fmt.Fprintf(r.csvFile,
			"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.3f,%.2f,%.2f,%d,%d,%.3f,%.3f\n",
			t, wp.SpeedMPS, velocity, diag.Error,
			output.TorqueNm, output.BrakePct,
			diag.P, diag.I, dTermApprox, diag.Integral,
			0.0, 0.0, 1.0, // est_mass, est_drag, model_conf
			cmd.SteerDeg, cmd.GearPosition,
			r.wpIdx, crossTrackErr, headingErrDeg,
		)
	}
}

// SensorFeedback contains decoded sensor data from CAN RX
type SensorFeedback struct {
	VelocityMPS float64
	YawRateRPS  float64
	PosX        float64 // metres (from 0x330)
	PosY        float64 // metres (from 0x330)
	YawDeg      float64 // degrees (from 0x331)
	HasPosition bool    // true when 0x330/0x331 have been received at least once
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

			// Decode VEHICLE_STATE_1 for truth velocity
			if frame.ID == r.vehicleStateID {
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

			// Decode POSITION_STATE: pos_x_m @ bit0 32-bit, pos_y_m @ bit32 32-bit, factor=0.01
			if frame.ID == r.positionStateID {
				posX := r.decodeSignal(frame.Data[:], 0, 32, true, 0.01, 0.0)
				posY := r.decodeSignal(frame.Data[:], 32, 32, true, 0.01, 0.0)
				select {
				case feedback <- SensorFeedback{
					PosX:        posX,
					PosY:        posY,
					HasPosition: true,
					Timestamp:   time.Now(),
				}:
				default:
				}
			}

			// Decode ORIENTATION_STATE: yaw_deg @ bit0 16-bit signed, factor=0.1
			if frame.ID == r.orientStateID {
				yawDeg := r.decodeSignal(frame.Data[:], 0, 16, true, 0.1, 0.0)
				select {
				case feedback <- SensorFeedback{
					YawDeg:    yawDeg,
					Timestamp: time.Now(),
				}:
				default:
				}
			}
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

// compile-time assurance
var _ can.Frame
