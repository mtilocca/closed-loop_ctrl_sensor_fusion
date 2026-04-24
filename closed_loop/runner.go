package main

import (
	"context"
	"fmt"
	"math"
	"os"
	"time"

	control "closed_loop_ctrl_sensor_fusion/closed_loop/longitudinal_control"
	"closed_loop_ctrl_sensor_fusion/utils"
)

// SensorFeedback is an alias for the shared transport type.
type SensorFeedback = utils.SensorFeedback

type RunnerConfig struct {
	ScenarioPath  string
	TransportDesc string // e.g. "vcan0 (CAN)" or "tcp://host:1883 (MQTT)"
}

type Runner struct {
	cfg      RunnerConfig
	log      *utils.Logger
	scen     Scenario
	transport utils.Transport
	cycleMS  int

	// Controllers (only one active at a time)
	pid     *control.PIDController
	mpc     *control.MPCController
	autoMPC *control.AutoMPCController

	// Waypoint tracking (waypoint_pid mode)
	wpIdx     int
	posX      float64
	posY      float64
	posYawDeg float64

	// CSV logging
	csvFile *os.File
	csvPath string
}

func NewRunner(ctx context.Context, cfg RunnerConfig, transport utils.Transport, log *utils.Logger) (*Runner, error) {
	scen, err := LoadScenario(cfg.ScenarioPath)
	if err != nil {
		return nil, fmt.Errorf("load scenario: %w", err)
	}

	// Derive control cycle from scenario timing (dt_s = 0.01 → 10 ms).
	cycleMS := int(scen.Timing.DtS * 1000)
	if cycleMS <= 0 {
		cycleMS = 10
	}

	csvPath := generateCSVFilename(cfg.ScenarioPath, scen.Meta.ControlMode)

	r := &Runner{
		cfg:       cfg,
		log:       log,
		scen:      scen,
		transport: transport,
		cycleMS:   cycleMS,
		csvPath:   csvPath,
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

		csvFile, err := os.Create(r.csvPath)
		if err != nil {
			return nil, fmt.Errorf("create CSV log: %w", err)
		}
		r.csvFile = csvFile
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

		csvFile, err := os.Create(r.csvPath)
		if err != nil {
			return nil, fmt.Errorf("create CSV log: %w", err)
		}
		r.csvFile = csvFile
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

		csvFile, err := os.Create(r.csvPath)
		if err != nil {
			return nil, fmt.Errorf("create CSV log: %w", err)
		}
		r.csvFile = csvFile
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
	if r.transport != nil {
		_ = r.transport.Close()
	}
}

// sendShutdownCommand delegates the safe-stop sequence to the active transport.
func (r *Runner) sendShutdownCommand() {
	if err := r.transport.SendShutdown(context.Background()); err != nil {
		r.log.Error("Shutdown command failed: %v", err)
	}
}

func (r *Runner) Run(ctx context.Context) error {
	controlModeStr := r.scen.Meta.ControlMode
	if controlModeStr == "" {
		controlModeStr = "open_loop"
	}

	r.log.Info("Starting: transport=%s cycle_ms=%d scenario=%s duration=%.2fs mode=%s",
		r.cfg.TransportDesc, r.cycleMS, r.scen.Meta.Name, r.scen.Timing.DurationS, controlModeStr)

	start := time.Now()
	ticker := time.NewTicker(time.Duration(r.cycleMS) * time.Millisecond)
	defer ticker.Stop()

	endAfter := time.Duration(r.scen.Timing.DurationS * float64(time.Second))
	var sent uint64

	currentVelocity := 0.0
	lastRxTime := time.Now()
	currentX := 0.0
	currentY := 0.0
	currentYawDeg := 0.0

	currentGear := 1
	gearChangePending := false

	r.transport.Start(ctx)
	rxChan := r.transport.Feedback()

	for {
		select {
		case <-ctx.Done():
			r.log.Warn("Context canceled; stopping TX")
			r.sendShutdownCommand()
			r.log.Info("Completed TX. frames_sent=%d", sent)
			return ctx.Err()

		case feedback := <-rxChan:
			if feedback.HasVelocity {
				currentVelocity = feedback.VelocityMPS
				lastRxTime = time.Now()
			}
			if feedback.HasPosition {
				currentX = feedback.PosX
				currentY = feedback.PosY
				r.posX = feedback.PosX
				r.posY = feedback.PosY
			}
			if feedback.YawDeg != 0 {
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
			dt := float64(r.cycleMS) / 1000.0

			rxAge := now.Sub(lastRxTime)
			if rxAge > 500*time.Millisecond && (r.pid != nil || r.mpc != nil || r.autoMPC != nil) {
				r.log.Warn("No sensor feedback for %.1f ms - controller may be unreliable", rxAge.Seconds()*1000)
			}

			segEval := EvalSegment(&r.scen, t)
			cmd := segEval.Cmd

			// === GEAR-CHANGE SAFETY GUARD ===
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

			// case "open_loop" — use cmd as-is from scenario
			}

			if err := r.transport.SendActuator(ctx, utils.ActuatorValues{
				SystemEnable: cmd.SystemEnable,
				GearPosition: cmd.GearPosition,
				Mode:         cmd.Mode,
				TorqueNm:     cmd.TorqueNm,
				SteerDeg:     cmd.SteerDeg,
				BrakePct:     cmd.BrakePct,
			}); err != nil {
				r.log.Critical("Transmit failed at t=%.3f: %v", t, err)
				return err
			}
			sent++
		}
	}
}

// applyPID updates command with PID controller output
func (r *Runner) applyPID(cmd *ActuatorCmd, velocity float64, dt float64, t float64, iter uint64, segmentTargetVel *float64) {
	if segmentTargetVel != nil {
		r.pid.SetTargetVelocity(*segmentTargetVel)
	}

	output := r.pid.Update(velocity, dt)
	cmd.TorqueNm = output.TorqueNm
	cmd.BrakePct = output.BrakePct

	if iter%100 == 0 {
		diag := r.pid.GetDiagnostics()
		controlMode := "ACCEL"
		if output.IsBrake {
			controlMode = "BRAKE"
		}
		r.log.Debug("PID: v=%.2f err=%.3f torque=%.1f brake=%.1f P=%.1f I=%.1f [%s]",
			velocity, diag.Error, output.TorqueNm, output.BrakePct, diag.P, diag.I, controlMode)
	}

	if r.csvFile != nil {
		diag := r.pid.GetDiagnostics()
		var dTermApprox float64
		if output.IsAccel {
			dTermApprox = output.TorqueNm - diag.P - diag.I
		} else {
			brakeTorqueEquiv := -(output.BrakePct / 100.0) * 12536.0
			dTermApprox = brakeTorqueEquiv - diag.P - diag.I
		}
		fmt.Fprintf(r.csvFile, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.3f,%.2f,%.2f,%d\n",
			t,
			r.pid.GetTargetVelocity(),
			velocity,
			diag.Error,
			output.TorqueNm,
			output.BrakePct,
			diag.P,
			diag.I,
			dTermApprox,
			diag.Integral,
			0.0, // est_mass
			0.0, // est_drag
			1.0, // model_conf
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

	if iter%100 == 0 {
		diag := r.mpc.GetDiagnostics()
		velocityError := r.mpc.GetTargetVelocity() - velocity
		r.log.Debug("MPC: v=%.2f err=%.3f torque=%.1f brake=%.1f mass=%.0f conf=%.2f %s",
			velocity, velocityError, output.TorqueNm, output.BrakePct,
			diag.EstimatedMass, diag.ModelConfidence,
			control.GetControlModeStr(output))
	}

	if r.csvFile != nil {
		diag := r.mpc.GetDiagnostics()
		velocityError := r.mpc.GetTargetVelocity() - velocity
		fmt.Fprintf(r.csvFile, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.3f,%.2f,%.2f,%d\n",
			t,
			r.mpc.GetTargetVelocity(),
			velocity,
			velocityError,
			output.TorqueNm,
			output.BrakePct,
			0.0, // p_term
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
		velocityError := r.autoMPC.GetTargetVelocity() - velocity
		r.log.Debug("AUTO: v=%.2f err=%.3f torque=%.0f brake=%.1f mass=%.0f conf=%.2f Kp=%.1f %s",
			velocity, velocityError, output.TorqueNm, output.BrakePct,
			diag.EstimatedMass, diag.MassConfidence, diag.AdaptiveKp,
			control.GetControlModeStr(output))
	}

	if r.csvFile != nil {
		diag := r.autoMPC.GetDiagnostics()
		velocityError := r.autoMPC.GetTargetVelocity() - velocity
		pTerm := diag.AdaptiveKp * velocityError
		iTerm := diag.AdaptiveKi * velocityError * dt
		dTerm := output.TorqueNm - pTerm - iTerm

		fmt.Fprintf(r.csvFile, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.3f,%.2f,%.2f,%d\n",
			t,
			r.autoMPC.GetTargetVelocity(),
			velocity,
			velocityError,
			output.TorqueNm,
			output.BrakePct,
			pTerm,
			iTerm,
			dTerm,
			0.0, // integral
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
	const (
		lookaheadM  = 10.0
		wheelbaseM  = 5.5
		maxSteerDeg = 15.0
	)
	dx := wp.X - r.posX
	dy := wp.Y - r.posY
	yawRad := yawDeg * math.Pi / 180.0
	localX := math.Cos(yawRad)*dx + math.Sin(yawRad)*dy
	localY := -math.Sin(yawRad)*dx + math.Cos(yawRad)*dy
	dist := math.Sqrt(dx*dx + dy*dy)
	ld := math.Max(dist, lookaheadM)
	curvature := 2.0 * localY / (ld * ld)
	steerRad := math.Atan(curvature * wheelbaseM)
	steerDeg := steerRad * 180.0 / math.Pi
	if cmd.GearPosition == 2 {
		steerDeg = -steerDeg
	}
	cmd.SteerDeg = control.ClampFloat(steerDeg, -maxSteerDeg, maxSteerDeg)

	r.pid.SetTargetVelocity(wp.SpeedMPS)
	output := r.pid.Update(velocity, dt)
	cmd.TorqueNm = output.TorqueNm
	cmd.BrakePct = output.BrakePct

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
			0.0, 0.0, 1.0,
			cmd.SteerDeg, cmd.GearPosition,
			r.wpIdx, crossTrackErr, headingErrDeg,
		)
	}
}
