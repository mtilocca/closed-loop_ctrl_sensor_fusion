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
	fd     *utils.FrameDef
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

	// Einride transport
	writer, err := NewSocketCANWriter(ctx, cfg.Interface)
	if err != nil {
		return nil, err
	}

	return &Runner{
		cfg:    cfg,
		log:    log,
		cmap:   cmap,
		scen:   scen,
		writer: writer,
		fd:     fd,
	}, nil
}

func (r *Runner) Close() {
	if r.writer != nil {
		_ = r.writer.Close()
	}
}

func (r *Runner) Run(ctx context.Context) error {
	r.log.Info("Starting TX: frame=%s id=0x%X dlc=%d cycle_ms=%d iface=%s scenario=%s duration=%.2fs",
		r.fd.Name, r.fd.ID, r.fd.DLC, r.fd.CycleMS, r.cfg.Interface, r.scen.Meta.Name, r.scen.Timing.DurationS)

	start := time.Now()
	ticker := time.NewTicker(time.Duration(r.fd.CycleMS) * time.Millisecond)
	defer ticker.Stop()

	endAfter := time.Duration(r.scen.Timing.DurationS * float64(time.Second))
	var sent uint64

	for {
		select {
		case <-ctx.Done():
			r.log.Warn("Context canceled; stopping TX")
			r.log.Info("Completed TX. frames_sent=%d", sent)
			return ctx.Err()

		case now := <-ticker.C:
			elapsed := now.Sub(start)
			if elapsed > endAfter {
				r.log.Info("Completed TX. frames_sent=%d", sent)
				return nil
			}

			t := elapsed.Seconds()
			cmd := EvalActCmd(&r.scen, t)

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

// compile-time assurance the transmitted frame type is what we expect
var _ can.Frame
