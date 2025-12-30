package main

import (
	"context"
	"flag"
	"fmt"
	"os"
	"os/signal"
	"path/filepath"
	"strings"
	"syscall"

	"dds-fusion-core/utils"
)

func main() {
	var (
		iface     = flag.String("iface", "vcan0", "SocketCAN interface name")
		mapPath   = flag.String("map", "config/can/can_map.csv", "Path to can_map.csv")
		scenPath  = flag.String("scenario", "closed_loop/scenarios/constant_velocity_turns.json", "Scenario JSON file")
		frameName = flag.String("frame", "ACTUATOR_CMD_1", "Frame name to transmit")
		logLevel  = flag.String("log", "info", "trace|debug|info|warn|error|critical")
	)
	flag.Parse()

	level := parseLevel(*logLevel)

	log, err := utils.NewFileLogger("closed_loop.log", level, true)
	if err != nil {
		_, _ = os.Stderr.WriteString("ERROR: cannot open closed_loop.log: " + err.Error() + "\n")
		os.Exit(1)
	}
	defer log.Close()

	cfg := RunnerConfig{
		Interface:    *iface,
		MapPath:      *mapPath,
		ScenarioPath: *scenPath,
		FrameName:    *frameName,
	}

	ctx, stop := signal.NotifyContext(context.Background(), syscall.SIGINT, syscall.SIGTERM)
	defer stop()

	runner, err := NewRunner(ctx, cfg, log)
	if err != nil {
		log.Critical("Startup failed: %v", err)
		os.Exit(1)
	}
	defer runner.Close()

	// Print scenario info
	log.Info("========================================")
	log.Info("Scenario: %s (v%d)", runner.scen.Meta.Name, runner.scen.Meta.Version)
	log.Info("Description: %s", runner.scen.Meta.Description)
	log.Info("Control Mode: %s", runner.scen.Meta.ControlMode)
	log.Info("CSV Output: %s", runner.csvPath)

	if runner.scen.Meta.ControlMode == "velocity_pid" && runner.scen.PIDConfig != nil {
		log.Info("PID Configuration:")
		log.Info("  Target Velocity: %.2f m/s", runner.scen.PIDConfig.TargetVelocityMPS)
		log.Info("  Kp: %.1f", runner.scen.PIDConfig.Kp)
		log.Info("  Ki: %.1f", runner.scen.PIDConfig.Ki)
		log.Info("  Kd: %.1f", runner.scen.PIDConfig.Kd)
		log.Info("  Torque Limits: [%.0f, %.0f] Nm",
			runner.scen.PIDConfig.MinTorqueNm,
			runner.scen.PIDConfig.MaxTorqueNm)
	} else if runner.scen.Meta.ControlMode == "velocity_mpc" && runner.scen.MPCConfig != nil {
		log.Info("MPC Configuration:")
		log.Info("  Target Velocity: %.2f m/s", runner.scen.MPCConfig.TargetVelocityMPS)
		log.Info("  Prediction Horizon: %d steps", runner.scen.MPCConfig.PredictionHorizon)
		log.Info("  Adaptation: %v", runner.scen.MPCConfig.EnableAdaptation)
	} else if runner.scen.Meta.ControlMode == "auto_mpc" && runner.scen.AutoMPCConfig != nil {
		log.Info("Auto-MPC Configuration:")
		log.Info("  Target Velocity: %.2f m/s", runner.scen.AutoMPCConfig.TargetVelocityMPS)
		log.Info("  Aggressive Tuning: %v", runner.scen.AutoMPCConfig.AggressiveTuning)
		log.Info("  Learning Rate: %.3f", runner.scen.AutoMPCConfig.LearningRate)
	}
	log.Info("========================================")

	if err := runner.Run(ctx); err != nil && err != context.Canceled {
		log.Critical("Run failed: %v", err)
		os.Exit(1)
	}

	log.Info("Shutdown complete")
	log.Info("Results saved to: %s", runner.csvPath)
}

func parseLevel(s string) utils.LogLevel {
	switch s {
	case "trace":
		return utils.TRACE
	case "debug":
		return utils.DEBUG
	case "info":
		return utils.INFO
	case "warn", "warning":
		return utils.WARN
	case "error":
		return utils.ERROR
	case "critical":
		return utils.CRITICAL
	default:
		return utils.INFO
	}
}

// generateCSVFilename creates a descriptive CSV filename from scenario path and control mode
func generateCSVFilename(scenarioPath string, controlMode string) string {
	// Extract scenario name from path
	// e.g., "closed_loop/scenarios/gentle_slalom_pid.json" -> "gentle_slalom_pid"
	basename := filepath.Base(scenarioPath)
	scenarioName := strings.TrimSuffix(basename, filepath.Ext(basename))

	// Create filename: <scenario_name>_<control_mode>.csv
	// e.g., "gentle_slalom_pid_velocity_pid.csv" or just "gentle_slalom_pid.csv"
	filename := fmt.Sprintf("%s.csv", scenarioName)

	return filename
}
