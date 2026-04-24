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

	"closed_loop_ctrl_sensor_fusion/utils"
)

func main() {
	var (
		// CAN flags
		iface     = flag.String("iface", "vcan0", "SocketCAN interface name (CAN transport only)")
		mapPath   = flag.String("map", "config/can/can_map.csv", "Path to can_map.csv (CAN transport only)")
		frameName = flag.String("frame", "ACTUATOR_CMD_1", "CAN frame to transmit (CAN transport only)")
		// Common flags
		scenPath      = flag.String("scenario", "closed_loop/scenarios/constant_velocity_turns.json", "Scenario JSON file")
		logLevel      = flag.String("log", "info", "trace|debug|info|warn|error|critical")
		transportMode = flag.String("transport", "can", "Transport mode: can|mqtt")
		mqttCfgPath   = flag.String("mqtt-config", "config/mqtt.yaml", "Path to mqtt.yaml (MQTT transport only)")
	)
	flag.Parse()

	level := parseLevel(*logLevel)

	log, err := utils.NewFileLogger("closed_loop.log", level, true)
	if err != nil {
		_, _ = os.Stderr.WriteString("ERROR: cannot open closed_loop.log: " + err.Error() + "\n")
		os.Exit(1)
	}
	defer log.Close()

	ctx, stop := signal.NotifyContext(context.Background(), syscall.SIGINT, syscall.SIGTERM)
	defer stop()

	var transport utils.Transport
	var transportDesc string

	switch *transportMode {
	case "can":
		cmap, err := utils.LoadCANMap(*mapPath)
		if err != nil {
			log.Critical("Load CAN map: %v", err)
			os.Exit(1)
		}
		fd, err := cmap.FrameByName(*frameName)
		if err != nil {
			log.Critical("CAN frame %q not found: %v", *frameName, err)
			os.Exit(1)
		}
		if fd.CycleMS <= 0 {
			log.Critical("Frame %s has invalid cycle_ms %d", fd.Name, fd.CycleMS)
			os.Exit(1)
		}
		vsFrame, err := cmap.FrameByName("VEHICLE_STATE_1")
		if err != nil {
			log.Critical("VEHICLE_STATE_1 not in CAN map: %v", err)
			os.Exit(1)
		}
		posFrame, err := cmap.FrameByName("POSITION_STATE")
		if err != nil {
			log.Critical("POSITION_STATE not in CAN map: %v", err)
			os.Exit(1)
		}
		orientFrame, err := cmap.FrameByName("ORIENTATION_STATE")
		if err != nil {
			log.Critical("ORIENTATION_STATE not in CAN map: %v", err)
			os.Exit(1)
		}
		transport, err = utils.NewCANTransport(ctx, *iface, cmap, fd,
			vsFrame.ID, posFrame.ID, orientFrame.ID, log)
		if err != nil {
			log.Critical("CAN transport init: %v", err)
			os.Exit(1)
		}
		transportDesc = fmt.Sprintf("%s (CAN)", *iface)

	case "mqtt":
		mqttCfg, err := utils.LoadMQTTConfig(*mqttCfgPath)
		if err != nil {
			log.Critical("Load MQTT config: %v", err)
			os.Exit(1)
		}
		transport, err = utils.NewMQTTTransport(mqttCfg, log)
		if err != nil {
			log.Critical("MQTT transport init: %v", err)
			os.Exit(1)
		}
		transportDesc = fmt.Sprintf("%s (MQTT)", mqttCfg.Broker)

	default:
		log.Critical("Unknown transport %q — use can or mqtt", *transportMode)
		os.Exit(1)
	}

	cfg := RunnerConfig{
		ScenarioPath:  *scenPath,
		TransportDesc: transportDesc,
	}

	runner, err := NewRunner(ctx, cfg, transport, log)
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

// generateCSVFilename creates a descriptive CSV filename from scenario path and control mode.
func generateCSVFilename(scenarioPath string, controlMode string) string {
	basename := filepath.Base(scenarioPath)
	scenarioName := strings.TrimSuffix(basename, filepath.Ext(basename))
	return fmt.Sprintf("%s.csv", scenarioName)
}
