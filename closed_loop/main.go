package main

import (
	"context"
	"flag"
	"os"
	"os/signal"
	"syscall"

	"dds-fusion-core/utils"
)

func main() {
	var (
		iface     = flag.String("iface", "vcan0", "SocketCAN interface name")
		mapPath   = flag.String("map", "config/can/can_map.csv", "Path to can_map.csv")
		scenPath  = flag.String("scenario", "closed_loop/slalom_aggressive_60s.json", "Scenario JSON file")
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

	if err := runner.Run(ctx); err != nil && err != context.Canceled {
		log.Critical("Run failed: %v", err)
		os.Exit(1)
	}
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
