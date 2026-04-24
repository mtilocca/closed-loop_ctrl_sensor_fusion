package utils

import (
	"fmt"
	"os"

	"gopkg.in/yaml.v3"
)

// MQTTConfig holds broker connection parameters loaded from mqtt.yaml.
type MQTTConfig struct {
	Broker        string `yaml:"broker"`
	ClientID      string `yaml:"client_id"`
	ActuatorTopic string `yaml:"actuator_topic"`
	StateTopic    string `yaml:"state_topic"`
	QOS           byte   `yaml:"qos"`
	KeepAliveS    int    `yaml:"keep_alive_s"`
	ConnTimeoutS  int    `yaml:"conn_timeout_s"`
}

// LoadMQTTConfig reads and parses a YAML file into MQTTConfig, applying defaults for missing fields.
func LoadMQTTConfig(path string) (MQTTConfig, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return MQTTConfig{}, fmt.Errorf("read mqtt config: %w", err)
	}
	var cfg MQTTConfig
	if err := yaml.Unmarshal(data, &cfg); err != nil {
		return MQTTConfig{}, fmt.Errorf("parse mqtt config: %w", err)
	}
	if cfg.ClientID == "" {
		cfg.ClientID = "closed_loop_ctrl"
	}
	if cfg.ActuatorTopic == "" {
		cfg.ActuatorTopic = "hdv/cmd/actuator"
	}
	if cfg.StateTopic == "" {
		cfg.StateTopic = "hdv/state/vehicle"
	}
	if cfg.KeepAliveS <= 0 {
		cfg.KeepAliveS = 30
	}
	if cfg.ConnTimeoutS <= 0 {
		cfg.ConnTimeoutS = 10
	}
	return cfg, nil
}
