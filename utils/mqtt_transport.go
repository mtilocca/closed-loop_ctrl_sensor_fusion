package utils

import (
	"context"
	"encoding/json"
	"fmt"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
)

// mqttActuatorMsg is the JSON payload published to the actuator topic.
type mqttActuatorMsg struct {
	Enable int     `json:"enable"`
	Gear   string  `json:"gear"`
	Torque float64 `json:"torque"`
	Steer  float64 `json:"steer"`
	Brake  float64 `json:"brake"`
}

// mqttStateMsg is the JSON payload received from the vehicle state topic.
type mqttStateMsg struct {
	SpeedMPS float64 `json:"speed_mps"`
	YawDeg   float64 `json:"yaw_deg"`
	XM       float64 `json:"x_m"`
	YM       float64 `json:"y_m"`
	// soc_pct, batt_v, motor_kw received but not used by the controller.
}

// MQTTTransport implements Transport over MQTT.
type MQTTTransport struct {
	cfg    MQTTConfig
	client mqtt.Client
	fbChan chan SensorFeedback
	log    *Logger
}

// Compile-time interface check.
var _ Transport = (*MQTTTransport)(nil)

// NewMQTTTransport creates and connects an MQTT client. Returns when the broker connection
// is established (or the first connect attempt times out).
func NewMQTTTransport(cfg MQTTConfig, log *Logger) (*MQTTTransport, error) {
	t := &MQTTTransport{
		cfg:    cfg,
		fbChan: make(chan SensorFeedback, 100),
		log:    log,
	}

	opts := mqtt.NewClientOptions().
		AddBroker(cfg.Broker).
		SetClientID(cfg.ClientID).
		SetKeepAlive(time.Duration(cfg.KeepAliveS) * time.Second).
		SetConnectTimeout(time.Duration(cfg.ConnTimeoutS) * time.Second).
		SetAutoReconnect(true).
		SetConnectRetry(true).
		SetConnectRetryInterval(2 * time.Second).
		SetCleanSession(true).
		SetOnConnectHandler(t.onConnect).
		SetConnectionLostHandler(t.onConnectionLost)

	t.client = mqtt.NewClient(opts)

	token := t.client.Connect()
	if !token.WaitTimeout(time.Duration(cfg.ConnTimeoutS) * time.Second) {
		return nil, fmt.Errorf("mqtt connect timeout (%ds) to %s", cfg.ConnTimeoutS, cfg.Broker)
	}
	if err := token.Error(); err != nil {
		return nil, fmt.Errorf("mqtt connect: %w", err)
	}

	log.Info("MQTT connected to %s (client_id=%s)", cfg.Broker, cfg.ClientID)
	return t, nil
}

// Start subscribes to the vehicle state topic. Call once before reading Feedback().
func (t *MQTTTransport) Start(ctx context.Context) {
	t.subscribe()
	// Unsubscribe and disconnect when context is done.
	go func() {
		<-ctx.Done()
		t.client.Unsubscribe(t.cfg.StateTopic)
	}()
}

// Feedback returns the channel delivering decoded vehicle state.
func (t *MQTTTransport) Feedback() <-chan SensorFeedback {
	return t.fbChan
}

// SendActuator publishes a JSON actuator command to the broker.
func (t *MQTTTransport) SendActuator(ctx context.Context, v ActuatorValues) error {
	msg := mqttActuatorMsg{
		Enable: boolToInt(v.SystemEnable),
		Gear:   gearToStr(v.GearPosition),
		Torque: v.TorqueNm,
		Steer:  v.SteerDeg,
		Brake:  v.BrakePct,
	}
	payload, err := json.Marshal(msg)
	if err != nil {
		return fmt.Errorf("mqtt marshal: %w", err)
	}
	token := t.client.Publish(t.cfg.ActuatorTopic, t.cfg.QOS, false, payload)
	token.Wait()
	return token.Error()
}

// SendShutdown publishes a single zero-command message to the broker.
func (t *MQTTTransport) SendShutdown(ctx context.Context) error {
	t.log.Info("MQTT: sending shutdown command...")
	return t.SendActuator(ctx, ActuatorValues{
		SystemEnable: false,
		GearPosition: 0,
		TorqueNm:     0,
		SteerDeg:     0,
		BrakePct:     0,
	})
}

// Close disconnects from the broker.
func (t *MQTTTransport) Close() error {
	t.client.Disconnect(250)
	return nil
}

// subscribe registers the message handler for the vehicle state topic.
func (t *MQTTTransport) subscribe() {
	token := t.client.Subscribe(t.cfg.StateTopic, t.cfg.QOS, t.onStateMessage)
	token.Wait()
	if err := token.Error(); err != nil {
		t.log.Error("MQTT subscribe failed: %v", err)
	} else {
		t.log.Info("MQTT subscribed to %s", t.cfg.StateTopic)
	}
}

// onConnect is called by paho on each successful connection or reconnection.
func (t *MQTTTransport) onConnect(c mqtt.Client) {
	t.log.Info("MQTT connected/reconnected to %s", t.cfg.Broker)
	// Re-subscribe after reconnect (required with CleanSession=true).
	t.subscribe()
}

// onConnectionLost is called by paho when the TCP connection drops.
func (t *MQTTTransport) onConnectionLost(c mqtt.Client, err error) {
	t.log.Warn("MQTT connection lost: %v — auto-reconnecting...", err)
}

// onStateMessage is the paho callback for incoming vehicle state messages.
func (t *MQTTTransport) onStateMessage(_ mqtt.Client, msg mqtt.Message) {
	var state mqttStateMsg
	if err := json.Unmarshal(msg.Payload(), &state); err != nil {
		t.log.Error("MQTT state decode error: %v", err)
		return
	}
	fb := SensorFeedback{
		VelocityMPS: state.SpeedMPS,
		YawDeg:      state.YawDeg,
		PosX:        state.XM,
		PosY:        state.YM,
		HasVelocity: true,
		HasPosition: true,
		Timestamp:   time.Now(),
	}
	select {
	case t.fbChan <- fb:
	default:
		// Channel full — drop the message.
	}
}

// gearToStr converts the internal gear integer to the MQTT string representation.
func gearToStr(gear int) string {
	switch gear {
	case 1:
		return "F"
	case 2:
		return "R"
	default:
		return "N"
	}
}

func boolToInt(b bool) int {
	if b {
		return 1
	}
	return 0
}
