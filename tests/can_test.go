// Package tests contains black-box integration tests for the CAN utilities.
// Tests run against the exported API of closed_loop_ctrl_sensor_fusion/utils.
// Internal bit-manipulation behaviour (getBits, setBits, signed encoding) is
// verified indirectly through EncodeFrame/DecodeFrame roundtrips below.
package tests

import (
	"math"
	"os"
	"strings"
	"testing"

	"closed_loop_ctrl_sensor_fusion/utils"
)

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// makeMinimalCANMap builds a small in-memory CANMap (no CSV) for codec tests.
func makeMinimalCANMap() *utils.CANMap {
	fd := &utils.FrameDef{
		ID:      0x200,
		Name:    "MOCK_FRAME",
		DLC:     8,
		CycleMS: 10,
		Signals: []utils.SignalDef{
			{
				Name: "speed_mps", StartBit: 0, BitLength: 8,
				Factor: 0.5, Min: 0, Max: 127.5, Default: 0,
			},
			{
				Name: "torque_nm", StartBit: 8, BitLength: 16,
				Signed: true, Factor: 10.0, Min: -327680, Max: 327670, Default: 0,
			},
			{
				Name: "gear", StartBit: 24, BitLength: 2,
				Factor: 1, Min: 0, Max: 3, Default: 1,
			},
		},
	}
	return &utils.CANMap{
		ByID:   map[uint32]*utils.FrameDef{0x200: fd},
		ByName: map[string]*utils.FrameDef{"MOCK_FRAME": fd},
	}
}

const realCANMapPath = "../config/can/can_map.csv"

// ---------------------------------------------------------------------------
// CAN codec: EncodeFrame / DecodeFrame / EncodeEinrideFrame
// ---------------------------------------------------------------------------

func TestEncodeDecodeRoundtrip(t *testing.T) {
	m := makeMinimalCANMap()
	values := map[string]float64{
		"speed_mps": 50.0,    // raw=100
		"torque_nm": -1000.0, // raw=-100
		"gear":      2,
	}

	payload, id, err := m.EncodeFrame("MOCK_FRAME", values)
	if err != nil {
		t.Fatalf("EncodeFrame: %v", err)
	}
	if id != 0x200 {
		t.Errorf("frame ID = 0x%X, want 0x200", id)
	}

	decoded, err := m.DecodeFrame(0x200, payload)
	if err != nil {
		t.Fatalf("DecodeFrame: %v", err)
	}
	if math.Abs(decoded["speed_mps"]-50.0) > 0.5 {
		t.Errorf("speed_mps = %.2f, want ~50.0", decoded["speed_mps"])
	}
	if math.Abs(decoded["torque_nm"]-(-1000.0)) > 10.0 {
		t.Errorf("torque_nm = %.1f, want ~-1000.0", decoded["torque_nm"])
	}
	if decoded["gear"] != 2.0 {
		t.Errorf("gear = %.0f, want 2", decoded["gear"])
	}
}

func TestSignedNegativeRoundtrip(t *testing.T) {
	// Tests that signed 16-bit two's-complement encoding is correct.
	m := makeMinimalCANMap()
	for _, v := range []float64{-10.0, -100.0, -327680.0, 0.0, 327670.0} {
		payload, _, _ := m.EncodeFrame("MOCK_FRAME", map[string]float64{"torque_nm": v})
		decoded, _ := m.DecodeFrame(0x200, payload)
		if math.Abs(decoded["torque_nm"]-v) > 10.0 {
			t.Errorf("torque_nm roundtrip %.1f: got %.1f", v, decoded["torque_nm"])
		}
	}
}

func TestEncodeUsesDefaultForMissingSignal(t *testing.T) {
	m := makeMinimalCANMap()
	payload, _, _ := m.EncodeFrame("MOCK_FRAME", map[string]float64{"speed_mps": 10.0})
	decoded, _ := m.DecodeFrame(0x200, payload)
	// torque default=0, gear default=1
	if decoded["torque_nm"] != 0 {
		t.Errorf("omitted torque_nm should default to 0, got %.1f", decoded["torque_nm"])
	}
	if decoded["gear"] != 1 {
		t.Errorf("omitted gear should default to 1, got %.0f", decoded["gear"])
	}
}

func TestEncodeClampsBeyondRange(t *testing.T) {
	m := makeMinimalCANMap()
	payload, _, _ := m.EncodeFrame("MOCK_FRAME", map[string]float64{"speed_mps": 9999.0})
	decoded, _ := m.DecodeFrame(0x200, payload)
	if decoded["speed_mps"] > 127.5+0.5 {
		t.Errorf("speed_mps should clamp to max 127.5, got %.2f", decoded["speed_mps"])
	}
}

func TestEncodeZeroValues(t *testing.T) {
	m := makeMinimalCANMap()
	values := map[string]float64{"speed_mps": 0, "torque_nm": 0, "gear": 0}
	payload, _, _ := m.EncodeFrame("MOCK_FRAME", values)
	decoded, _ := m.DecodeFrame(0x200, payload)
	if decoded["speed_mps"] != 0 || decoded["torque_nm"] != 0 || decoded["gear"] != 0 {
		t.Errorf("zero values roundtrip failed: speed=%.1f torque=%.1f gear=%.0f",
			decoded["speed_mps"], decoded["torque_nm"], decoded["gear"])
	}
}

func TestEncodeUnknownFrameReturnsError(t *testing.T) {
	m := makeMinimalCANMap()
	_, _, err := m.EncodeFrame("NONEXISTENT", nil)
	if err == nil {
		t.Error("expected error for unknown frame name")
	}
}

func TestDecodeUnknownFrameIDReturnsError(t *testing.T) {
	m := makeMinimalCANMap()
	_, err := m.DecodeFrame(0xDEAD, make([]byte, 8))
	if err == nil {
		t.Error("expected error for unknown frame ID")
	}
}

func TestDecodeShortPayloadReturnsError(t *testing.T) {
	m := makeMinimalCANMap()
	_, err := m.DecodeFrame(0x200, []byte{0x01, 0x02}) // DLC=8 but only 2 bytes
	if err == nil {
		t.Error("expected error when payload is shorter than DLC")
	}
}

func TestEncodeEinrideFrameIDAndLength(t *testing.T) {
	m := makeMinimalCANMap()
	f, err := m.EncodeEinrideFrame("MOCK_FRAME", map[string]float64{"speed_mps": 20.0})
	if err != nil {
		t.Fatalf("EncodeEinrideFrame: %v", err)
	}
	if f.ID != 0x200 {
		t.Errorf("einride frame ID = 0x%X, want 0x200", f.ID)
	}
	if int(f.Length) != 8 {
		t.Errorf("einride frame length = %d, want 8", f.Length)
	}
}

// ---------------------------------------------------------------------------
// CAN map loader: LoadCANMap, FrameByName, FrameByID
// ---------------------------------------------------------------------------

func TestLoadCANMapRealFile(t *testing.T) {
	m, err := utils.LoadCANMap(realCANMapPath)
	if err != nil {
		t.Fatalf("LoadCANMap(%q): %v", realCANMapPath, err)
	}
	if len(m.ByID) == 0 {
		t.Error("expected at least one frame in CAN map")
	}
}

func TestLoadCANMapACTUATOR_CMD_1(t *testing.T) {
	m, _ := utils.LoadCANMap(realCANMapPath)
	fd, err := m.FrameByID(0x100)
	if err != nil {
		t.Fatalf("ACTUATOR_CMD_1 (0x100) not found: %v", err)
	}
	if fd.Name != "ACTUATOR_CMD_1" {
		t.Errorf("name = %q, want ACTUATOR_CMD_1", fd.Name)
	}
	if fd.DLC != 8 {
		t.Errorf("DLC = %d, want 8", fd.DLC)
	}
}

func TestLoadCANMapGearPositionSignal(t *testing.T) {
	m, _ := utils.LoadCANMap(realCANMapPath)
	fd, _ := m.FrameByID(0x100)

	var gear *utils.SignalDef
	for i := range fd.Signals {
		if fd.Signals[i].Name == "gear_position" {
			gear = &fd.Signals[i]
			break
		}
	}
	if gear == nil {
		t.Fatal("gear_position signal not found in ACTUATOR_CMD_1")
	}
	if gear.BitLength != 2 {
		t.Errorf("gear_position bit_length = %d, want 2", gear.BitLength)
	}
	if gear.Signed {
		t.Error("gear_position should be unsigned")
	}
	if gear.Default != 1 {
		t.Errorf("gear_position default = %.0f, want 1 (Forward)", gear.Default)
	}
}

func TestLoadCANMapVehicleSpeedSignal(t *testing.T) {
	m, _ := utils.LoadCANMap(realCANMapPath)
	fd, err := m.FrameByID(0x300)
	if err != nil {
		t.Fatalf("VEHICLE_STATE_1 (0x300) not found: %v", err)
	}

	var speed *utils.SignalDef
	for i := range fd.Signals {
		if fd.Signals[i].Name == "vehicle_speed_mps" {
			speed = &fd.Signals[i]
			break
		}
	}
	if speed == nil {
		t.Fatal("vehicle_speed_mps not found in VEHICLE_STATE_1")
	}
	if !speed.Signed {
		t.Error("vehicle_speed_mps should be signed (supports reverse velocity)")
	}
}

func TestFrameByNameMatchesFrameByID(t *testing.T) {
	m, _ := utils.LoadCANMap(realCANMapPath)
	byID, _ := m.FrameByID(0x100)
	byName, err := m.FrameByName("ACTUATOR_CMD_1")
	if err != nil {
		t.Fatalf("FrameByName: %v", err)
	}
	if byID != byName {
		t.Error("FrameByID and FrameByName should return the same pointer")
	}
}

func TestFrameByNameUnknownReturnsError(t *testing.T) {
	m, _ := utils.LoadCANMap(realCANMapPath)
	_, err := m.FrameByName("DOES_NOT_EXIST")
	if err == nil {
		t.Error("expected error for unknown frame name")
	}
}

func TestFrameByIDUnknownReturnsError(t *testing.T) {
	m, _ := utils.LoadCANMap(realCANMapPath)
	_, err := m.FrameByID(0xDEADBEEF)
	if err == nil {
		t.Error("expected error for unknown frame ID")
	}
}

func TestLoadCANMapMissingFileReturnsError(t *testing.T) {
	_, err := utils.LoadCANMap("/no/such/path/can_map.csv")
	if err == nil {
		t.Error("expected error for non-existent file")
	}
}

func TestLoadCANMapMissingRequiredColumnReturnsError(t *testing.T) {
	// CSV missing the "signed" column
	content := "direction,frame_id,frame_name,cycle_ms,dlc,signal_name,start_bit,bit_length,factor,offset,min,max,default,unit,comment\n" +
		"rx,0x100,TEST,10,8,speed,0,8,0.01,0,0,100,0,,speed\n"
	f, err := os.CreateTemp("", "can_map_*.csv")
	if err != nil {
		t.Fatal(err)
	}
	defer os.Remove(f.Name())
	f.WriteString(content)
	f.Close()

	_, err = utils.LoadCANMap(f.Name())
	if err == nil {
		t.Error("expected error for CSV missing required column")
	}
	if !strings.Contains(err.Error(), "missing required column") {
		t.Errorf("error should mention missing column: %v", err)
	}
}

func TestLoadCANMapSignalsAreSortedByStartBit(t *testing.T) {
	m, _ := utils.LoadCANMap(realCANMapPath)
	fd, _ := m.FrameByID(0x100)
	for i := 1; i < len(fd.Signals); i++ {
		if fd.Signals[i].StartBit < fd.Signals[i-1].StartBit {
			t.Errorf("signals not sorted at index %d: %d < %d",
				i, fd.Signals[i].StartBit, fd.Signals[i-1].StartBit)
		}
	}
}

// ---------------------------------------------------------------------------
// Smoke: encode the real ACTUATOR_CMD_1 frame with all signals
// ---------------------------------------------------------------------------

func TestEncodeActuatorCMD1AllSignals(t *testing.T) {
	m, err := utils.LoadCANMap(realCANMapPath)
	if err != nil {
		t.Fatalf("LoadCANMap: %v", err)
	}

	values := map[string]float64{
		"system_enable":       1.0,
		"gear_position":       2.0, // Reverse
		"mode":                0.0,
		"steer_cmd_deg":       15.0,
		"drive_torque_cmd_nm": 50000.0,
		"brake_cmd_pct":       0.0,
	}

	f, err := m.EncodeEinrideFrame("ACTUATOR_CMD_1", values)
	if err != nil {
		t.Fatalf("EncodeEinrideFrame ACTUATOR_CMD_1: %v", err)
	}

	decoded, err := m.DecodeFrame(f.ID, f.Data[:f.Length])
	if err != nil {
		t.Fatalf("DecodeFrame ACTUATOR_CMD_1: %v", err)
	}

	if decoded["gear_position"] != 2.0 {
		t.Errorf("gear_position = %.0f, want 2", decoded["gear_position"])
	}
	if math.Abs(decoded["steer_cmd_deg"]-15.0) > 0.2 {
		t.Errorf("steer_cmd_deg = %.2f, want ~15.0", decoded["steer_cmd_deg"])
	}
	if decoded["system_enable"] != 1.0 {
		t.Errorf("system_enable = %.0f, want 1", decoded["system_enable"])
	}
}
