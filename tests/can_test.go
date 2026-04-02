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
	fd, err := m.FrameByID(0x18EFF021) // J1939: priority=6, pgn=0xEF00(PDU1), sa=0x21, da=0xF0
	if err != nil {
		t.Fatalf("ACTUATOR_CMD_1 (0x18EFF021) not found: %v", err)
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
	fd, _ := m.FrameByID(0x18EFF021)

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
	fd, err := m.FrameByID(0x18FF50F0) // J1939: priority=6, pgn=0xFF50(PDU2), sa=0xF0
	if err != nil {
		t.Fatalf("VEHICLE_STATE_1 (0x18FF50F0) not found: %v", err)
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
	byID, _ := m.FrameByID(0x18EFF021)
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
	// CSV missing the "signed" column (uses J1939 columns, but omits "signed")
	content := "direction,priority,pgn,sa,da,frame_name,cycle_ms,dlc,signal_name,target,start_bit,bit_length,endianness,factor,offset,min,max,default,unit,counter_bits,crc,comment\n" +
		"rx,6,0xEF00,0x21,0xF0,TEST,10,8,speed,actuator_cmd,0,8,little,0.01,0,0,100,0,m/s,,,speed\n"
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
	fd, _ := m.FrameByID(0x18EFF021)
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

// ---------------------------------------------------------------------------
// Byte-level encoding verification for ACTUATOR_CMD_1
// These tests pin the exact wire bytes to catch any CAN map / codec regressions
// that would cause the simulator to receive incorrect commands.
//
// ACTUATOR_CMD_1 (0x100) layout (little-endian):
//   Byte 0 : bit0=system_enable, bits1-2=gear_position, bits3-4=mode
//   Bytes 1-2 : steer_cmd_deg   (signed 16-bit, factor=0.1)
//   Bytes 3-4 : drive_torque_cmd_nm (signed 16-bit, factor=10)
//   Byte 5   : brake_cmd_pct   (unsigned 8-bit, factor=1)
//   Bytes 6-7 : unused (zero)
// ---------------------------------------------------------------------------

// TestActuatorCMD1BrakeByteEncoding verifies that brake_cmd_pct=100 appears in
// byte 5 of the wire frame as the value 100 (0x64), and that drive torque is 0.
func TestActuatorCMD1BrakeByteEncoding(t *testing.T) {
	m, err := utils.LoadCANMap(realCANMapPath)
	if err != nil {
		t.Fatalf("LoadCANMap: %v", err)
	}

	payload, _, err := m.EncodeFrame("ACTUATOR_CMD_1", map[string]float64{
		"system_enable":       1.0,
		"gear_position":       1.0,
		"mode":                0.0,
		"steer_cmd_deg":       0.0,
		"drive_torque_cmd_nm": 0.0,
		"brake_cmd_pct":       100.0,
	})
	if err != nil {
		t.Fatalf("EncodeFrame: %v", err)
	}

	// Byte 0: system_enable=1 (bit0), gear=1 (bits1-2=01b), mode=0 (bits3-4=00b) → 0x03
	if payload[0] != 0x03 {
		t.Errorf("byte[0] = 0x%02X, want 0x03 (system_enable=1, gear=1, mode=0)", payload[0])
	}
	// Bytes 3-4: drive_torque=0 → raw=0 → both bytes 0x00
	if payload[3] != 0x00 || payload[4] != 0x00 {
		t.Errorf("bytes[3:5] = 0x%02X 0x%02X, want 0x00 0x00 (torque=0)", payload[3], payload[4])
	}
	// Byte 5: brake_cmd_pct=100 → raw=100 → 0x64
	if payload[5] != 0x64 {
		t.Errorf("byte[5] = 0x%02X, want 0x64 (brake_cmd_pct=100%%)", payload[5])
	}
}

// TestActuatorCMD1DriveTorqueByteEncoding verifies that a known motor torque
// command encodes to the expected little-endian bytes.
// drive_torque_cmd_nm=125000, factor=10 → raw=12500 (0x30D4) → byte3=0xD4, byte4=0x30.
func TestActuatorCMD1DriveTorqueByteEncoding(t *testing.T) {
	m, err := utils.LoadCANMap(realCANMapPath)
	if err != nil {
		t.Fatalf("LoadCANMap: %v", err)
	}

	payload, _, err := m.EncodeFrame("ACTUATOR_CMD_1", map[string]float64{
		"system_enable":       1.0,
		"gear_position":       1.0,
		"mode":                0.0,
		"steer_cmd_deg":       0.0,
		"drive_torque_cmd_nm": 125000.0, // raw = 125000/10 = 12500 = 0x30D4
		"brake_cmd_pct":       0.0,
	})
	if err != nil {
		t.Fatalf("EncodeFrame: %v", err)
	}

	// 12500 = 0x30D4; little-endian → byte3=0xD4, byte4=0x30
	if payload[3] != 0xD4 {
		t.Errorf("byte[3] = 0x%02X, want 0xD4 (torque low byte)", payload[3])
	}
	if payload[4] != 0x30 {
		t.Errorf("byte[4] = 0x%02X, want 0x30 (torque high byte)", payload[4])
	}
	// brake must be 0 when accelerating
	if payload[5] != 0x00 {
		t.Errorf("byte[5] = 0x%02X, want 0x00 (brake_cmd_pct=0 while accelerating)", payload[5])
	}
}

// TestActuatorCMD1SystemEnableMustBeSetForCommands checks that system_enable=1
// sets bit 0 of byte 0, and system_enable=0 clears it.
func TestActuatorCMD1SystemEnableBit(t *testing.T) {
	m, _ := utils.LoadCANMap(realCANMapPath)

	// With system_enable=1
	p1, _, _ := m.EncodeFrame("ACTUATOR_CMD_1", map[string]float64{"system_enable": 1.0})
	if p1[0]&0x01 != 0x01 {
		t.Errorf("system_enable=1: byte[0] bit0 = 0, want 1 (got 0x%02X)", p1[0])
	}

	// With system_enable=0 (all other signals default)
	p0, _, _ := m.EncodeFrame("ACTUATOR_CMD_1", map[string]float64{"system_enable": 0.0})
	if p0[0]&0x01 != 0x00 {
		t.Errorf("system_enable=0: byte[0] bit0 = 1, want 0 (got 0x%02X)", p0[0])
	}
}

// TestActuatorCMD1BrakeAndTorqueMutualExclusion verifies that when brake>0,
// torque can be independently set to 0 (the simulator contract).
func TestActuatorCMD1BrakeAndTorqueMutualExclusion(t *testing.T) {
	m, _ := utils.LoadCANMap(realCANMapPath)

	cases := []struct {
		torque float64
		brake  float64
	}{
		{125000, 0},   // full accel, no brake
		{0, 100},      // full brake, no torque
		{0, 50},       // half brake
		{0, 0},        // coast
	}

	for _, c := range cases {
		payload, _, err := m.EncodeFrame("ACTUATOR_CMD_1", map[string]float64{
			"system_enable":       1.0,
			"drive_torque_cmd_nm": c.torque,
			"brake_cmd_pct":       c.brake,
		})
		if err != nil {
			t.Fatalf("torque=%.0f brake=%.0f: EncodeFrame: %v", c.torque, c.brake, err)
		}

		decoded, _ := m.DecodeFrame(0x18EFF021, payload)
		if math.Abs(decoded["drive_torque_cmd_nm"]-c.torque) > 10.0 {
			t.Errorf("torque=%.0f: decoded=%.0f (diff>10)", c.torque, decoded["drive_torque_cmd_nm"])
		}
		if math.Abs(decoded["brake_cmd_pct"]-c.brake) > 0.5 {
			t.Errorf("brake=%.0f: decoded=%.1f", c.brake, decoded["brake_cmd_pct"])
		}
	}
}

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
