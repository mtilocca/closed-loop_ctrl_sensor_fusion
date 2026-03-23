// Package tests contains black-box tests for the scenario package.
package tests

import (
	"encoding/json"
	"os"
	"testing"

	scenario "closed_loop_ctrl_sensor_fusion/closed_loop/scenario"
)

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

func writeTempScenario(t *testing.T, v interface{}) string {
	t.Helper()
	b, err := json.Marshal(v)
	if err != nil {
		t.Fatalf("marshal scenario: %v", err)
	}
	f, err := os.CreateTemp("", "scenario_*.json")
	if err != nil {
		t.Fatalf("create temp file: %v", err)
	}
	if _, err := f.Write(b); err != nil {
		t.Fatalf("write temp file: %v", err)
	}
	f.Close()
	t.Cleanup(func() { os.Remove(f.Name()) })
	return f.Name()
}

// minimalValidScenario returns the minimum valid velocity_pid scenario map.
func minimalValidScenario(defaults map[string]interface{}, segments []interface{}) map[string]interface{} {
	if segments == nil {
		segments = []interface{}{}
	}
	return map[string]interface{}{
		"meta": map[string]interface{}{
			"name":         "test",
			"version":      1,
			"control_mode": "velocity_pid",
		},
		"timing": map[string]interface{}{
			"dt_s":       0.01,
			"duration_s": 10.0,
			"log_hz":     10.0,
		},
		"pid_config": map[string]interface{}{
			"target_velocity_mps": 5.0,
			"kp":                  1000.0, "ki": 50.0, "kd": 100.0,
			"max_torque_nm": 10000.0, "min_torque_nm": -5000.0,
			"integral_limit": 1000.0,
		},
		"defaults": defaults,
		"segments": segments,
	}
}

// --- LoadScenario tests ---

func TestLoadScenarioValid(t *testing.T) {
	path := writeTempScenario(t, minimalValidScenario(
		map[string]interface{}{"system_enable": true, "mode": 0},
		[]interface{}{
			map[string]interface{}{"t0": 0.0, "t1": 10.0, "steer_cmd_deg": 5.0},
		},
	))
	scen, err := scenario.LoadScenario(path)
	if err != nil {
		t.Fatalf("LoadScenario: %v", err)
	}
	if scen.Meta.ControlMode != "velocity_pid" {
		t.Errorf("control_mode = %q, want velocity_pid", scen.Meta.ControlMode)
	}
	if scen.Timing.DurationS != 10.0 {
		t.Errorf("duration = %.1f, want 10.0", scen.Timing.DurationS)
	}
	if len(scen.Segments) != 1 {
		t.Errorf("segments = %d, want 1", len(scen.Segments))
	}
}

func TestLoadScenarioMissingFileReturnsError(t *testing.T) {
	_, err := scenario.LoadScenario("/no/such/file.json")
	if err == nil {
		t.Error("expected error for non-existent file")
	}
}

func TestLoadScenarioZeroDurationReturnsError(t *testing.T) {
	s := minimalValidScenario(map[string]interface{}{}, nil)
	s["timing"] = map[string]interface{}{"dt_s": 0.01, "duration_s": 0.0}
	path := writeTempScenario(t, s)
	_, err := scenario.LoadScenario(path)
	if err == nil {
		t.Error("expected error for duration_s=0")
	}
}

func TestLoadScenarioMissingPIDConfigReturnsError(t *testing.T) {
	s := minimalValidScenario(map[string]interface{}{}, nil)
	delete(s, "pid_config")
	path := writeTempScenario(t, s)
	_, err := scenario.LoadScenario(path)
	if err == nil {
		t.Error("expected error for velocity_pid mode without pid_config")
	}
}

func TestLoadScenarioInvalidJSONReturnsError(t *testing.T) {
	f, _ := os.CreateTemp("", "scenario_*.json")
	f.WriteString("{invalid json{{")
	f.Close()
	t.Cleanup(func() { os.Remove(f.Name()) })
	_, err := scenario.LoadScenario(f.Name())
	if err == nil {
		t.Error("expected error for invalid JSON")
	}
}

func TestLoadScenarioDefaultsToOpenLoop(t *testing.T) {
	s := map[string]interface{}{
		"meta":     map[string]interface{}{"name": "x", "version": 1},
		"timing":   map[string]interface{}{"dt_s": 0.01, "duration_s": 5.0},
		"defaults": map[string]interface{}{},
		"segments": []interface{}{},
	}
	path := writeTempScenario(t, s)
	scen, err := scenario.LoadScenario(path)
	if err != nil {
		t.Fatalf("LoadScenario: %v", err)
	}
	if scen.Meta.ControlMode != "open_loop" {
		t.Errorf("missing control_mode should default to open_loop, got %q", scen.Meta.ControlMode)
	}
}

// --- Gear backward-compat tests ---

func TestGearDefaultsToForwardWhenOmitted(t *testing.T) {
	path := writeTempScenario(t, minimalValidScenario(
		map[string]interface{}{"system_enable": true},
		nil,
	))
	scen, err := scenario.LoadScenario(path)
	if err != nil {
		t.Fatalf("LoadScenario: %v", err)
	}
	if scen.Defaults.GearPosition != 1 {
		t.Errorf("GearPosition should default to 1 (Forward), got %d", scen.Defaults.GearPosition)
	}
}

func TestGearZeroInDefaultsBecomesForward(t *testing.T) {
	path := writeTempScenario(t, minimalValidScenario(
		map[string]interface{}{"gear_position": 0},
		nil,
	))
	scen, _ := scenario.LoadScenario(path)
	if scen.Defaults.GearPosition != 1 {
		t.Errorf("gear_position=0 in defaults should become 1 (Forward), got %d", scen.Defaults.GearPosition)
	}
}

func TestGearExplicitForwardPreserved(t *testing.T) {
	path := writeTempScenario(t, minimalValidScenario(
		map[string]interface{}{"gear_position": 1},
		nil,
	))
	scen, _ := scenario.LoadScenario(path)
	if scen.Defaults.GearPosition != 1 {
		t.Errorf("explicit gear=1 should stay 1, got %d", scen.Defaults.GearPosition)
	}
}

func TestGearExplicitReversePreserved(t *testing.T) {
	path := writeTempScenario(t, minimalValidScenario(
		map[string]interface{}{"gear_position": 2},
		nil,
	))
	scen, _ := scenario.LoadScenario(path)
	if scen.Defaults.GearPosition != 2 {
		t.Errorf("explicit gear=2 should stay 2, got %d", scen.Defaults.GearPosition)
	}
}

// --- EvalSegment tests ---

func TestEvalSegmentReturnsFirstSegment(t *testing.T) {
	gear2 := 2
	v55 := 5.5
	path := writeTempScenario(t, minimalValidScenario(
		map[string]interface{}{"system_enable": true, "gear_position": 1},
		[]interface{}{
			map[string]interface{}{
				"t0": 0.0, "t1": 5.0,
				"steer_cmd_deg":       15.0,
				"target_velocity_mps": v55,
			},
			map[string]interface{}{
				"t0": 5.0, "t1": 10.0,
				"steer_cmd_deg":       -15.0,
				"gear_position":       gear2,
				"target_velocity_mps": 2.5,
			},
		},
	))
	scen, err := scenario.LoadScenario(path)
	if err != nil {
		t.Fatalf("LoadScenario: %v", err)
	}

	// t=2.0 → segment 0
	eval := scenario.EvalSegment(&scen, 2.0)
	if eval.Cmd.SteerDeg != 15.0 {
		t.Errorf("t=2.0 steer = %.1f, want 15.0", eval.Cmd.SteerDeg)
	}
	if eval.Cmd.GearPosition != 1 {
		t.Errorf("t=2.0 gear = %d, want 1 (Forward, from defaults)", eval.Cmd.GearPosition)
	}
	if eval.TargetVelocityMPS == nil || *eval.TargetVelocityMPS != v55 {
		t.Errorf("t=2.0 target velocity should be %.1f", v55)
	}
}

func TestEvalSegmentReturnsSecondSegment(t *testing.T) {
	gear2 := 2
	path := writeTempScenario(t, minimalValidScenario(
		map[string]interface{}{"system_enable": true, "gear_position": 1},
		[]interface{}{
			map[string]interface{}{"t0": 0.0, "t1": 5.0, "steer_cmd_deg": 0.0},
			map[string]interface{}{
				"t0": 5.0, "t1": 10.0,
				"steer_cmd_deg":       -15.0,
				"gear_position":       gear2,
				"target_velocity_mps": 2.5,
			},
		},
	))
	scen, _ := scenario.LoadScenario(path)

	eval := scenario.EvalSegment(&scen, 7.0)
	if eval.Cmd.GearPosition != gear2 {
		t.Errorf("t=7.0 gear = %d, want 2 (Reverse)", eval.Cmd.GearPosition)
	}
	if eval.Cmd.SteerDeg != -15.0 {
		t.Errorf("t=7.0 steer = %.1f, want -15.0", eval.Cmd.SteerDeg)
	}
	if eval.TargetVelocityMPS == nil || *eval.TargetVelocityMPS != 2.5 {
		t.Error("t=7.0 target velocity should be 2.5")
	}
}

func TestEvalSegmentBeyondAllSegmentsUsesDefaults(t *testing.T) {
	path := writeTempScenario(t, minimalValidScenario(
		map[string]interface{}{"system_enable": true, "gear_position": 1},
		[]interface{}{
			map[string]interface{}{"t0": 0.0, "t1": 5.0, "steer_cmd_deg": 10.0},
		},
	))
	scen, _ := scenario.LoadScenario(path)

	eval := scenario.EvalSegment(&scen, 9.0) // after all segments
	if eval.TargetVelocityMPS != nil {
		t.Error("beyond all segments: TargetVelocityMPS should be nil")
	}
	if eval.Cmd.GearPosition != 1 {
		t.Errorf("beyond all segments: gear should be defaults (1), got %d", eval.Cmd.GearPosition)
	}
}

func TestEvalSegmentT1NegativeMeansEndOfScenario(t *testing.T) {
	path := writeTempScenario(t, minimalValidScenario(
		map[string]interface{}{"gear_position": 1},
		[]interface{}{
			map[string]interface{}{
				"t0": 0.0, "t1": -1, // -1 means run to end
				"steer_cmd_deg": 5.0,
			},
		},
	))
	scen, _ := scenario.LoadScenario(path)

	// Should still be active near the end
	eval := scenario.EvalSegment(&scen, 9.5)
	if eval.Cmd.SteerDeg != 5.0 {
		t.Errorf("t=9.5 with t1=-1 should still be active, steer=%.1f", eval.Cmd.SteerDeg)
	}
}

// --- Real scenario smoke tests ---

const realScenariosDir = "../closed_loop/scenarios/"

func TestLoadRealFullCruiseScenario(t *testing.T) {
	scen, err := scenario.LoadScenario(realScenariosDir + "full_cruise_maneuver_pid.json")
	if err != nil {
		t.Fatalf("LoadScenario full_cruise_maneuver: %v", err)
	}
	if scen.Meta.ControlMode != "velocity_pid" {
		t.Errorf("control_mode = %q, want velocity_pid", scen.Meta.ControlMode)
	}
	if scen.Timing.DurationS != 180.0 {
		t.Errorf("duration = %.1f, want 180.0", scen.Timing.DurationS)
	}
	if scen.PIDConfig == nil {
		t.Fatal("pid_config must not be nil")
	}

	// Must contain at least one Reverse segment (gear=2)
	hasReverse := false
	for _, seg := range scen.Segments {
		if seg.GearPosition != nil && *seg.GearPosition == 2 {
			hasReverse = true
			break
		}
	}
	if !hasReverse {
		t.Error("full_cruise_maneuver must contain at least one Reverse segment (gear=2)")
	}
}

func TestLoadRealFullCruiseGearSequence(t *testing.T) {
	scen, _ := scenario.LoadScenario(realScenariosDir + "full_cruise_maneuver_pid.json")

	// Phase 1 (t=5): gear=1 Forward, straight
	eval := scenario.EvalSegment(&scen, 5.0)
	if eval.Cmd.GearPosition != 1 {
		t.Errorf("t=5s: expected Forward gear (1), got %d", eval.Cmd.GearPosition)
	}

	// Phase 5 (t=80): gear=2 Reverse
	eval = scenario.EvalSegment(&scen, 80.0)
	if eval.Cmd.GearPosition != 2 {
		t.Errorf("t=80s: expected Reverse gear (2), got %d", eval.Cmd.GearPosition)
	}

	// Phase 8 (t=135): gear=1 Forward again
	eval = scenario.EvalSegment(&scen, 135.0)
	if eval.Cmd.GearPosition != 1 {
		t.Errorf("t=135s: expected Forward gear (1), got %d", eval.Cmd.GearPosition)
	}
}

func TestLoadRealCuspingScenario(t *testing.T) {
	// The existing cusping scenario should still load without errors (backward compat)
	scen, err := scenario.LoadScenario(realScenariosDir + "cusping_maneuver_pid.json")
	if err != nil {
		t.Fatalf("LoadScenario cusping_maneuver: %v", err)
	}
	// Should default gear to 1 (Forward) since it has no gear_position field
	if scen.Defaults.GearPosition != 1 {
		t.Errorf("cusping scenario (old JSON) gear default = %d, want 1", scen.Defaults.GearPosition)
	}
}

func TestLoadRealGentleSlalomScenario(t *testing.T) {
	scen, err := scenario.LoadScenario(realScenariosDir + "gentle_slalom_pid.json")
	if err != nil {
		t.Fatalf("LoadScenario gentle_slalom_pid: %v", err)
	}
	if len(scen.Segments) == 0 {
		t.Error("gentle_slalom_pid should have at least one segment")
	}
}

// ---------------------------------------------------------------------------
// EvalWaypoint tests
// ---------------------------------------------------------------------------

// TestEvalWaypointAdvances verifies that the waypoint index advances when the
// vehicle reaches the arrival radius of each waypoint.
func TestEvalWaypointAdvances(t *testing.T) {
	scen := scenario.Scenario{
		Meta: scenario.ScenarioMeta{
			Name:        "wp_test",
			Version:     1,
			ControlMode: "waypoint_pid",
		},
		Timing: scenario.ScenarioTiming{DtS: 0.01, DurationS: 60.0},
		Waypoints: []scenario.Waypoint{
			{X: 0.0, Y: 0.0, SpeedMPS: 5.5, GearPos: 1, ArriveR: 2.0},
			{X: 10.0, Y: 0.0, SpeedMPS: 5.5, GearPos: 1, ArriveR: 2.0},
			{X: 20.0, Y: 0.0, SpeedMPS: 2.5, GearPos: 1, ArriveR: 2.0},
		},
	}

	idx := 0

	// Far from first waypoint — should stay at index 0
	eval := scenario.EvalWaypoint(&scen, 50.0, 50.0, &idx)
	if idx != 0 {
		t.Errorf("far from wp0: idx = %d, want 0", idx)
	}
	if eval.Done {
		t.Error("should not be Done when far from all waypoints")
	}

	// Within arrival radius of wp0 → should advance to idx 1
	eval = scenario.EvalWaypoint(&scen, 0.5, 0.5, &idx)
	if idx != 1 {
		t.Errorf("inside wp0 radius: idx = %d, want 1", idx)
	}
	if eval.Waypoint.X != 10.0 {
		t.Errorf("after wp0 advance: waypoint.X = %.1f, want 10.0", eval.Waypoint.X)
	}

	// Within arrival radius of wp1 → should advance to idx 2
	eval = scenario.EvalWaypoint(&scen, 10.5, 0.0, &idx)
	if idx != 2 {
		t.Errorf("inside wp1 radius: idx = %d, want 2", idx)
	}

	// Within arrival radius of wp2 (last) → Done
	eval = scenario.EvalWaypoint(&scen, 20.5, 0.0, &idx)
	if !eval.Done {
		t.Error("inside last waypoint radius: expected Done=true")
	}
}

// TestEvalWaypointEmptyScenarioDone verifies that an empty waypoint list
// immediately returns Done.
func TestEvalWaypointEmptyScenarioDone(t *testing.T) {
	scen := scenario.Scenario{
		Timing:    scenario.ScenarioTiming{DtS: 0.01, DurationS: 10.0},
		Waypoints: []scenario.Waypoint{},
	}
	idx := 0
	eval := scenario.EvalWaypoint(&scen, 0, 0, &idx)
	if !eval.Done {
		t.Error("empty waypoints: expected Done=true immediately")
	}
}

// TestEvalWaypointDefaultArriveRadius verifies that a waypoint with ArriveR=0
// uses the default 2.0 m radius.
func TestEvalWaypointDefaultArriveRadius(t *testing.T) {
	scen := scenario.Scenario{
		Timing: scenario.ScenarioTiming{DtS: 0.01, DurationS: 10.0},
		Waypoints: []scenario.Waypoint{
			{X: 0.0, Y: 0.0, SpeedMPS: 5.5, GearPos: 1}, // ArriveR omitted → 0 → default 2.0
		},
	}
	idx := 0
	// 1.9 m away — within default 2.0 m radius
	eval := scenario.EvalWaypoint(&scen, 1.9, 0.0, &idx)
	if !eval.Done {
		t.Error("1.9 m from wp with default radius 2.0 m: expected Done=true")
	}
}

// TestLoadRealPathToGoalScenario verifies the new waypoint scenario loads cleanly.
func TestLoadRealPathToGoalScenario(t *testing.T) {
	scen, err := scenario.LoadScenario(realScenariosDir + "path_to_goal.json")
	if err != nil {
		t.Fatalf("LoadScenario path_to_goal: %v", err)
	}
	if scen.Meta.ControlMode != "waypoint_pid" {
		t.Errorf("control_mode = %q, want waypoint_pid", scen.Meta.ControlMode)
	}
	if len(scen.Waypoints) == 0 {
		t.Error("path_to_goal should have at least one waypoint")
	}
	// Verify at least one reverse waypoint exists
	hasReverse := false
	for _, wp := range scen.Waypoints {
		if wp.GearPos == 2 {
			hasReverse = true
			break
		}
	}
	if !hasReverse {
		t.Error("path_to_goal should contain at least one reverse waypoint (gear_position=2)")
	}
}
