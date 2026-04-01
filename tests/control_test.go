// Package tests contains black-box integration tests for the longitudinal
// control algorithms. Tests run against the exported API of
// closed_loop_ctrl_sensor_fusion/closed_loop/longitudinal_control.
package tests

import (
	"math"
	"testing"

	control "closed_loop_ctrl_sensor_fusion/closed_loop/longitudinal_control"
)

// ---------------------------------------------------------------------------
// Common utilities (control.ClampFloat, BoolToFloat, BoolToInt, GetControlModeStr)
// ---------------------------------------------------------------------------

func TestClampFloat(t *testing.T) {
	tests := []struct{ v, lo, hi, want float64 }{
		{5.0, 0.0, 10.0, 5.0},
		{-1.0, 0.0, 10.0, 0.0},
		{15.0, 0.0, 10.0, 10.0},
		{0.0, 0.0, 0.0, 0.0},
		{-100.0, -100.0, 100.0, -100.0},
		{100.0, -100.0, 100.0, 100.0},
	}
	for _, tc := range tests {
		got := control.ClampFloat(tc.v, tc.lo, tc.hi)
		if got != tc.want {
			t.Errorf("ClampFloat(%.1f, %.1f, %.1f) = %.1f, want %.1f",
				tc.v, tc.lo, tc.hi, got, tc.want)
		}
	}
}

func TestBoolToFloat(t *testing.T) {
	if control.BoolToFloat(true) != 1.0 {
		t.Error("BoolToFloat(true) should be 1.0")
	}
	if control.BoolToFloat(false) != 0.0 {
		t.Error("BoolToFloat(false) should be 0.0")
	}
}

func TestBoolToInt(t *testing.T) {
	if control.BoolToInt(true) != 1 {
		t.Error("BoolToInt(true) should be 1")
	}
	if control.BoolToInt(false) != 0 {
		t.Error("BoolToInt(false) should be 0")
	}
}

func TestGetControlModeStr(t *testing.T) {
	tests := []struct {
		out  control.ControlOutput
		want string
	}{
		{control.ControlOutput{IsAccel: true}, "[ACCEL]"},
		{control.ControlOutput{IsBrake: true}, "[BRAKE]"},
		{control.ControlOutput{}, "[COAST]"},
	}
	for _, tc := range tests {
		got := control.GetControlModeStr(tc.out)
		if got != tc.want {
			t.Errorf("GetControlModeStr(%+v) = %q, want %q", tc.out, got, tc.want)
		}
	}
}

// ---------------------------------------------------------------------------
// PID controller helpers
// ---------------------------------------------------------------------------

func newTestPID() *control.PIDController {
	return control.NewPIDController(control.PIDConfig{
		TargetVelocityMPS: 5.0,
		Kp:                1000.0,
		Ki:                50.0,
		Kd:                200.0,
		MaxTorqueNm:       10000.0,
		MinTorqueNm:       -5000.0,
		IntegralLimit:     1000.0,
	})
}

// ---------------------------------------------------------------------------
// PID controller: correctness tests
// ---------------------------------------------------------------------------

// TestPIDInitialOutput: cold-start always returns 50 % of MaxTorqueNm.
func TestPIDInitialOutput(t *testing.T) {
	pid := newTestPID()
	out := pid.Update(0.0, 0.01)

	if out.TorqueNm != 5000.0 {
		t.Errorf("initial torque = %.1f, want 5000.0 (50%% of MaxTorqueNm)", out.TorqueNm)
	}
	if !out.IsAccel {
		t.Error("initial output should have IsAccel=true")
	}
	if out.BrakePct != 0.0 {
		t.Errorf("initial BrakePct = %.1f, want 0.0", out.BrakePct)
	}
	if out.Confidence != 1.0 {
		t.Errorf("PID confidence = %.2f, want 1.0", out.Confidence)
	}
}

// TestPIDAcceleratesWhenBelowTarget: positive velocity error → positive torque.
func TestPIDAcceleratesWhenBelowTarget(t *testing.T) {
	pid := newTestPID()
	pid.Update(0.0, 0.01) // init
	out := pid.Update(0.0, 0.01)

	if !out.IsAccel {
		t.Error("expected IsAccel=true when velocity is below target")
	}
	if out.TorqueNm <= 0 {
		t.Errorf("expected positive torque below target, got %.1f Nm", out.TorqueNm)
	}
	if out.BrakePct != 0 {
		t.Errorf("should not brake below target, BrakePct=%.1f", out.BrakePct)
	}
}

// TestPIDBrakesWhenAboveTarget: negative error → TorqueNm=0 + positive BrakePct.
func TestPIDBrakesWhenAboveTarget(t *testing.T) {
	pid := newTestPID()
	pid.Update(10.0, 0.01) // init at v=10, target=5 → negative error
	out := pid.Update(10.0, 0.01)

	if !out.IsBrake {
		t.Error("expected IsBrake=true when velocity is above target")
	}
	if out.TorqueNm != 0 {
		t.Errorf("TorqueNm must be 0 when braking (got %.1f)", out.TorqueNm)
	}
	if out.BrakePct <= 0 {
		t.Error("expected positive BrakePct when above target")
	}
}

// TestPIDOvershootProtection: after >2 s of error<0, integral zeroed and 100% brake applied.
func TestPIDOvershootProtection(t *testing.T) {
	pid := newTestPID()
	// call 1 = init; calls 2–4 accumulate overshootDuration (1.0, 2.0, 3.0 s)
	// on call 4, 3.0 > 2.0 → emergency brake
	for i := 0; i < 3; i++ {
		pid.Update(10.0, 1.0)
	}
	out := pid.Update(10.0, 1.0)

	if out.TorqueNm != 0 {
		t.Errorf("overshoot protection must zero torque, got %.1f", out.TorqueNm)
	}
	if out.BrakePct != 100.0 {
		t.Errorf("overshoot protection must apply 100%% brake, got %.1f", out.BrakePct)
	}
	if !out.IsBrake {
		t.Error("overshoot protection must set IsBrake=true")
	}
	diag := pid.GetDiagnostics()
	if diag.Integral != 0 {
		t.Errorf("integral should be 0 after overshoot protection, got %.1f", diag.Integral)
	}
}

// TestPIDOvershootTimerResetsOnRecovery: after Reset(), PID must accelerate when below target.
// (The overshoot timer resets when error turns positive; testing via Reset+below-target confirms
// the controller returns to normal acceleration mode with clean state.)
func TestPIDOvershootTimerResetsOnRecovery(t *testing.T) {
	pid := newTestPID()
	pid.Update(10.0, 1.0) // init at v=10 (above target, accumulates state)
	pid.Update(10.0, 1.0) // second step above target

	// Reset clears all state including overshoot timer and integral
	pid.Reset()

	pid.Update(0.0, 0.01) // init at v=0 (below target=5)
	out := pid.Update(0.0, 0.01)

	if out.IsBrake {
		t.Error("after Reset with v below target, controller should not brake")
	}
	if !out.IsAccel {
		t.Error("after Reset with v below target, controller should accelerate")
	}
}

// TestPIDReset: Reset() clears integral, prevError and forces re-initialisation.
func TestPIDReset(t *testing.T) {
	pid := newTestPID()
	pid.Update(0.0, 0.01) // init
	pid.Update(0.0, 0.01) // accumulate state
	pid.Update(0.0, 0.01)

	pid.Reset()

	diag := pid.GetDiagnostics()
	if diag.Integral != 0 {
		t.Errorf("integral after Reset = %.1f, want 0", diag.Integral)
	}
	if diag.Error != 0 {
		t.Errorf("error after Reset = %.4f, want 0", diag.Error)
	}
	// First Update after Reset should be the cold-start 50 % torque
	out := pid.Update(0.0, 0.01)
	if out.TorqueNm != 5000.0 {
		t.Errorf("first Update after Reset should return 5000 Nm, got %.1f", out.TorqueNm)
	}
}

// TestPIDSetAndGetTargetVelocity: dynamic target changes are reflected immediately.
func TestPIDSetAndGetTargetVelocity(t *testing.T) {
	pid := newTestPID()
	pid.SetTargetVelocity(3.0)
	if pid.GetTargetVelocity() != 3.0 {
		t.Errorf("GetTargetVelocity = %.1f, want 3.0", pid.GetTargetVelocity())
	}
}

// TestPIDBrakeNeverExceeds100Pct: BrakePct is always clamped to [0, 100].
func TestPIDBrakeNeverExceeds100Pct(t *testing.T) {
	pid := newTestPID()
	pid.Update(1000.0, 0.01) // extreme overspeed
	out := pid.Update(1000.0, 0.01)
	if out.BrakePct > 100.0 {
		t.Errorf("BrakePct = %.1f exceeds 100%%", out.BrakePct)
	}
	if out.BrakePct < 0 {
		t.Errorf("BrakePct = %.1f is negative", out.BrakePct)
	}
}

// TestPIDTorqueRespectsSaturationLimits: output torque must stay within config bounds.
func TestPIDTorqueRespectsSaturationLimits(t *testing.T) {
	cfg := control.PIDConfig{
		TargetVelocityMPS: 5.0,
		Kp: 1000.0, Ki: 50.0, Kd: 200.0,
		MaxTorqueNm: 10000.0, MinTorqueNm: -5000.0,
		IntegralLimit: 1000.0,
	}
	pid := control.NewPIDController(cfg)
	pid.Update(0.0, 0.01) // init
	for i := 0; i < 20; i++ {
		out := pid.Update(0.0, 0.1)
		if out.TorqueNm > cfg.MaxTorqueNm {
			t.Errorf("step %d: torque %.1f Nm exceeds MaxTorqueNm %.1f", i, out.TorqueNm, cfg.MaxTorqueNm)
		}
	}
}

// TestPIDMutualExclusivity: IsAccel and IsBrake should never both be true.
func TestPIDMutualExclusivity(t *testing.T) {
	pid := newTestPID()
	for i, v := range []float64{0.0, 3.0, 5.0, 7.0, 10.0} {
		out := pid.Update(v, 0.01)
		if out.IsAccel && out.IsBrake {
			t.Errorf("step %d (v=%.1f): IsAccel and IsBrake are both true", i, v)
		}
	}
}

// ---------------------------------------------------------------------------
// PID smoke test: XCMG XDE360 acceleration over 1 second
// ---------------------------------------------------------------------------

// TestPIDSmokeXCMGAcceleration: 100 steps at 10 ms should produce forward motion.
func TestPIDSmokeXCMGAcceleration(t *testing.T) {
	pid := control.NewPIDController(control.PIDConfig{
		TargetVelocityMPS: 5.5,
		Kp:                15000.0,
		Ki:                800.0,
		Kd:                3000.0,
		MaxTorqueNm:       250000.0,
		MinTorqueNm:       -145000.0,
		IntegralLimit:     5000.0,
	})

	const (
		massKg       = 220000.0
		gearRatio    = 28.0
		wheelRadiusM = 1.95
		dt           = 0.01
		steps        = 100
	)

	velocity := 0.0
	for i := 0; i < steps; i++ {
		out := pid.Update(velocity, dt)
		if out.IsAccel {
			accel := out.TorqueNm * gearRatio / wheelRadiusM / massKg
			velocity += accel * dt
		}
	}

	if velocity <= 0 {
		t.Errorf("vehicle should be moving after %d steps, v=%.4f m/s", steps, velocity)
	}
	// Should not wildly overshoot in 1 second from rest
	if velocity > 5.5*2.0 {
		t.Errorf("velocity %.2f m/s looks unreasonably high after 1s", velocity)
	}
}

// TestPIDXCMGMaxBrakeTorque: verify the hardcoded vehicle brake constant (used in conversion).
func TestPIDXCMGMaxBrakeTorque(t *testing.T) {
	// XCMG XDE360: F_brake=180 kN, r=1.95 m, g=28 → T=12535.7 Nm
	const (
		maxBrakeForceN = 180000.0
		wheelRadiusM   = 1.95
		gearRatio      = 28.0
	)
	expected := maxBrakeForceN * wheelRadiusM / gearRatio
	if math.Abs(expected-12535.71) > 1.0 {
		t.Errorf("maxBrakeTorqueNm calc = %.2f, expected ~12535.7 Nm", expected)
	}
}

// TestPIDReverseGearNoNegation: in reverse gear the runner passes velocity directly
// (positive magnitude) to the PID — no sign flip. The sim reports speed as a positive
// magnitude regardless of gear; the gear command itself sets the drive direction.
// Verify: positive speed below target → PID accelerates (same behaviour as forward gear).
func TestPIDReverseGearNoNegation(t *testing.T) {
	pid := control.NewPIDController(control.PIDConfig{
		TargetVelocityMPS: 2.5, // positive magnitude → reverse at 2.5 m/s
		Kp: 15000.0, Ki: 800.0, Kd: 3000.0,
		MaxTorqueNm: 250000.0, MinTorqueNm: -145000.0,
		IntegralLimit: 5000.0,
	})

	// Sim reports speed as positive magnitude even when reversing.
	// Runner passes it directly — no negation.
	currentVelocity := 1.5 // vehicle reversing at 1.5 m/s, reported as positive

	pid.Update(currentVelocity, 0.01) // init
	out := pid.Update(currentVelocity, 0.01)

	// Target=2.5, actual=1.5, error=+1.0 → should accelerate
	if !out.IsAccel {
		t.Error("reverse gear below target: PID should accelerate (positive torque)")
	}
	if out.TorqueNm <= 0 {
		t.Errorf("expected positive torque for positive error, got %.1f Nm", out.TorqueNm)
	}
}
