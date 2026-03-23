package main

// This file re-exports the scenario types and functions from the importable
// sub-package closed_loop/scenario so that package main can use them without
// duplication and external test packages can import them directly.

import scenpkg "closed_loop_ctrl_sensor_fusion/closed_loop/scenario"

// Type aliases — runner.go and main.go use these names directly.
type Scenario = scenpkg.Scenario
type ScenarioMeta = scenpkg.ScenarioMeta
type ScenarioTiming = scenpkg.ScenarioTiming
type ScenarioSegment = scenpkg.ScenarioSegment
type ActuatorCmd = scenpkg.ActuatorCmd
type SegmentEvaluation = scenpkg.SegmentEvaluation
type Waypoint = scenpkg.Waypoint
type WaypointEval = scenpkg.WaypointEval

// Function wrappers — keep call sites in runner.go unchanged.
func LoadScenario(path string) (Scenario, error) { return scenpkg.LoadScenario(path) }
func EvalSegment(scen *Scenario, t float64) SegmentEvaluation {
	return scenpkg.EvalSegment(scen, t)
}
func EvalActCmd(scen *Scenario, t float64) ActuatorCmd { return scenpkg.EvalActCmd(scen, t) }
func EvalWaypoint(scen *Scenario, x, y float64, idx *int) WaypointEval {
	return scenpkg.EvalWaypoint(scen, x, y, idx)
}
