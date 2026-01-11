#!/usr/bin/env python3
"""
Auto-MPC Controller with CasADi Optimization
For XCMG XDE360 Heavy-Duty Mining Truck

This is a Python/CasADi version of the Go Auto-MPC controller,
with proper numerical optimization instead of heuristic control laws.

Author: Mario Tilocca
Vehicle: XCMG XDE360 (220-ton electric mining truck)
"""

import numpy as np
import casadi as ca
from dataclasses import dataclass
from typing import Tuple, List
import time


@dataclass
class AutoMPCConfig:
    """Configuration for Auto-MPC controller"""
    target_velocity_mps: float = 12.0
    prediction_horizon: int = 10
    control_horizon: int = 3
    dt: float = 0.05  # Time step (20 Hz control)
    
    # Cost function weights
    weight_velocity: float = 100.0
    weight_accel: float = 1.0
    weight_jerk: float = 0.1
    weight_terminal: float = 200.0
    
    # Constraints
    max_torque_nm: float = 145000.0
    max_brake_force_n: float = 180000.0
    max_accel_mps2: float = 1.5
    max_decel_mps2: float = 3.0
    max_velocity_mps: float = 20.0
    
    # Adaptation
    enable_adaptation: bool = True
    learning_rate: float = 0.01
    
    # Vehicle geometry
    wheel_radius_m: float = 1.95
    gear_ratio: float = 28.0


@dataclass
class VehicleModel:
    """Estimated vehicle dynamics model"""
    mass_kg: float = 220000.0          # 220 tons
    drag_coeff: float = 3.5            # N/(m/s)²
    rolling_resist_n: float = 8000.0   # Rolling resistance
    road_grade: float = 0.0            # Grade (radians)
    
    # Confidence in estimates (0-1)
    mass_confidence: float = 0.3
    drag_confidence: float = 0.3
    grade_confidence: float = 0.1


@dataclass
class ControlOutput:
    """Controller output"""
    torque_nm: float
    brake_pct: float
    is_accel: bool
    is_brake: bool
    confidence: float
    solve_time_ms: float = 0.0


class AutoMPCControllerCasadi:
    """
    Auto-MPC controller with CasADi numerical optimization
    
    Features:
    - Online vehicle parameter adaptation (mass, drag, rolling resistance)
    - Nonlinear MPC with proper QP/NLP solving
    - Constraint handling (torque, brake, acceleration limits)
    - Jerk minimization for passenger comfort
    """
    
    def __init__(self, config: AutoMPCConfig):
        self.cfg = config
        self.model = VehicleModel()
        
        # History for adaptation
        self.velocity_history: List[float] = []
        self.accel_history: List[float] = []
        self.torque_history: List[float] = []
        
        # Previous solution (warm start)
        self.prev_u = np.zeros(config.control_horizon)
        
        # Iteration counter
        self.iteration = 0
        
        # Build MPC optimization problem
        self._build_mpc_problem()
        
    def _build_mpc_problem(self):
        """Build CasADi optimization problem for MPC"""
        N = self.cfg.prediction_horizon
        M = self.cfg.control_horizon
        dt = self.cfg.dt
        
        # Decision variables: [u0, u1, ..., u_{M-1}]
        # where u = force (N) - positive = accel, negative = brake
        self.opti = ca.Opti()
        self.u = self.opti.variable(M)
        
        # Parameters (updated each solve)
        self.p_v0 = self.opti.parameter()        # Initial velocity
        self.p_vref = self.opti.parameter()      # Reference velocity
        self.p_mass = self.opti.parameter()      # Current mass estimate
        self.p_drag = self.opti.parameter()      # Current drag estimate
        self.p_rolling = self.opti.parameter()   # Current rolling resistance
        self.p_grade = self.opti.parameter()     # Current grade estimate
        
        # State prediction over horizon
        v = self.p_v0
        cost = 0
        u_prev = 0  # Previous control for jerk calculation
        
        for k in range(N):
            # Control input (piecewise constant over control horizon)
            if k < M:
                u_k = self.u[k]
            else:
                u_k = self.u[M-1]  # Hold last control
            
            # Vehicle dynamics: F = ma + F_drag + F_rolling + F_grade
            # a = (u - drag * v² - rolling - m*g*sin(grade)) / m
            drag_force = self.p_drag * v * ca.fabs(v)
            rolling_force = self.p_rolling * ca.sign(v)
            grade_force = self.p_mass * 9.81 * ca.sin(self.p_grade)
            
            accel = (u_k - drag_force - rolling_force - grade_force) / self.p_mass
            
            # Velocity update (Euler integration)
            v_next = v + accel * dt
            
            # Cost function
            velocity_error = v_next - self.p_vref
            cost += self.cfg.weight_velocity * velocity_error**2
            cost += self.cfg.weight_accel * accel**2
            
            # Jerk penalty (rate of control change)
            if k > 0:
                jerk = (u_k - u_prev) / dt
                cost += self.cfg.weight_jerk * jerk**2
            
            # Terminal cost (encourage settling at end of horizon)
            if k == N - 1:
                cost += self.cfg.weight_terminal * velocity_error**2
            
            # Update for next iteration
            v = v_next
            u_prev = u_k
            
            # Velocity constraint
            self.opti.subject_to(v >= 0)
            self.opti.subject_to(v <= self.cfg.max_velocity_mps)
        
        # Control constraints
        max_accel_force = self.cfg.max_accel_mps2 * self.p_mass
        max_decel_force = -self.cfg.max_decel_mps2 * self.p_mass
        
        for k in range(M):
            # Torque → force conversion
            max_motor_force = self._torque_to_force(self.cfg.max_torque_nm)
            max_brake_force = self.cfg.max_brake_force_n
            
            self.opti.subject_to(self.u[k] >= -max_brake_force)
            self.opti.subject_to(self.u[k] <= max_motor_force)
            self.opti.subject_to(self.u[k] >= max_decel_force)
            self.opti.subject_to(self.u[k] <= max_accel_force)
        
        # Objective
        self.opti.minimize(cost)
        
        # Solver options (IPOPT for NLP)
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_iter': 50,
            'ipopt.tol': 1e-4,
            'ipopt.warm_start_init_point': 'yes'
        }
        self.opti.solver('ipopt', opts)
        
    def update(self, current_velocity: float, dt: float) -> ControlOutput:
        """
        Compute optimal control using MPC
        
        Args:
            current_velocity: Current vehicle velocity (m/s)
            dt: Time since last update (s)
            
        Returns:
            ControlOutput with torque/brake commands
        """
        self.iteration += 1
        t_start = time.time()
        
        # Step 1: Adapt vehicle model
        if self.cfg.enable_adaptation and len(self.velocity_history) >= 3:
            self._adapt_model(current_velocity, dt)
        
        # Step 2: Solve MPC optimization
        try:
            # Set parameters
            self.opti.set_value(self.p_v0, current_velocity)
            self.opti.set_value(self.p_vref, self.cfg.target_velocity_mps)
            self.opti.set_value(self.p_mass, self.model.mass_kg)
            self.opti.set_value(self.p_drag, self.model.drag_coeff)
            self.opti.set_value(self.p_rolling, self.model.rolling_resist_n)
            self.opti.set_value(self.p_grade, self.model.road_grade)
            
            # Warm start with previous solution
            if len(self.prev_u) == self.cfg.control_horizon:
                self.opti.set_initial(self.u, self.prev_u)
            
            # Solve
            sol = self.opti.solve()
            
            # Extract first control action
            u_opt = sol.value(self.u)
            force_cmd = u_opt[0]
            
            # Store for warm start
            self.prev_u = u_opt
            
        except Exception as e:
            print(f"MPC solve failed: {e}. Using fallback control.")
            # Fallback: simple P control
            error = self.cfg.target_velocity_mps - current_velocity
            force_cmd = 5000.0 * error  # Proportional gain
        
        solve_time_ms = (time.time() - t_start) * 1000.0
        
        # Step 3: Convert force to actuator commands
        output = self._force_to_actuators(force_cmd)
        output.solve_time_ms = solve_time_ms
        
        # Step 4: Update history
        self._update_history(current_velocity, dt, output)
        
        return output
    
    def _adapt_model(self, velocity: float, dt: float):
        """Online adaptation of vehicle model parameters"""
        if len(self.velocity_history) < 2 or len(self.torque_history) < 1:
            return
        
        v_prev = self.velocity_history[-1]
        torque_prev = self.torque_history[-1]
        
        # Measured acceleration
        accel_measured = (velocity - v_prev) / dt
        
        # Convert torque to force
        force_prev = self._torque_to_force(torque_prev)
        
        # Predicted acceleration using current model
        drag = self.model.drag_coeff * v_prev * abs(v_prev)
        rolling = self.model.rolling_resist_n
        grade_force = self.model.mass_kg * 9.81 * np.sin(self.model.road_grade)
        
        accel_predicted = (force_prev - drag - rolling - grade_force) / self.model.mass_kg
        
        # Prediction error
        accel_error = accel_measured - accel_predicted
        
        # Adaptive learning rate
        alpha = self.cfg.learning_rate * (1.0 - 0.5 * self.model.mass_confidence)
        
        # Update mass (inverse relationship with acceleration)
        if abs(force_prev) > 1000:
            mass_update = alpha * accel_error * force_prev * 0.01
            self.model.mass_kg += mass_update
            self.model.mass_kg = np.clip(self.model.mass_kg, 50000, 400000)
            self.model.mass_confidence = min(0.99, self.model.mass_confidence + 0.002)
        
        # Update drag (at higher speeds)
        if abs(v_prev) > 2.0:
            drag_update = -alpha * accel_error * v_prev * abs(v_prev) * 0.05
            self.model.drag_coeff += drag_update
            self.model.drag_coeff = np.clip(self.model.drag_coeff, 0.5, 15.0)
            self.model.drag_confidence = min(0.99, self.model.drag_confidence + 0.001)
        
        # Update rolling resistance (at low speeds)
        if 0.5 < abs(v_prev) < 3.0:
            rolling_update = -alpha * accel_error * np.sign(v_prev) * 200.0
            self.model.rolling_resist_n += rolling_update
            self.model.rolling_resist_n = np.clip(self.model.rolling_resist_n, 100, 20000)
    
    def _torque_to_force(self, torque_nm: float) -> float:
        """Convert motor torque to wheel force"""
        # F = T * gear_ratio / wheel_radius
        return torque_nm * self.cfg.gear_ratio / self.cfg.wheel_radius_m
    
    def _force_to_torque(self, force_n: float) -> float:
        """Convert wheel force to motor torque"""
        # T = F * wheel_radius / gear_ratio
        return force_n * self.cfg.wheel_radius_m / self.cfg.gear_ratio
    
    def _force_to_actuators(self, force_n: float) -> ControlOutput:
        """Convert force command to torque/brake"""
        confidence = (self.model.mass_confidence + self.model.drag_confidence) / 2.0
        
        if force_n >= 0:
            # Acceleration - use motor
            torque = self._force_to_torque(force_n)
            torque = np.clip(torque, 0, self.cfg.max_torque_nm)
            return ControlOutput(
                torque_nm=torque,
                brake_pct=0.0,
                is_accel=True,
                is_brake=False,
                confidence=confidence
            )
        else:
            # Braking
            brake_force = -force_n
            brake_pct = (brake_force / self.cfg.max_brake_force_n) * 100.0
            brake_pct = np.clip(brake_pct, 0, 100)
            return ControlOutput(
                torque_nm=0.0,
                brake_pct=brake_pct,
                is_accel=False,
                is_brake=True,
                confidence=confidence
            )
    
    def _update_history(self, velocity: float, dt: float, output: ControlOutput):
        """Update history for adaptation"""
        self.velocity_history.append(velocity)
        if len(self.velocity_history) > 200:
            self.velocity_history.pop(0)
        
        if len(self.velocity_history) >= 2:
            v_prev = self.velocity_history[-2]
            accel = (velocity - v_prev) / dt
            self.accel_history.append(accel)
            if len(self.accel_history) > 200:
                self.accel_history.pop(0)
        
        self.torque_history.append(output.torque_nm)
        if len(self.torque_history) > 200:
            self.torque_history.pop(0)
    
    def get_diagnostics(self) -> dict:
        """Get diagnostic information"""
        return {
            'estimated_mass_kg': self.model.mass_kg,
            'estimated_drag': self.model.drag_coeff,
            'estimated_rolling_n': self.model.rolling_resist_n,
            'mass_confidence': self.model.mass_confidence,
            'drag_confidence': self.model.drag_confidence,
            'iteration': self.iteration,
        }
    
    def reset(self):
        """Reset controller state"""
        self.velocity_history.clear()
        self.accel_history.clear()
        self.torque_history.clear()
        self.prev_u = np.zeros(self.cfg.control_horizon)
        self.iteration = 0
        self.model = VehicleModel()


# ============================================================================
# EXAMPLE USAGE
# ============================================================================

def example_usage():
    """Example of using the Auto-MPC controller"""
    
    # Configure controller
    config = AutoMPCConfig(
        target_velocity_mps=12.0,
        prediction_horizon=10,
        control_horizon=3,
        dt=0.05,
        weight_velocity=100.0,
        weight_accel=1.0,
        weight_jerk=0.1,
        max_torque_nm=145000.0,
        max_brake_force_n=180000.0,
        enable_adaptation=True,
        learning_rate=0.05
    )
    
    # Create controller
    controller = AutoMPCControllerCasadi(config)
    
    # Simulation loop
    dt = 0.05  # 20 Hz
    current_velocity = 0.0
    
    print("Time\tVelocity\tTorque\t\tBrake\t\tMass Est\tSolve Time")
    print("-" * 80)
    
    for i in range(100):
        t = i * dt
        
        # Update controller
        output = controller.update(current_velocity, dt)
        
        # Simulate vehicle response (simplified)
        # In reality, this would come from vehicle-dynamics-sim
        accel = output.torque_nm / 220000.0 * 28.0 / 1.95
        if output.is_brake:
            accel = -output.brake_pct / 100.0 * 3.0  # Max 3 m/s² braking
        
        current_velocity += accel * dt
        current_velocity = max(0, current_velocity)
        
        # Print diagnostics
        if i % 10 == 0:
            diag = controller.get_diagnostics()
            print(f"{t:.2f}\t{current_velocity:.2f}\t\t{output.torque_nm:.0f}\t\t"
                  f"{output.brake_pct:.1f}\t\t{diag['estimated_mass_kg']:.0f}\t\t"
                  f"{output.solve_time_ms:.1f} ms")


if __name__ == "__main__":
    example_usage()