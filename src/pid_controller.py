# pid_controller.py
# -*- coding: utf-8-*-
# @Author: Benzon Carlitos Salazar (carrliitos)
# @Email:  salazarbc24@uww.edu
# @Date:   2024-03-14
#
# @Last update:   2024-03-14 04:23 PM
#
# This module implements a PID (Proportional-Integral-Derivative) controller.
# A PID controller is a feedback control mechanism commonly used in control 
# systems to maintain a desired setpoint by continuously adjusting an output 
# based on the error between the setpoint and the measured value.
#
# The PID controller in this implementation:
# - Computes a control output based on proportional (P), integral (I), 
#   and derivative (D) terms.
# - Uses an anti-windup mechanism to prevent excessive integral accumulation.
# - Computes the derivative term based on the measured value rather than 
#   error to improve stability.
# - Clamps the output within specified limits to ensure it stays within 
#   the allowable range.
# - Supports configurable sample time for updates.

import time

class PIDController:
  def __init__(self, kp, ki, kd, setpoint=0.0, output_limits=(0, 65535), sample_time=0.01):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.setpoint = setpoint
    self.output_limits = output_limits
    self.sample_time = sample_time

    self.integral = 0.0
    self.previous_error = 0.0
    self.previous_input = None
    self.last_time = time.monotonic()

  def _clamp(self, value, limits):
    """Clamp the value within output limits."""
    lower, upper = limits
    return max(lower, min(value, upper))

  def update(self, measured_value, dt=None):
    """Compute PID output given a measured value."""
    now = time.monotonic()
    if dt is None:
      dt = now - self.last_time
    if dt <= 0:
      dt = 1e-16  # Prevent division by zero
    
    error = self.setpoint - measured_value
    d_input = measured_value - (self.previous_input if self.previous_input is not None else measured_value)
    d_error = error - self.previous_error

    # Compute Proportional Term
    proportional = self.kp * error

    # Compute Integral Term (with anti-windup)
    self.integral += self.ki * error * dt
    self.integral = self._clamp(self.integral, self.output_limits)

    # Compute Derivative Term (on measurement)
    derivative = -self.kd * d_input / dt if self.previous_input is not None else 0

    # Compute final output and clamp
    output = proportional + self.integral + derivative
    output = self._clamp(output, self.output_limits)

    # Save state for next iteration
    self.previous_error = error
    self.previous_input = measured_value
    self.last_time = now

    return int(output)

  def set_setpoint(self, setpoint):
    """Update the target setpoint."""
    self.setpoint = setpoint

  def reset(self):
    """Reset PID internals."""
    self.integral = 0.0
    self.previous_error = 0.0
    self.previous_input = None
    self.last_time = time.monotonic()
