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

    self.last_proportional = 0.0
    self.last_integral = 0.0
    self.last_derivative = 0.0

  def _clamp(self, value, limits):
    lower, upper = limits
    return max(lower, min(value, upper))

  def update(self, measured_value, dt=None):
    now = time.monotonic()
    if dt is None:
      dt = now - self.last_time
    if dt <= 0:
      dt = 1e-16

    error = self.setpoint - measured_value
    d_input = measured_value - (self.previous_input if self.previous_input is not None else measured_value)
    d_error = error - self.previous_error

    # Proportional
    proportional = self.kp * error

    # Integral
    self.integral += self.ki * error * dt
    self.integral = self._clamp(self.integral, self.output_limits)

    # Derivative (on measurement)
    derivative = -self.kd * d_input / dt if self.previous_input is not None else 0

    # Final Output
    output = proportional + self.integral + derivative
    output = self._clamp(output, self.output_limits)

    # Save state
    self.previous_error = error
    self.previous_input = measured_value
    self.last_time = now

    self.last_proportional = proportional
    self.last_integral = self.integral
    self.last_derivative = derivative

    return int(output)

  def get_kpe(self):
    """Return the proportional term (Kp × error)."""
    return self.last_proportional

  def get_kie(self):
    """Return the integral term (Ki × ∫error)."""
    return self.last_integral

  def get_kde(self):
    """Return the derivative term (Kd × d(error)/dt)."""
    return self.last_derivative

  def set_setpoint(self, setpoint):
    self.setpoint = setpoint

  def reset(self):
    self.integral = 0.0
    self.previous_error = 0.0
    self.previous_input = None

    self.last_proportional = 0.0
    self.last_integral = 0.0
    self.last_derivative = 0.0
    
    self.last_time = time.monotonic()
