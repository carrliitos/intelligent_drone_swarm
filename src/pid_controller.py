class PIDController:
  def __init__(self, kp, ki, kd, setpoint=0.0, output_limits=(0, 65535)):
    self.kp = kp              # Proportional gain
    self.ki = ki              # Integral gain
    self.kd = kd              # Derivative gain
    self.setpoint = setpoint  # Desired altitude
    self.output_limits = output_limits

    self.integral = 0.0
    self.previous_error = 0.0

  def update(self, measured_value, dt):
    error = self.setpoint - measured_value
    self.integral += error * dt
    derivative = (error - self.previous_error) / dt

    # PID calculation
    output = (
      (self.kp * error) + 
      (self.ki * self.integral) + 
      (self.kd * derivative)
    )

    # Save error for next derivative calculation
    self.previous_error = error

    # Clamp output to limits
    output = max(min(output, self.output_limits[1]), self.output_limits[0])
    return int(output)

  def set_setpoint(self, setpoint):
    self.setpoint = setpoint
