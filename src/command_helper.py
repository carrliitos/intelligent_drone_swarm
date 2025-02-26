LINEAR_PR = True     # Linear Pitch and Roll -- the calculation for pitch and roll follows a linear transformation.
LINEAR_THRUST = True # Linear Thrust         -- the calculation for thrust follows a linear transformation.

from abc import ABC, abstractmethod

class CrazyFlieDataProviderProtocol(ABC):
  @property
  @abstractmethod
  def value(self):
    pass

class CrazyFlieXProvideable:
  def __init__(self, x: float):
    self.x = x

class CrazyFlieYProvideable:
  def __init__(self, y: float):
    self.y = y

class SimpleXDataProvider(CrazyFlieDataProviderProtocol):
  def __init__(self, providable: CrazyFlieXProvideable):
    self.providable = providable
  
  @property
  def value(self):
    return self.providable.x

class SimpleYDataProvider(CrazyFlieDataProviderProtocol):
  def __init__(self, providable: CrazyFlieYProvideable):
    self.providable = providable
  
  @property
  def value(self):
    return self.providable.y

class CrazyFlieDataProvider:
  def __init__(self, provider):
    if isinstance(provider, CrazyFlieXProvideable):
      self.provider = SimpleXDataProvider(provider)
    elif isinstance(provider, CrazyFlieYProvideable):
      self.provider = SimpleYDataProvider(provider)
    else:
      raise TypeError("Unsupported provider type")

class BoundsValue:
  def __init__(self, min_value: float, max_value: float, value: float = 0):
    self.min_value = min_value
    self.max_value = max_value
    self.value = value

class CommandHelper:
  def __init__(self, pitch_provider, roll_provider, yaw_provider, thrust_provider, settings, allow_negative_values=True):
    self.pitch_provider = pitch_provider
    self.yaw_provider = yaw_provider
    self.roll_provider = roll_provider
    self.thrust_provider = thrust_provider
    self.allow_negative_values = allow_negative_values

    self.pitch_rate = settings["pitchRate"]
    self.yaw_rate = settings["yawRate"]
    self.max_thrust = settings["maxThrust"]

    self.pitch_bounds = BoundsValue(0, 1)
    self.roll_bounds = BoundsValue(0, 1)
    self.thrust_bounds = BoundsValue(0, 1)
    self.yaw_bounds = BoundsValue(0, 1)
  
  @property
  def pitch(self):
    return self.pitch_bounds.value
  
  @property
  def roll(self):
    return self.roll_bounds.value
  
  @property
  def thrust(self):
    return self.thrust_bounds.value
  
  @property
  def yaw(self):
    return self.yaw_bounds.value

  def prepare_data(self):
    self.pitch_bounds.value = self._pitch(self.pitch_provider.provider.value)
    self.roll_bounds.value = self._roll(self.roll_provider.provider.value)
    self.thrust_bounds.value = self._thrust(self.thrust_provider.provider.value)
    self.yaw_bounds.value = self._yaw(self.yaw_provider.provider.value)
  
  def _pitch(self, control: float) -> float:
    """
    Computes the pitch adjustment based on the control input and `LINEAR_PR` setting.
    
    - When `LINEAR_PR = True`: Uses a direct linear transformation.
      `pitch = control * -1 * pitch_rate`
    - When `LINEAR_PR = False`: Uses a quadratic transformation for smoother sensitivity.
      `pitch = (control^2) * -1 * pitch_rate * (1 if control > 0, otherwise -1)`

    Args:
      control (float): The input control value for pitch.
    
    Returns:
      float: The computed pitch adjustment.
    """
    if LINEAR_PR:
      if control >= 0 or self.allow_negative_values:
        return control * -1 * self.pitch_rate
    else:
      if control >= 0:
        return pow(control, 2) * -1 * self.pitch_rate * (1 if control > 0 else -1)
    return 0
  
  def _roll(self, control: float) -> float:
    """
    Computes the roll adjustment based on the control input and `LINEAR_PR` setting.

    - When `LINEAR_PR = True`: Uses a direct linear transformation.
      `roll = control * pitch_rate`
    - When `LINEAR_PR = False`: Uses a quadratic transformation for smoother sensitivity.
      `roll = (control^2) * pitch_rate * (1 if control > 0, otherwise -1)`

    Args:
      control (float): The input control value for roll.
    
    Returns:
      float: The computed roll adjustment.
    """
    if LINEAR_PR:
      if control >= 0 or self.allow_negative_values:
        return control * self.pitch_rate
    else:
      if control >= 0:
        return pow(control, 2) * self.pitch_rate * (1 if control > 0 else -1)
    return 0
  
  def _yaw(self, control: float) -> float:
    return control * self.yaw_rate
  
  def _thrust(self, control: float) -> float:
    """
    Computes the thrust adjustment based on the control input and `LINEAR_THRUST` setting.

    - When `LINEAR_THRUST = True`: Uses a direct linear transformation.
      `thrust = control * 65535 * (max_thrust / 100)`
    - When `LINEAR_THRUST = False`: Uses a square root transformation for smoother thrust scaling.
      `thrust = (control^0.5) * 65535 * (max_thrust / 100)`

    Args:
        control (float): The input control value for thrust.

    Returns:
        float: The computed thrust adjustment, clamped between 0 and 65535.
    """
    thrust = control * 65535 * (self.max_thrust / 100) if LINEAR_THRUST else (control ** 0.5) * 65535 * (self.max_thrust / 100)
    return min(max(thrust, 0), 65535)
