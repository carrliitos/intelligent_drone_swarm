LINEAR_PR = True
LINEAR_THRUST = True

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
