import logging

class CustomFormatter(logging.Formatter):
  """
  Custom formatter to include line number only for ERROR logs.
  """
  def __init__(self):
    super().__init__()
    self.formatters = {
      logging.ERROR: logging.Formatter("%(asctime)s %(name)s [%(levelname)s] (%(lineno)d) %(message)s"),
      "default": logging.Formatter("%(asctime)s %(name)s [%(levelname)s] %(message)s"),
    }

  def format(self, record):
    # Use the ERROR-specific formatter if the level is ERROR
    formatter = self.formatters.get(record.levelno, self.formatters["default"])
    return formatter.format(record)

def setup_logger(logger_name, log_file, log_level=logging.DEBUG):
  """
  To set up as many loggers as needed.

  :param logger_name: Name of logger.
  :param log_file: Name of logging file.
  :param log_level: Log level. Defaults to DEBUG.
  """

  # Create and configure the logger
  logger = logging.getLogger(logger_name)
  logger.setLevel(log_level)

  # Custom formatter
  formatter = CustomFormatter()

  # File handler
  file_handler = logging.FileHandler(log_file)
  file_handler.setFormatter(formatter)
  logger.addHandler(file_handler)

  # Console handler
  console_handler = logging.StreamHandler()
  console_handler.setFormatter(formatter)
  logger.addHandler(console_handler)

  return logger
