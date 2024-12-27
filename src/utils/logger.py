import logging

formatter = logging.Formatter("%(asctime)s %(name)s [%(levelname)s] %(message)s")

def setup_logger(logger_name, log_file, log_level=logging.DEBUG):
  """
  To set up as many loggers as needed

  :param logger_name: Name of logger.
  :param log_file: Name of logging file.
  :param log_level: Log level. Defaults to DEBUG
  """

  root_logger = logging.getLogger(logger_name)
  root_logger.setLevel(log_level)

  file_handler = logging.FileHandler(log_file)
  file_handler.setFormatter(formatter)
  root_logger.addHandler(file_handler)

  console_handler = logging.StreamHandler()
  console_handler.setFormatter(formatter)
  root_logger.addHandler(console_handler)

  return root_logger
