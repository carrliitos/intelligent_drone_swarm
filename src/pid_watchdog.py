import pandas as pd
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import time
import os
from pathlib import Path

from utils import logger, context

directory = context.get_context(os.path.abspath(__file__))
logger_name = Path(__file__).stem
logger = logger.setup_logger(logger_name, f"{directory}/logs/{logger_name}.log")

class CSVFileHandler(FileSystemEventHandler):
  def __init__(self, command):
    super().__init__()
    self.command = command
    self.last_event_time = 0

  def on_modified(self, event):
    if event.src_path.endswith("pid.csv"):
      current_time = time.time()
      if current_time - self.last_event_time < 0.5:
        return

      self.last_event_time = current_time
      df = pd.read_csv(event.src_path)

      roll_kp = float(df.loc[df["k"] == "roll", "p"].iloc[0])
      roll_ki = float(df.loc[df["k"] == "roll", "i"].iloc[0])
      roll_kd = float(df.loc[df["k"] == "roll", "d"].iloc[0])

      pitch_kp = float(df.loc[df["k"] == "pitch", "p"].iloc[0])
      pitch_ki = float(df.loc[df["k"] == "pitch", "i"].iloc[0])
      pitch_kd = float(df.loc[df["k"] == "pitch", "d"].iloc[0])

      yaw_kp = float(df.loc[df["k"] == "yaw", "p"].iloc[0])
      yaw_ki = float(df.loc[df["k"] == "yaw", "i"].iloc[0])
      yaw_kd = float(df.loc[df["k"] == "yaw", "d"].iloc[0])

      self.command.roll_rate_pid.kp = roll_kp
      self.command.roll_rate_pid.ki = roll_ki
      self.command.roll_rate_pid.kd = roll_kd

      self.command.pitch_rate_pid.kp = pitch_kp
      self.command.pitch_rate_pid.ki = pitch_ki
      self.command.pitch_rate_pid.kd = pitch_kd

      self.command.yaw_rate_pid.kp = yaw_kp
      self.command.yaw_rate_pid.ki = yaw_ki
      self.command.yaw_rate_pid.kd = yaw_kd

      print(f"Updated PID values:\n"
            f"  Roll  -> P: {roll_kp}, I: {roll_ki}, D: {roll_kd}\n"
            f"  Pitch -> P: {pitch_kp}, I: {pitch_ki}, D: {pitch_kd}\n"
            f"  Yaw   -> P: {yaw_kp}, I: {yaw_ki}, D: {yaw_kd}")

