import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.dates as mdates
import pandas as pd
import glob
import os
import datetime
from utils import logger, context

timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
directory = context.get_context(os.path.abspath(__file__))
DATA_DIR = f"{directory}/data"
PLOT_SAVE_PATH = f"{directory}/data/figs/{timestamp}_telemetry.png"
WINDOW = 100 # Rolling window size

def get_latest_csv():
  csv_files = glob.glob(os.path.join(DATA_DIR, "telemetry_log.csv"))
  if not csv_files:
    raise FileNotFoundError("No telemetry log CSV found in data directory.")
  return max(csv_files, key=os.path.getctime)

latest_csv = get_latest_csv()
df_preview = pd.read_csv(latest_csv, nrows=5)
columns = df_preview.columns

gyro_cols = [col for col in columns if "gyro" in col]
cmd_cols = [col for col in columns if "cmd" in col]

gyro_x = next((col for col in gyro_cols if "x" in col.lower()), "gyro.x")
gyro_y = next((col for col in gyro_cols if "y" in col.lower()), "gyro.y")
gyro_z = next((col for col in gyro_cols if "z" in col.lower()), "gyro.z")

cmd_roll = next((col for col in cmd_cols if "roll" in col.lower()), "controller.cmd_roll")
cmd_pitch = next((col for col in cmd_cols if "pitch" in col.lower()), "controller.cmd_pitch")
cmd_yaw = next((col for col in cmd_cols if "yaw" in col.lower()), "controller.cmd_yaw")

fig, axs = plt.subplots(3, 2, figsize=(14, 8))
(ax1_left, ax1_right), (ax2_left, ax2_right), (ax3_left, ax3_right) = axs

def animate(i):
  try:
    df = pd.read_csv(get_latest_csv())
    df = df.tail(WINDOW)

    if "timestamp" not in df.columns:
      raise ValueError("Missing 'timestamp' column in telemetry CSV")

    df["timestamp"] = pd.to_datetime(df["timestamp"])

    for ax in axs.flat:
      ax.clear()

    # Row 1: Roll + Gyro.x
    ax1_left.plot(df["timestamp"], df[gyro_x], label="gyro.x")
    ax1_left.set_title("Roll Rate (gyro.x)")
    ax1_left.grid(True)

    ax1_right.plot(df["timestamp"], df[cmd_roll], label="cmd_roll", linestyle="--", color="orange")
    ax1_right.set_title("Roll PID Output (cmd_roll)")
    ax1_right.grid(True)

    # Row 2: Pitch + Gyro.y
    ax2_left.plot(df["timestamp"], df[gyro_y], label="gyro.y")
    ax2_left.set_title("Pitch Rate (gyro.y)")
    ax2_left.grid(True)

    ax2_right.plot(df["timestamp"], df[cmd_pitch], label="cmd_pitch", linestyle="--", color="orange")
    ax2_right.set_title("Pitch PID Output (cmd_pitch)")
    ax2_right.grid(True)

    # Row 3: Yaw + Gyro.z
    ax3_left.plot(df["timestamp"], df[gyro_z], label="gyro.z")
    ax3_left.set_title("Yaw Rate (gyro.z)")
    ax3_left.grid(True)

    ax3_right.plot(df["timestamp"], df[cmd_yaw], label="cmd_yaw", linestyle="--", color="orange")
    ax3_right.set_title("Yaw PID Output (cmd_yaw)")
    ax3_right.grid(True)

    for ax in axs.flat:
      ax.tick_params(axis='x', rotation=30)
      ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))

  except Exception as e:
    print(f"[Plot Error] {e}")

ani = animation.FuncAnimation(fig, animate, interval=500)
plt.tight_layout()
plt.show()
fig.savefig(PLOT_SAVE_PATH)
print(f"Saved plot to {PLOT_SAVE_PATH}")
