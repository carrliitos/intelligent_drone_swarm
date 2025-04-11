import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.dates as mdates

import pandas as pd
import glob
import os
import datetime
from utils import context

# Paths
timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
directory = context.get_context(os.path.abspath(__file__))
DATA_DIR = f"{directory}/data"
GYRO_CSV = f"{DATA_DIR}/telemetry_log.csv"
PID_CSV = f"{DATA_DIR}/PID_logs.csv"
FIG_SAVE_PATH = f"{directory}/data/figs/{timestamp}_combined_gyro_pid.png"
WINDOW = 100

def get_latest_csv(path):
  files = glob.glob(path)
  if not files:
    raise FileNotFoundError(f"No CSV found in {path}")
  return max(files, key=os.path.getctime)

# Prepare figure with 2 subplots: left = 3D, right = PID lines
fig = plt.figure(figsize=(14, 6))
ax3d = fig.add_subplot(1, 2, 1, projection='3d')
axpid = fig.add_subplot(1, 2, 2)

def animate(i):
  try:
    # Load GYRO DATA
    gyro_df = pd.read_csv(GYRO_CSV)
    gyro_df = gyro_df.tail(WINDOW)
    gyro_x = [col for col in gyro_df.columns if "gyro" in col.lower() and "x" in col.lower()][0]
    gyro_y = [col for col in gyro_df.columns if "gyro" in col.lower() and "y" in col.lower()][0]
    gyro_z = [col for col in gyro_df.columns if "gyro" in col.lower() and "z" in col.lower()][0]

    # Load PID DATA
    pid_df = pd.read_csv(PID_CSV)
    pid_df = pid_df.tail(WINDOW)
    pid_df["timestamp"] = pd.to_datetime(pid_df["timestamp"], unit="s")

    # Clear plots
    ax3d.clear()
    axpid.clear()

    # Plot 3D GYRO Path
    ax3d.plot3D(gyro_df[gyro_x], gyro_df[gyro_y], gyro_df[gyro_z], label="Gyro 3D Path", color="blue")
    ax3d.set_xlabel("Gyro X")
    ax3d.set_ylabel("Gyro Y")
    ax3d.set_zlabel("Gyro Z")
    ax3d.set_title("Gyro Orientation Over Time")
    ax3d.legend()
    ax3d.grid(True)

    # Plot PID Terms Over Time
    axpid.plot(pid_df["timestamp"], pid_df["proportional"], label="Proportional", linestyle="--", color="blue")
    # axpid.plot(pid_df["timestamp"], pid_df["integral"], label="Integral", linestyle="--", color="green")
    # axpid.plot(pid_df["timestamp"], pid_df["derivative"], label="Derivative", linestyle="--", color="red")
    axpid.set_title("PID Controller Terms")
    axpid.set_xlabel("Time")
    axpid.set_ylabel("Value")
    axpid.legend()
    axpid.grid(True)

    # Time formatting
    axpid.tick_params(axis='x', rotation=30)
    axpid.xaxis.set_major_locator(mdates.SecondLocator(interval=1))
    axpid.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))

  except Exception as e:
    print(f"[Plot Error] {e}")

ani = animation.FuncAnimation(fig, animate, interval=100)
plt.tight_layout()
plt.show()
fig.savefig(FIG_SAVE_PATH)
print(f"Saved combined plot to {FIG_SAVE_PATH}")
