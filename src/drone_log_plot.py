import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.dates as mdates
from matplotlib.ticker import MaxNLocator

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
fig = plt.figure(figsize=(14, 12))
# Top row: 3D on left, PID on right
ax3d = plt.subplot2grid((3, 2), (0, 0), projection='3d')
axpid = plt.subplot2grid((3, 2), (0, 1))
# Second row: thrust on left, battery on right
axthrust = plt.subplot2grid((3, 2), (1, 0))
axbattery = plt.subplot2grid((3, 2), (1, 1))
# Third row for motors
axmotors = plt.subplot2grid((3, 2), (2, 0), colspan=2)

def animate(i):
  try:
    # Load GYRO + THRUST DATA
    gyro_df = pd.read_csv(GYRO_CSV)
    gyro_df = gyro_df.tail(WINDOW)

    # Parse timestamp for telemetry
    if "timestamp" in gyro_df.columns:
      gyro_df["timestamp"] = pd.to_datetime(gyro_df["timestamp"], unit="s")

    # Detect gyro columns
    gyro_x = [col for col in gyro_df.columns if "gyro" in col.lower() and "x" in col.lower()][0]
    gyro_y = [col for col in gyro_df.columns if "gyro" in col.lower() and "y" in col.lower()][0]
    gyro_z = [col for col in gyro_df.columns if "gyro" in col.lower() and "z" in col.lower()][0]

    # Load PID DATA
    pid_df = pd.read_csv(PID_CSV)
    pid_df = pid_df.tail(WINDOW)
    pid_df["timestamp"] = pd.to_datetime(pid_df["timestamp"], unit="s")

    # Clear all axes
    ax3d.clear()
    axpid.clear()
    axthrust.clear()
    axbattery.clear()
    axmotors.clear()

    # Plot 3D GYRO Path
    ax3d.xaxis.set_major_locator(MaxNLocator(nbins=5))
    ax3d.yaxis.set_major_locator(MaxNLocator(nbins=5))
    ax3d.zaxis.set_major_locator(MaxNLocator(nbins=5))

    ax3d.plot3D(gyro_df[gyro_x], gyro_df[gyro_y], gyro_df[gyro_z], label="Gyro 3D Path", color="blue")
    ax3d.set_xlabel("X")
    ax3d.set_ylabel("Y")
    ax3d.set_zlabel("Z")
    ax3d.legend()
    ax3d.grid(True)

    # Plot PID Terms
    axpid.plot(pid_df["timestamp"], pid_df["proportional"], label="Proportional", linestyle="--", color="blue")
    axpid.plot(pid_df["timestamp"], pid_df["integral"], label="Integral", linestyle="--", color="green")
    axpid.plot(pid_df["timestamp"], pid_df["derivative"], label="Derivative", linestyle="--", color="red")
    axpid.set_xlabel("Time")
    axpid.set_ylabel("Value")
    axpid.legend()
    axpid.grid(True)
    axpid.tick_params(axis='x', rotation=30)
    axpid.xaxis.set_major_locator(mdates.SecondLocator(interval=5))
    axpid.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))

    if "thrust" in gyro_df.columns and "timestamp" in gyro_df.columns:
      axthrust.plot(gyro_df["timestamp"], gyro_df["thrust"], label="Thrust", color="purple")
      axthrust.set_xlabel("Time")
      axthrust.set_ylabel("Thrust")
      axthrust.legend()
      axthrust.grid(True)
      axthrust.tick_params(axis='x', rotation=30)
      axthrust.xaxis.set_major_locator(mdates.SecondLocator(interval=5))
      axthrust.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
    else:
      axthrust.set_title("No 'thrust' or 'timestamp' column in telemetry log.")

    if "pm_batteryLevel" in gyro_df.columns and "timestamp" in gyro_df.columns:
      axbattery.plot(gyro_df["timestamp"], gyro_df["pm_batteryLevel"], label="Battery Level", color="orange")
      axbattery.set_xlabel("Time")
      axbattery.set_ylabel("Battery (%)")
      axbattery.legend()
      axbattery.grid(True)
      axbattery.tick_params(axis='x', rotation=30)
      axbattery.xaxis.set_major_locator(mdates.SecondLocator(interval=5))
      axbattery.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
    else:
      axbattery.set_title("No 'pm_batteryLevel' or 'timestamp' column in telemetry log.")

    # Bar Plot for Motor Values
    latest = gyro_df.iloc[-1] if not gyro_df.empty else None
    if latest is not None and all(k in latest for k in ["motor_m1", "motor_m2", "motor_m3", "motor_m4"]):
      motor_labels = ["M1", "M2", "M3", "M4"]
      motor_values = [latest["motor_m1"], latest["motor_m2"], latest["motor_m3"], latest["motor_m4"]]
      axmotors.bar(motor_labels, motor_values, color=["red", "green", "blue", "orange"])
      axmotors.set_ylim(0, max(50000, max(motor_values) * 1.1))
      axmotors.set_title("Motor Output Levels")
      axmotors.set_ylabel("PWM Value")
      axmotors.grid(True)
    else:
      axmotors.set_title("Motor data unavailable")

  except Exception as e:
    print(f"[Plot Error] {e}")

ani = animation.FuncAnimation(fig, animate, interval=100)
plt.tight_layout()
fig.subplots_adjust(top=0.97, bottom=0.06, left=0.06, right=0.98, hspace=0.45, wspace=0.3)
plt.show()
fig.savefig(FIG_SAVE_PATH)
print(f"Saved combined plot to {FIG_SAVE_PATH}")
