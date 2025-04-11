import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd
import glob
import os
import datetime
from mpl_toolkits.mplot3d import Axes3D
from utils import logger, context

timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
directory = context.get_context(os.path.abspath(__file__))
DATA_DIR = f"{directory}/data"
PLOT_SAVE_PATH = f"{directory}/data/figs/{timestamp}_gyro_3d.png"
WINDOW = 100

def get_latest_csv():
  csv_files = glob.glob(os.path.join(DATA_DIR, "telemetry_log.csv"))
  if not csv_files:
    raise FileNotFoundError("No telemetry log CSV found in data directory.")
  return max(csv_files, key=os.path.getctime)

latest_csv = get_latest_csv()
df_preview = pd.read_csv(latest_csv, nrows=5)
columns = df_preview.columns

gyro_cols = [col for col in columns if "gyro" in col.lower()]
gyro_x = next((col for col in gyro_cols if "x" in col.lower()), "gyro.x")
gyro_y = next((col for col in gyro_cols if "y" in col.lower()), "gyro.y")
gyro_z = next((col for col in gyro_cols if "z" in col.lower()), "gyro.z")

fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

def animate(i):
  try:
    df = pd.read_csv(get_latest_csv())
    df = df.tail(WINDOW)

    x = df[gyro_x]
    y = df[gyro_y]
    z = df[gyro_z]

    ax.clear()
    ax.plot3D(x, y, z, label="Gyro 3D Path", color="blue")
    ax.set_xlabel("Position X")
    ax.set_ylabel("Position Y")
    ax.set_zlabel("Position Z")
    ax.set_title("Position Over Time")
    ax.legend()
    ax.grid(True)

  except Exception as e:
    print(f"[3D Plot Error] {e}")

ani = animation.FuncAnimation(fig, animate, interval=100)
plt.tight_layout()
plt.show()
fig.savefig(PLOT_SAVE_PATH)
print(f"Saved 3D plot to {PLOT_SAVE_PATH}")
