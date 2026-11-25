
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading
import time

# Serial Configuration
PORT = "COM12"
BAUDRATE = 115200

# Data Storage
MAX_POINTS = 300

time_data = deque(maxlen=MAX_POINTS)
gyro_data = deque(maxlen=MAX_POINTS)

start_time = time.time()

# Serial Connection
ser = serial.Serial(
    PORT,
    BAUDRATE,
    timeout=0.1
)

gimbal_enabled = False

time.sleep(2)
ser.reset_input_buffer()

# Serial Reader
def read_serial():

    while True:

        try:
            line = ser.readline().decode(
                "utf-8"
            ).strip()

            if not line:
                continue

            value = float(line)

            current_time = time.time() - start_time

            time_data.append(current_time)
            gyro_data.append(value)

        except ValueError:
            continue

        except Exception as e:
            print("Serial Error:", e)
            break


# Background Thread
thread = threading.Thread(
    target=read_serial,
    daemon=True
)

thread.start()

# Plot Configuration
fig, ax = plt.subplots(figsize=(10, 5))

line, = ax.plot(
    [],
    [],
    label="Filtered Gyro Z",
    linewidth=1.5
)

ax.set_title("Gimbal - Gyroscope Live Data")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Angular Velocity (deg/s)")

ax.grid(True)
ax.legend()

def on_key(event):
    global gimbal_enabled

    if event.key == ' ':

        gimbal_enabled = not gimbal_enabled

        if gimbal_enabled:
            ser.write(b'1')
            print("GIMBAL ENABLED")

        else:
            ser.write(b'0')
            print("GIMBAL DISABLED")

# Update Plot
def update(frame):

    if len(time_data) == 0:
        return line,

    line.set_data(
        list(time_data),
        list(gyro_data)
    )

    ax.relim()
    ax.autoscale_view()

    return line,

   
ani = animation.FuncAnimation(
    fig,
    update,
    interval=50,
    blit=False,
    cache_frame_data=False
)

fig.canvas.mpl_connect('key_press_event', on_key)

plt.tight_layout()
plt.show()

ser.close()
