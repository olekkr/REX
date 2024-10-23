import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import robot

arlo = robot.Robot()

# Initialize empty lists to hold the data for each sensor
sensor_1_data = []
sensor_2_data = []
sensor_3_data = []
sensor_4_data = []

# Time array (starting empty)
time_stamps = []

time_interval = 0.1
current_time = 0

# Create a figure and axis for plotting
fig, ax = plt.subplots(figsize=(10, 6))

# Initialize lines for each sensor
line1, = ax.plot([], [], label='Front Sensor', marker='o')
line2, = ax.plot([], [], label='Left Sensor', marker='o')
line3, = ax.plot([], [], label='Right Sensor', marker='o')
line4, = ax.plot([], [], label='Back Sensor', marker='o')

# Set plot limits (adjust as necessary)
ax.set_xlim(0, 60)  # X-axis (time) limit, adjust based on expected time range
ax.set_ylim(0, 5000)  # Y-axis (distance in mm)

# Set labels and title
ax.set_xlabel('Time (seconds)')
ax.set_ylabel('Distance (mm)')
ax.set_title('Real-Time Sonar Sensor Data')
ax.legend()

# Function to update the plot
def update(frame):
    global current_time

    # Read new data from the sensors
    sensor_1_value = arlo.read_front_ping_sensor()
    sensor_2_value = arlo.read_left_ping_sensor()
    sensor_3_value = arlo.read_right_ping_sensor()
    sensor_4_value = arlo.read_back_ping_sensor()

    # Append new readings to the data lists
    sensor_1_data.append(sensor_1_value)
    sensor_2_data.append(sensor_2_value)
    sensor_3_data.append(sensor_3_value)
    sensor_4_data.append(sensor_4_value)

    # Append the new time stamp
    time_stamps.append(current_time)

    # Update the line data
    line1.set_data(time_stamps, sensor_1_data)
    line2.set_data(time_stamps, sensor_2_data)
    line3.set_data(time_stamps, sensor_3_data)
    line4.set_data(time_stamps, sensor_4_data)

    # Auto-adjust the x-axis limits to show the latest data
    ax.set_xlim(max(0, current_time - 60), current_time + 1)

    # Increment the time
    current_time += time_interval

    return line1, line2, line3

ani = FuncAnimation(fig, update, interval=100)

arlo.go_diff(32,32,1,0)

# Display the plot
plt.show()
