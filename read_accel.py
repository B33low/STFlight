import serial
from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np

# Open the serial connection
# Set timeout for non-blocking reads
arduino = serial.Serial('/dev/ttyACM1', 115200, timeout=0.1)

# Drop the first line of data
arduino.readline()

# Set up the figure and axes
fig = plt.figure()
# Adjusted y-limits for better visualization
ax = plt.axes(xlim=(0, 50), ylim=(-2, 2))

max_points = 50
# Initialize line objects for each axis
linex, = ax.plot(np.arange(max_points),
                 np.ones(max_points, dtype=np.float32) * np.nan,
                 lw=2,
                 label='x')

liney, = ax.plot(np.arange(max_points),
                 np.ones(max_points, dtype=np.float32) * np.nan,
                 lw=2,
                 label='y')

linez, = ax.plot(np.arange(max_points),
                 np.ones(max_points, dtype=np.float32) * np.nan,
                 lw=2,
                 label='z')

ax.legend()
ax.grid()

# Initialize the plot


def init():
    return linex, liney, linez

# Animation update function


def animate(i):
    try:
        # Read all available data from the serial buffer
        raw_data = arduino.read_all().decode().splitlines()

        # If there's valid data, process the last line
        if raw_data:
            raw_s = raw_data[-1].replace('\r\n', '').split(',')
            # Parse the x, y, z values from the received data
            x = int(raw_s[0][3:])/4200
            y = int(raw_s[1][4:])/4200
            z = int(raw_s[2][4:])/4200
            print(x, y, z)

            # Update the x, y, and z line data
            old_x = linex.get_ydata()  # grab current data
            new_x = np.r_[old_x[1:], x]  # append new data
            linex.set_ydata(new_x)  # update y data for x

            old_y = liney.get_ydata()  # grab current data
            new_y = np.r_[old_y[1:], y]  # append new data
            liney.set_ydata(new_y)  # update y data for y

            old_z = linez.get_ydata()  # grab current data
            new_z = np.r_[old_z[1:], z]  # append new data
            linez.set_ydata(new_z)  # update y data for z

    except Exception as e:
        print("Error:", e)  # Print error if the data parsing fails
        pass

    return linex, liney, linez  # return the updated lines for redrawing


# Create the animation with a smaller interval for faster updates
anim = animation.FuncAnimation(
    fig, animate, init_func=init, frames=200, interval=50, blit=True)

# Show the plot
plt.show()
