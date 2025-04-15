import serial
from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np

# Open the serial connection
# Set timeout for non-blocking reads
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)

# Drop the first line of data
arduino.readline()

# Set up the figure and axes
# Create figure and two axes for pressure (left) and temp (right)
fig = plt.figure()
ax1 = fig.add_subplot(111)
ax2 = ax1.twinx()

max_points = 50
# Initialize line objects for each axis
linep, = ax1.plot(np.arange(max_points),
                  np.ones(max_points, dtype=np.float32) * np.nan,
                  lw=2,
                  label='pressure')

linet, = ax2.plot(np.arange(max_points),
                  np.ones(max_points, dtype=np.float32) * np.nan,
                  lw=2,
                  label='temp',
                  c='orange')

# Set X limits the same on both axes
ax1.set_xlim(0, 50)
ax2.set_xlim(0, 50)

# Pressure on the left axis: 1000..1100
ax1.set_ylim(1000, 1100)

# Temperature on the right axis: 17..40
ax2.set_ylim(17, 40)

ax1.legend()
ax2.legend()
ax1.grid()

# Initialize the plot


def init():
    return linep, linet

# Animation update function


def animate(i):
    try:
        # Read all available data from the serial buffer
        raw_data = arduino.read_all().decode().splitlines()
        if len(raw_data) < 2:
            return linep, linet

        # If there's valid data, process the last line
        if raw_data:

            raw_s = raw_data[-2].replace('\r\n', '').split(',')
            print(raw_data[-2])  # Print error if the data parsing fails

            # print(raw_s)
            # Parse the x, y, z values from the received data
            p = float(raw_s[0][3:])
            t = float(raw_s[1][4:])
            # print(x, y, z)

            # Update the x, y, and z line data
            old_p = linep.get_ydata()  # grab current data
            new_p = np.r_[old_p[1:], p]  # append new data
            linep.set_ydata(new_p)  # update y data for x

            old_t = linet.get_ydata()  # grab current data
            new_t = np.r_[old_t[1:], t]  # append new data
            linet.set_ydata(new_t)  # update y data for y

    except Exception as e:
        # print(raw_data[-2])  # Print error if the data parsing fails

        pass

    return linep, linet  # return the updated lines for redrawing


# Create the animation with a smaller interval for faster updates
anim = animation.FuncAnimation(
    fig, animate, init_func=init, frames=200, interval=100, blit=True)

# Show the plot
plt.show()
