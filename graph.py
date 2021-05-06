import csv
import sys
import matplotlib.pyplot as plt

fname = sys.argv[1]
data = []
with open(fname) as csvfile:
    rdr = csv.reader(csvfile)
    next(rdr)
    for row in rdr:
        data.append([float(r) for r in row])

fig, axs = plt.subplots(4, 1, constrained_layout=True)
fig.suptitle('Estimated vs. True State Information')
axs[0].plot(list(range(0, len(data))), [r[0]
                                        for r in data], label="estimated x")
axs[0].plot(list(range(0, len(data))), [r[4] for r in data], label="true x")
axs[0].legend()
axs[0].set_title('X Position')
axs[0].set_xlabel('Time')
axs[0].set_ylabel('Position (m)')

axs[1].plot(list(range(0, len(data))), [r[1]
                                        for r in data], label="estimated y")
axs[1].plot(list(range(0, len(data))), [r[5] for r in data], label="true y")
axs[1].legend()
axs[1].set_title('Y Position')
axs[1].set_xlabel('Time')
axs[1].set_ylabel('Position (m)')

axs[2].plot(list(range(0, len(data))), [r[2]
                                        for r in data], label="estimated vx")
axs[2].plot(list(range(0, len(data))), [r[6] for r in data], label="true vx")
axs[2].legend()
axs[2].set_title('X Velocity')
axs[2].set_xlabel('Time')
axs[2].set_ylabel('Velocity (m/s)')

axs[3].plot(list(range(0, len(data))), [r[3]
                                        for r in data], label="estimated vy")
axs[3].plot(list(range(0, len(data))), [r[7] for r in data], label="true vy")
axs[3].legend()
axs[3].set_title('Y Velocity')
axs[3].set_xlabel('Time')
axs[3].set_ylabel('Velocity (m/s)')

plt.show()
