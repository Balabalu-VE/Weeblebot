import numpy as np
import matplotlib.pyplot as plt

# Load the data
data = np.load("vel_log.npz")

# Extract the relevant variables
t = data["t"]
ukf = data["des_vel"]
acc = data["act_vel"]
vel_error = data["vel_error"]

# Shift time so it starts from 0
t = t - t[0]

# Plotting velocity error
plt.figure()
plt.plot(t, vel_error, label="Velocity Error", color='red')
plt.xlabel("Time (s)")
plt.ylabel("Velocity Error (m/s)")
plt.legend()
plt.title("Velocity Error over Time")
plt.grid(True)
plt.tight_layout()
plt.savefig("vel_error_plot.jpg", dpi=200)  # Save the plot to JPEG
