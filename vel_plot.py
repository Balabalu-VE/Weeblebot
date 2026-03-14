import numpy as np
import matplotlib.pyplot as plt

data = np.load("vel_log.npz")

t = data["t"]
ukf = data["des_vel"]
acc = data["act_vel"]
vel_error = data["vel_error"]

# Shift time so it starts from 0
t = t - t[0]

plt.figure()
plt.plot(t, ukf, label="Desired Velocity")
plt.plot(t, acc, label="Actual Velocity", linestyle='--')
#plt.plot(t, acc_comp, label="IMU with Gyro Compensation", linestyle=':')
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.legend()
plt.title("Desired Velocity vs Actual Velocity")
plt.grid(True)
plt.tight_layout()
plt.savefig("vel_plot.jpg", dpi=200)   # <-- Save to JPEG



