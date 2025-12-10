import numpy as np
import matplotlib.pyplot as plt

data = np.load("ukf_log.npz")

t = data["t"]
ukf = data["ukf"]
acc = data["acc"]
acc_comp = data["acc_comp"]

plt.figure()
plt.plot(t, ukf, label="UKF Angle")
plt.plot(t, acc, label="Accelerometer Angle", linestyle='--')
#plt.plot(t, acc_comp, label="IMU with Gyro Compensation", linestyle=':')
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.legend()
plt.title("UKF vs Accelerometer Angle")
plt.grid(True)
plt.tight_layout()
plt.savefig("ukf_plot.jpg", dpi=200)   # <-- Save to JPEG
