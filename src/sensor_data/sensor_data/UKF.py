import rclpy
from rclpy.node import Node

import numpy as np
from messages.msg import IMUdata
from std_msgs.msg import Float32
import math

class AngleUKF(Node):
    def __init__(self):
        super().__init__("angle_ukf")

        # --- State definition ---
        # x = [ angle, wheel_speed, gyro_bias ]
        self.n = 3
        self.x = np.zeros(self.n)

        # Covariance
        self.P = np.eye(self.n) * 0.1

        # Process noise (TUNE THIS)
        self.Q = np.diag([0.0001,    # angle random walk (usually small)
                          0.001,     # wheel_speed uncertainty
                          0.00001])  # gyro bias random walk

        # Measurement noise (accelerometer angle variance)
        self.R = np.array([[0.01]])  # TUNE THIS (rad^2)

        # UKF scaling parameters
        self.alpha = 1e-3
        self.beta = 2.0
        self.kappa = 0.0

        # ROS subscriptions
        self.sub_angle_meas = self.create_subscription(
            IMUdata,
            "imu_data",          # <--- Replace with your topic
            self.meas_callback,
            10
        )

        # Output filtered angle
        self.pub_angle = self.create_publisher(Float32, "/ukf_angle", 10)

        self.last_time = self.get_clock().now()




    # -----------------------------------------------------------
    # -----------   UKF Core Functions   ------------------------
    # -----------------------------------------------------------

    def sigma_points(self, x, P):
        n = self.n
        lam = self.alpha**2 * (n + self.kappa) - n

        sigma = np.zeros((2*n + 1, n))
        U = np.linalg.cholesky((n + lam) * P)

        sigma[0] = x
        for i in range(n):
            sigma[i + 1]     = x + U[i]
            sigma[n + i + 1] = x - U[i]

        return sigma, lam

    def predict(self, dt, gyro):
        sigma, lam = self.sigma_points(self.x, self.P)

        # UKF weights
        n = self.n
        Wm = np.full(2*n + 1, 1/(2*(n + lam)))
        Wc = np.full(2*n + 1, 1/(2*(n + lam)))
        Wm[0] = lam / (n + lam)
        Wc[0] = lam / (n + lam) + (1 - self.alpha**2 + self.beta)

        # ----- PREDICT SIGMA POINTS USING YOUR DYNAMICS -----
        X_pred = np.zeros_like(sigma)
        for i, s in enumerate(sigma):
            angle   = s[0]
            omega_wheel   = s[1]
            bias    = s[2]

            # ---------------------------------------------------------
            # IMPORTANT: THIS IS WHERE YOU INSERT YOUR MODEL
            # ---------------------------------------------------------
            # Gyro reading from your IMU (passed into predict)
            omega_gyro = gyro  # or msg.gyro_x if you pass it in

            if(False):
                g = 9.81
                L = 1.0414
                r = 5.5 * .0254 #in meters

                # simple integration:
                new_angle = math.pi/2 - math.acos((omega_wheel*r)^2 / (2*g*L))
            elif(True):
                new_angle = angle + (omega_gyro - bias) * dt
            elif(False):
                # Predict new angle using gyro
                new_angle = angle + ((omega_gyro - bias) * dt)/2 + (math.pi/2 - math.acos((omega_wheel*r)^2 / (2*g*L)))/2
            new_omega = omega_wheel
            new_bias = bias  # random walk handled in Q

            X_pred[i] = np.array([new_angle, new_omega, new_bias])

        # Mean of prediction
        x_pred = np.sum(Wm[:, None] * X_pred, axis=0)

        # Covariance of prediction
        P_pred = self.Q.copy()
        for i in range(2*n + 1):
            dx = (X_pred[i] - x_pred)
            P_pred += Wc[i] * np.outer(dx, dx)

        self.x = x_pred
        self.P = P_pred

    def update(self, angle_meas):
        sigma, lam = self.sigma_points(self.x, self.P)

        n = self.n
        Wm = np.full(2*n + 1, 1/(2*(n + lam)))
        Wc = np.full(2*n + 1, 1/(2*(n + lam)))
        Wm[0] = lam / (n + lam)
        Wc[0] = lam / (n + lam) + (1 - self.alpha**2 + self.beta)

        # ----- MEASUREMENT MODEL -----
        # z = angle
        Z = sigma[:, 0:1]   # first state element

        z_pred = np.sum(Wm[:, None] * Z, axis=0)

        # Innovation covariance
        S = self.R.copy()
        for i in range(2*n + 1):
            dz = Z[i] - z_pred
            S += Wc[i] * dz @ dz.T

        # Cross covariance
        Cxz = np.zeros((self.n, 1))
        for i in range(2*n + 1):
            dx = sigma[i] - self.x
            dz = Z[i] - z_pred
            Cxz += Wc[i] * dx[:, None] @ dz.T

        # Kalman gain
        K = Cxz @ np.linalg.inv(S)

        # Update state
        y = angle_meas - z_pred
        self.x = self.x + (K @ y).flatten()

        # Update covariance
        self.P = self.P - K @ S @ K.T

    # -----------------------------------------------------------
    # -----------   ROS CALLBACK: Measurement Update   ----------
    # -----------------------------------------------------------
    def meas_callback(self, msg):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # 1. Predict using dynamics + gyro
        self.predict(dt, msg.gyro_y)

        # 2. Update using accelerometer angle
        acc_x = msg.acc_x
        acc_y = msg.acc_y
        acc_z = msg.acc_z
        pitch_acc = math.atan2(acc_x, acc_z)
        self.update(pitch_acc)

        # Publish filtered angle
        out = Float32()
        out.data = float(self.x[0])
        self.pub_angle.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = AngleUKF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
