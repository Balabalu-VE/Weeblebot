import rclpy
from rclpy.node import Node

import numpy as np
from messages.msg import IMUdata, Encdata
from std_msgs.msg import Float32
import math

class AngleUKF(Node):
    def __init__(self):
        super().__init__("angle_ukf")

        # --- State definition ---
        # x = [ angle, gyro_bias ]
        self.n = 2
        self.x = np.zeros(self.n)

        # Covariance
        self.P = np.eye(self.n) * 0.1

        # Process noise (TUNE THIS)
        self.Q = np.diag([0.01,    # angle random walk (usually small)
                          0.00001])  # gyro bias random walk

        # Measurement noise (accelerometer angle variance)
        self.R = np.array([[0.015]])  # TUNE THIS (rad^2)

        # UKF scaling parameters
        self.alpha = 0.7
        self.beta = 2.0
        self.kappa = 0.0

        # ROS subscriptions
        self.sub_angle_meas = self.create_subscription(
            IMUdata,
            "imu_data",          # <--- Replace with your topic
            self.meas_callback,
            10
        )

        # ROS subscriptions
        self.sub_encp = self.create_subscription(
            Encdata,
            "enc_velocity",          # <--- Replace with your topic
            self.get_enc_callback,
            10
        )
        self.omega_wheel = 0.0

        # Output filtered angle
        self.pub_angle = self.create_publisher(Float32, "ukf_angle", 10)

        self.last_time = self.get_clock().now()

        self.offset = 0.0

        # ROS subscriptions
        self.sub_encp = self.create_subscription(
            Float32,
            "angle_IMU_w_Gyro",          # <--- Replace with your topic
            self.get_IMU_to_angle_callback,
            10
        )
        self.IMU_to_angle = 0.0

        self.logged_ukf = []
        self.logged_acc = []
        self.logged_acc_comp = []
        self.logged_time = []

    def get_IMU_to_angle_callback(self, msg):
        self.IMU_to_angle = msg.data

    def get_enc_callback(self, msg):
        # Wheel angular velocity from encoders (rad/s)
        self.omega_wheel = (msg.left_enc)

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
            sigma[i + 1]     = x + U[:, i]     # COLUMN i
            sigma[n + i + 1] = x - U[:, i]

        return sigma, lam

    def predict(self, dt, gyro, omega_wheel):
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
            bias    = s[1]

            # ---------------------------------------------------------
            # IMPORTANT: THIS IS WHERE YOU INSERT YOUR MODEL
            # ---------------------------------------------------------
            # Gyro reading from your IMU (passed into predict)
            omega_gyro = gyro  # or msg.gyro_x if you pass it in
            # self.get_logger().info('Gyro reading: "%s"' % omega_gyro)
            # self.get_logger().info('Dt: "%s"' % dt)
            # self.get_logger().info('Gyro*Dt: "%s"' % float(omega_gyro*dt))
            # self.get_logger().info('(omega_gyro - bias) * dt: "%s"' % float((omega_gyro - bias) * dt))
            # self.get_logger().info('bias: "%s"' % float(bias))

            if(False):
                g = 9.81
                L = 1.0414
                r = 5.5 * .0254 #in meters

                # simple integration:
                new_angle = math.pi/2 - math.acos((omega_wheel*r)*(omega_wheel*r) / (2*g*L)) + self.offset #Offset of IMU
            elif(False):
                new_angle = angle + (omega_gyro-bias) * dt
            elif(True):
                g = 9.81
                L = 1.0414
                r = 5.5 * .0254 #in meters
                # Predict new angle using gyro
                new_angle_1 = math.pi/2 - math.acos((omega_wheel*r)*(omega_wheel*r) / (2*g*L)) + self.offset #Offset of IMU
                new_angle_2 = angle + (omega_gyro-bias) * dt
                new_angle = new_angle_1*0.5 + new_angle_2*0.5
            #self.get_logger().info('New Angle: "%s"' % new_angle)
            new_bias = bias + np.random.normal(0, np.sqrt(self.Q[1,1]))
            X_pred[i] = np.array([new_angle, new_bias])

        # Mean of prediction
        x_pred = np.sum(Wm[:, None] * X_pred, axis=0)

        # Covariance of prediction
        P_pred = self.Q.copy()
        for i in range(2*n + 1):
            dx = (X_pred[i] - x_pred)
            P_pred += Wc[i] * np.outer(dx, dx)

        self.x = x_pred
        self.P = 0.5 * (P_pred + P_pred.T)  # ensure symmetry
        eps = 1e-6
        self.P += eps * np.eye(self.n)

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
            dz = (Z[i] - z_pred).reshape(1, -1)  # shape (1,1)
            Cxz += Wc[i] * dx[:, None] @ dz      # dx[:, None] = (3,1), dz = (1,1) -> result (3,1)


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

        if(self.offset == 0.0):
            acc_x = msg.acc_x
            acc_y = msg.acc_y
            acc_z = msg.acc_z
            self.offset = math.atan2(acc_x, acc_z)
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # 1. Predict using dynamics + gyro
        self.predict(dt,  math.radians(msg.gyro_y), self.omega_wheel)

        # 2. Update using accelerometer angle
        acc_x = msg.acc_x
        acc_y = msg.acc_y
        acc_z = msg.acc_z
        pitch_acc = math.atan2(acc_x, acc_z)
        self.update(pitch_acc)

       
        

        # Publish filtered angle
        out = Float32()
        out.data = float(self.x[0] - self.offset)  # 1.7 `is offset to make angle zero when upright`
         
        # -------------- SAVE TO BUFFER --------------
        self.logged_ukf.append(out.data*(180/math.pi))  # 1.7 `is offset to make angle zero when upright`
        self.logged_acc.append((pitch_acc - self.offset)*(180/math.pi))
        self.logged_acc_comp.append((-self.IMU_to_angle + self.offset)*(180/math.pi))
        self.logged_time.append(self.get_clock().now().nanoseconds * 1e-9)

        # self.get_logger().info(
        #     f'UKF Angle: "{out.data * (180/math.pi)}", Raw pitch from accel: "{(pitch_acc - self.offset)*(180/math.pi)}"')
        self.pub_angle.publish(out)

    def destroy_node(self):
        import numpy as np
        self.get_logger().info(
            f'Destroying node and saving log...')

        #1.7 `is offset to make angle zero when upright`
        np.savez("ukf_log.npz",
                    ukf=np.array(self.logged_ukf),
                    acc=np.array(self.logged_acc),
                    acc_comp=np.array(self.logged_acc_comp),
                    t=np.array(self.logged_time))

        self.get_logger().info("Saved log to ukf_log.npz")
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = AngleUKF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()
