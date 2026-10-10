import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class MotorModelNode(Node):
    def __init__(self):
        super().__init__('motor_model_node')
        
        # Параметры ДПТ
        self.K_m = 0.05
        self.R = 2.0
        self.L = 0.01
        self.K_e = 0.05
        self.J_motor = 0.001
        self.J_reducer = 0.001
        self.b_motor = 0.001
        self.i_gear = 10.0
        
        self.tau_e = self.L / self.R
        self.dt = 0.001  # 1000 Гц - как в trajectory_generator
        
        # Параметры робота
        self.L1 = 0.35
        self.L2 = 0.25
        self.rho = 2800.0
        self.d = 0.05
        self.m1 = self.rho * np.pi * (self.d/2)**2 * self.L1
        self.m2 = self.rho * np.pi * (self.d/2)**2 * self.L2
        self.l1 = self.L1 / 2
        self.l2 = self.L2 / 2
        self.I1zz = (1/12) * self.m1 * (3*(self.d/2)**2 + self.L1**2)
        self.I2zz = (1/12) * self.m2 * (3*(self.d/2)**2 + self.L2**2)
        self.b1 = 0.1
        self.b2 = 0.1
        
        # Состояния модели двигателя
        self.M_dv1 = 0.0
        self.M_dv2 = 0.0
        self.omega1 = 0.0
        self.omega2 = 0.0
        
        self.U1 = 0.0
        self.U2 = 0.0
        
        # Состояния робота
        self.q1 = 0.0
        self.q2 = 0.0
        self.q1_dot = 0.0
        self.q2_dot = 0.0
        self.q1_ddot = 0.0
        self.q2_ddot = 0.0
        self.q1_dot_prev = 0.0
        self.q2_dot_prev = 0.0
        self.joint_state_received = False
        
        # Для сглаживания ускорений
        self.q1_ddot_buffer = []
        self.q2_ddot_buffer = []
        self.buffer_size = 10

        # ROS 2 интерфейсы
        self.voltage_sub = self.create_subscription(
            Float64MultiArray, '/voltage_commands', self.voltage_callback, 10)
        
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.effort_pub = self.create_publisher(
            Float64MultiArray, '/effort_controller/commands', 10)
        
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info('Motor model node started (dt=0.001, 1000 Гц)')
    
    def voltage_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 2:
            self.U1 = msg.data[0]
            self.U2 = msg.data[1]

    def joint_state_callback(self, msg: JointState):
        if len(msg.position) >= 2:
            self.q1 = msg.position[0]
            self.q2 = msg.position[1]
        
        if len(msg.velocity) >= 2:
            self.q1_dot = msg.velocity[0]
            self.q2_dot = msg.velocity[1]
        
        q1_ddot_new = (self.q1_dot - self.q1_dot_prev) / self.dt
        q2_ddot_new = (self.q2_dot - self.q2_dot_prev) / self.dt
        
        self.q1_ddot_buffer.append(q1_ddot_new)
        self.q2_ddot_buffer.append(q2_ddot_new)
        if len(self.q1_ddot_buffer) > self.buffer_size:
            self.q1_ddot_buffer.pop(0)
            self.q2_ddot_buffer.pop(0)
        
        self.q1_ddot = np.mean(self.q1_ddot_buffer)
        self.q2_ddot = np.mean(self.q2_ddot_buffer)
        
        self.q1_dot_prev = self.q1_dot
        self.q2_dot_prev = self.q2_dot
        self.joint_state_received = True

    def compute_load_torque_and_inertia(self, q1, q2, q1_dot, q2_dot, q1_ddot, q2_ddot):
        M11 = self.I1zz + self.I2zz + self.m1*self.l1**2 + self.m2*(self.L1**2 + self.l2**2 + 2*self.L1*self.l2*np.cos(q2))
        M12 = self.I2zz + self.m2*(self.l2**2 + self.L1*self.l2*np.cos(q2))
        M22 = self.I2zz + self.m2*self.l2**2
        
        c11 = -self.m2*self.L1*self.l2*np.sin(q2) * q2_dot
        c12 = -self.m2*self.L1*self.l2*np.sin(q2) * (q1_dot + q2_dot)
        c21 = self.m2*self.L1*self.l2*np.sin(q2) * q1_dot
        c22 = 0.0
        
        tau1 = M12*q2_ddot + c11*q1_dot + c12*q2_dot + self.b1*q1_dot
        tau2 = M12*q1_ddot + c21*q1_dot + c22*q2_dot + self.b2*q2_dot
        
        J_links1 = M11 / (self.i_gear**2)
        J_links2 = M22 / (self.i_gear**2)
        
        return tau1, tau2, J_links1, J_links2

    def control_loop(self):
        try:
            if not self.joint_state_received:
                return

            M_n_joint1, M_n_joint2, J_links1, J_links2 = self.compute_load_torque_and_inertia(
                self.q1, self.q2, self.q1_dot, self.q2_dot, self.q1_ddot, self.q2_ddot)
            
            M_n_motor1 = M_n_joint1 / self.i_gear
            M_n_motor2 = M_n_joint2 / self.i_gear
            
            J_total1 = max(self.J_motor + self.J_reducer + J_links1, 0.001)
            J_total2 = max(self.J_motor + self.J_reducer + J_links2, 0.001)

            omega_motor_real1 = self.q1_dot * self.i_gear
            omega_motor_real2 = self.q2_dot * self.i_gear

            # --- ДВИГАТЕЛЬ 1 ---
                        # --- ДВИГАТЕЛЬ 1 ---
            U_eff1 = self.U1 - self.K_e * omega_motor_real1
            dM_dv1 = ((self.K_m / self.R) * U_eff1 - self.M_dv1) / self.tau_e
            self.M_dv1 += dM_dv1 * self.dt
            
            M_sum1 = self.M_dv1 - M_n_motor1
            domega1 = M_sum1 / J_total1
            self.omega1 += domega1 * self.dt
            
            # ИСПРАВЛЕНИЕ: На редуктор передается полный электромагнитный момент двигателя
            M_out_motor1 = self.M_dv1  # Было: M_sum1

            # --- ДВИГАТЕЛЬ 2 ---
            U_eff2 = self.U2 - self.K_e * omega_motor_real2
            dM_dv2 = ((self.K_m / self.R) * U_eff2 - self.M_dv2) / self.tau_e
            self.M_dv2 += dM_dv2 * self.dt
            
            M_sum2 = self.M_dv2 - M_n_motor2
            domega2 = M_sum2 / J_total2
            self.omega2 += domega2 * self.dt
            
            # ИСПРАВЛЕНИЕ: Аналогично для второго двигателя
            M_out_motor2 = self.M_dv2  # Было: M_sum2

            # Публикация корректного момента на суставы
            cmd = Float64MultiArray()
            cmd.data = [float(M_out_motor1 * self.i_gear), float(M_out_motor2 * self.i_gear)]
            self.effort_pub.publish(cmd)
            
        except Exception as e:
            self.get_logger().error(f'Ошибка в control_loop: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorModelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        zero_cmd = Float64MultiArray()
        zero_cmd.data = [0.0, 0.0]
        node.effort_pub.publish(zero_cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()