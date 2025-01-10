#!/usr/bin/env python3
import numpy as np
import rclpy
from interfaces.msg import QuadControlTarget
from rcl_interfaces.srv import SetParameters
from rclpy import Parameter
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from unitree_go.msg import WirelessController
from pynput import keyboard
from pynput.keyboard import Key


# Simple script, that allows to reconfigure the quads pose using mpc controller with all legs in contact
# new trajectories are generated based on joystick inputs
# for now just send static values of the desired state.


class Joy2Target(Node):
    def __init__(self):
        super().__init__("joy2target")

        # general parameters
        self.declare_parameter("update_freq", 1.0)  # update frequency
        self.declare_parameter("init_robot_height", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("max_robot_height", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("max_acceleration", rclpy.Parameter.Type.DOUBLE)

        self.max_acceleration = self.get_parameter("max_acceleration").get_parameter_value().double_value
        self.v = np.array([0.0,0.0])


        # parameter client
        self.param_client = self.create_client(SetParameters, '/mit_controller_node/set_parameters')#'/mit_controller_node/set_parameters')
        self.emerg_damp_client = self.create_client(Trigger, '/set_emergency_damping_mode')
        self.reset_covar_client = self.create_client(Trigger, '/reset_state_estimation_covariances')
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        # publishers
        self.timer_period = 1 / self.get_parameter("update_freq").get_parameter_value().double_value
        #self.timer = self.create_timer(1, #self.timer_period,
        #                               self.timer_callback)  # INFO: Timer callback is now called in subscriber callback
        self.publisher_ = self.create_publisher(QuadControlTarget, "quad_control_target", 10)

        self.world_z = self.get_parameter("init_robot_height").get_parameter_value().double_value
        self.world_max_z = self.get_parameter("max_robot_height").get_parameter_value().double_value
        self.gait_sequencer = "Simple"


        self.current_swing_height = 0.05
        self.get_logger().info("init")


        self.body_x_dot = 0.0
        self.body_y_dot = 0.0

        listener = keyboard.Listener(
            on_press=self.on_key_press
        )
        listener.start()
        #self.send_update()

    def on_key_press(self, key):
        print("pressed:", key)
        dt = 0.1
        match key:
            case keyboard.Key.up:
                self.body_y_dot += dt
            case keyboard.Key.down:
                self.body_y_dot -= dt
            case keyboard.Key.left:
                self.body_x_dot -= dt
            case keyboard.Key.right:
                self.body_x_dot += dt
        print("body_x_dot: ", self.body_x_dot, ", body_y_dot: ", self.body_y_dot)

    def send_update(self):
        print("send_update...")
        msg = QuadControlTarget()
        msg.body_x_dot = self.body_x_dot
        msg.body_y_dot = self.body_y_dot
        msg.world_z = 0.3 #self.world_z
        msg.hybrid_theta_dot = 0.0
        msg.roll = 0.0
        msg.pitch = 0.0
        # self.publisher_.publish(msg)
        self.get_logger().info("send param update ...")

        #self.send_update_gait(NEW_GAIT="STATIC_WALK", NEW_GAIT_SEQUENCER="Simple")
        param_req = SetParameters.Request()
        param_req.parameters = [
                    Parameter(name='simple_gait_sequencer.gait', value="STATIC_WALK").to_parameter_msg()]
        future = self.param_client.call_async(param_req)

        #rclpy.spin_until_future_complete(self, future)
        print("send control target")
        self.get_logger().info("send param update done")
        self.publisher_.publish(msg)



    def send_update_gait(self, NEW_GAIT="", NEW_GAIT_SEQUENCER="", DELTA_SWING_HEIGHT=0):


        #NEW_GAIT = ""
        #NEW_GAIT_SEQUENCER = ""
        #DELTA_SWING_HEIGHT = 0
        start_rising = False

        """ if (shift_l and shift_r):
            pass
        elif (shift_l and X_rising):
            NEW_GAIT = "PACE"
        elif (shift_l and B_rising):
            NEW_GAIT = "BOUND"
        elif (shift_l and A_rising):
            NEW_GAIT = "PRONK"
        elif (shift_l and Y_rising):
            NEW_GAIT_SEQUENCER = "Simple"
        elif (Right_rising or (alt_r and Up_rising)):
            DELTA_SWING_HEIGHT = 0.0125
        elif (Left_rising or (alt_r and Down_rising)):
            DELTA_SWING_HEIGHT = -0.0125
        elif (A_rising):
            NEW_GAIT = "STAND"
        elif (X_rising):
            NEW_GAIT = "WALKING_TROT"
        elif (B_rising):
            NEW_GAIT = "STATIC_WALK"
        elif (Y_rising):
            NEW_GAIT_SEQUENCER = "Adaptive" """


        if (NEW_GAIT != ""):
            param_req = SetParameters.Request()
            if self.gait_sequencer == "Simple":
                param_req.parameters = [
                    Parameter(name='simple_gait_sequencer.gait', value=NEW_GAIT).to_parameter_msg()]
                # if self.gait_sequencer != "Simple":
                #     self.gait_sequencer = "Simple"
                #     param_req.parameters.append(
                #         Parameter(name="gait_sequencer", value=self.gait_sequencer).to_parameter_msg()
                #     )
            elif self.gait_sequencer == "Adaptive":
                if NEW_GAIT == "STAND":
                    self.gait_sequencer = "Simple"
                    param_req.parameters = [
                        Parameter(name='simple_gait_sequencer.gait', value=NEW_GAIT).to_parameter_msg(),
                        Parameter(name="gait_sequencer", value=self.gait_sequencer).to_parameter_msg(),
                    ]
                elif "TROT" in NEW_GAIT:
                    param_req.parameters = [
                        Parameter(name='adaptive_gait_sequencer.gait.switch_offsets', value=False).to_parameter_msg(),
                        Parameter(name='adaptive_gait_sequencer.gait.phase_offset',
                                  value=[0.0, 0.5, 0.5, 0.0]).to_parameter_msg(),
                    ]
                elif NEW_GAIT == "STATIC_WALK":
                    param_req.parameters = [
                        Parameter(name='adaptive_gait_sequencer.gait.switch_offsets', value=False).to_parameter_msg(),
                        Parameter(name='adaptive_gait_sequencer.gait.phase_offset',
                                  value=[0.0, 0.5, 0.75, 0.25]).to_parameter_msg(),
                    ]
                elif NEW_GAIT == "PACE":
                    param_req.parameters = [
                        Parameter(name='adaptive_gait_sequencer.gait.switch_offsets', value=False).to_parameter_msg(),
                        Parameter(name='adaptive_gait_sequencer.gait.phase_offset',
                                  value=[0.0, 0.5, 0.0, 0.5]).to_parameter_msg(),
                    ]
                elif NEW_GAIT == "BOUND":
                    param_req.parameters = [
                        Parameter(name='adaptive_gait_sequencer.gait.switch_offsets', value=False).to_parameter_msg(),
                        Parameter(name='adaptive_gait_sequencer.gait.phase_offset',
                                  value=[0.0, 0.0, 0.5, 0.5]).to_parameter_msg(),
                    ]
                elif NEW_GAIT == "PRONK":
                    param_req.parameters = [
                        Parameter(name='adaptive_gait_sequencer.gait.switch_offsets', value=False).to_parameter_msg(),
                        Parameter(name='adaptive_gait_sequencer.gait.phase_offset',
                                  value=[0.0, 0.0, 0.0, 0.0]).to_parameter_msg(),
                    ]

            self.param_client.call_async(param_req)
        if (NEW_GAIT_SEQUENCER not in {"", self.gait_sequencer}):
            self.gait_sequencer = NEW_GAIT_SEQUENCER
            param_req = SetParameters.Request()
            param_req.parameters = [
                Parameter(name="gait_sequencer", value=self.gait_sequencer).to_parameter_msg(),
                Parameter(name='adaptive_gait_sequencer.gait.switch_offsets', value=True).to_parameter_msg(),
            ]
            self.param_client.call_async(param_req)

        if DELTA_SWING_HEIGHT:
            self.current_swing_height = np.clip(self.current_swing_height + DELTA_SWING_HEIGHT, 0.0,
                                                self.world_z - 0.05)
            param_req = SetParameters.Request()
            param_req.parameters = [
                Parameter(name="slc_swing_height", value=self.current_swing_height).to_parameter_msg(),
            ]
            self.param_client.call_async(param_req)

        if start_rising:
            req = Trigger.Request()
            self.reset_covar_client.call_async(req)




    def clip_velocity(self, vx, vy):
        if self.max_acceleration == 0.0:
            self.v[0] = vx
            self.v[1] = vy
            return vx, vy
        v_new = np.array([vx,vy], dtype=np.float64)
        v_dif = (v_new - self.v)
        v_dif_norm = np.linalg.norm(v_dif)
        acc = v_dif_norm/self.timer_period
        if (abs(acc) > abs(self.max_acceleration)):
            v_new = self.v + (v_dif * abs(self.max_acceleration/acc))
        self.v = v_new
        return v_new[0], v_new[1]


def main(args=None):
    rclpy.init(args=args)
    joy2ee_node = Joy2Target()
    rclpy.spin(joy2ee_node)
    joy2ee_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
