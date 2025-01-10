#!/usr/bin/env python3
from textual.app import App, ComposeResult
from textual import on
from textual.containers import Horizontal, Vertical, Horizontal
from textual.widgets import Footer, Header, Select, Log, Button, Static
from textual.events import Key
import asyncio
import threading


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
        self.gait_sequencer = "Simple"
        self.gait_sequencer_gait="STATIC_WALK"


        self.spinning = True
        self.spinner = threading.Thread(target=self.spin)
        self.spinner.start()

    def spin(self):
        print("start spinning")
        while self.spinning:
            rclpy.spin_once(self, timeout_sec=0.1)

    def stop(self):
        self.spinning = False
        self.spinner.join()

    def send_walk_dir_cmd(self):
        print("send_walk_dir_cmd...")
        msg = QuadControlTarget()
        msg.body_x_dot = self.body_x_dot
        msg.body_y_dot = self.body_y_dot
        msg.world_z = 0.3 #self.world_z
        msg.hybrid_theta_dot = 0.0
        msg.roll = 0.0
        msg.pitch = 0.0
        self.publisher_.publish(msg)

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
        # self.get_logger().info("send param update ...")

        self.send_update_gait(
            NEW_GAIT=self.gait_sequencer_gait, 
            NEW_GAIT_SEQUENCER=self.gait_sequencer
        )
        #param_req = SetParameters.Request()
        #param_req.parameters = [
        #            Parameter(name='simple_gait_sequencer.gait', #value=self.gait_sequencer_gait).to_parameter_msg()]
        #future = self.param_client.call_async(param_req)

        #rclpy.spin_until_future_complete(self, future)
        # print("send control target")
        # self.get_logger().info("send param update done")
        self.publisher_.publish(msg)



    def send_update_gait(self, NEW_GAIT="", NEW_GAIT_SEQUENCER="", DELTA_SWING_HEIGHT=0):
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
        

        #if (NEW_GAIT_SEQUENCER not in {"", self.gait_sequencer}):
        if NEW_GAIT_SEQUENCER != "":
            self.gait_sequencer = NEW_GAIT_SEQUENCER
            param_req = SetParameters.Request()
            param_req.parameters = [
                Parameter(name="gait_sequencer", value=self.gait_sequencer).to_parameter_msg(),
                Parameter(name='adaptive_gait_sequencer.gait.switch_offsets', value=True).to_parameter_msg(),
            ]
            self.param_client.call_async(param_req)


        if NEW_GAIT != "":
            self.gait_sequencer_gait = NEW_GAIT
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
                    # self.gait_sequencer = "Simple"
                    assert self.gait_sequencer == "Simple"
                    assert False
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


class App(App):
    """A Textual app to manage stopwatches."""

    BINDINGS = [
        ("r", "reset_cmd()", "Reset cmd")
    ]
    TITLE = "Go2 cli controller gui"

    def __init__(self, args):
        super().__init__()
        self.body_x_dot = 0.0
        self.body_y_dot = 0.0

        rclpy.init(args=args)
        self.joy2ee_node = Joy2Target()
        #rclpy.spin(joy2ee_node)


    def ros_task():
        while True:
            rclpy.spin(self.joy2ee_node)


    def compose(self) -> ComposeResult:
        """Create child widgets for the app."""
        yield Header()
        yield Horizontal(
            Vertical(
                Select(((o, o) for o in [
                        "STAND",
                        "STATIC_WALK",
                        "WALKING_TROT",
                        "PACE",
                        "BOUND",
                        "PRONK",
                    ]), 
                    prompt="gait_sequencer.gait", value="STAND", name="gait_sequencer.gait", id="gait_sequencer_gait"
                ),
                Select(((o, o) for o in [
                        "Simple",
                        "Adaptive",
                    ]), 
                    prompt="gait_sequencer", value="Simple", name="gait_sequencer", id="gait_sequencer"
                ),
                Button("Reset Cmd Velocity", name="reset_cmd", variant="primary"),
            ),
            Log(auto_scroll=True)#, max_lines=5)
        )
        yield Footer()
        #self.ros_task = asyncio.create_task(asyncio.to_thread(self.ros_task))

    def on_ready(self) -> None:
        from threading import Timer
        self.joy2ee_node.send_update_gait(NEW_GAIT="STAND", NEW_GAIT_SEQUENCER="Simple")
        Timer(0.1, self.reset_cmd).start()

    def on_key(self, event: Key):
        # self.title = event.key
        dt = 0.1
        match event.key:
            case "w":
                self.joy2ee_node.body_y_dot += dt
            case "s":
                self.joy2ee_node.body_y_dot -= dt
            case "a":
                self.joy2ee_node.body_x_dot -= dt
            case "d":
                self.joy2ee_node.body_x_dot += dt
            case "q":
                self.exit()
            case "r":
                self.reset_cmd()
            case _:
                return
        self.log_msg(f"body_x_dot: {self.joy2ee_node.body_x_dot}, body_y_dot: {self.joy2ee_node.body_y_dot}")
        self.joy2ee_node.send_walk_dir_cmd()

    @on(Select.Changed)
    def select_changed(self, event: Select.Changed) -> None:
        self.title = str(event.value)
        value = str(event.value)

        changed = False
        match event.control.name:
            case "gait_sequencer":
                if self.joy2ee_node.gait_sequencer != value:
                    NEW_GAIT = ""
                    if value == "Adaptive":
                        NEW_GAIT = "STATIC_WALK"
                    else:
                        NEW_GAIT = "STAND"
                    self.query_one("#gait_sequencer_gait", Select).value = NEW_GAIT
                    self.joy2ee_node.send_update_gait(NEW_GAIT=NEW_GAIT,  NEW_GAIT_SEQUENCER=value)
                    #self.joy2ee_node.gait_sequencer = str(event.value)
                    changed = True

            case "gait_sequencer.gait":
                if self.joy2ee_node.gait_sequencer_gait != value:
                    #self.joy2ee_node.gait_sequencer_gait = str(event.value)
                    NEW_GAIT_SEQUENCER = ""
                    if value == "STAND":
                        NEW_GAIT_SEQUENCER = "Simple"
                        self.query_one("#gait_sequencer", Select).value = NEW_GAIT_SEQUENCER
                    self.joy2ee_node.send_update_gait(NEW_GAIT=value, NEW_GAIT_SEQUENCER=NEW_GAIT_SEQUENCER)
                    changed = True

        if changed:
            self.log_msg(f"gait_sequencer: {self.joy2ee_node.gait_sequencer}, gait_sequencer.gait: {self.joy2ee_node.gait_sequencer_gait}")
            self.joy2ee_node.send_walk_dir_cmd()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        match event.button.name:
            case "reset_cmd":
                self.reset_cmd()
    
    def reset_cmd(self):
        self.joy2ee_node.body_y_dot = 0.0
        self.joy2ee_node.body_x_dot = 0.0
        self.joy2ee_node.send_walk_dir_cmd()
        self.log_msg("reset cmd")


    def log_msg(self, msg):
        log = self.query_one(Log)
        log.write_line(msg)
        print(msg)

    def send_update(self):
        log = self.query_one(Log)
        log.write_line(f"body_x_dot: {self.joy2ee_node.body_x_dot}, body_y_dot: {self.joy2ee_node.body_y_dot}")
        self.joy2ee_node.send_update()


    async def on_unmount(self) -> None:
        self.joy2ee_node.stop()


def main(args=None):
    app = App(args=args)
    app.run()


if __name__ == "__main__":
    main()
