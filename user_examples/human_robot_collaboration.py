import os
import sys
import carb
import math
import numpy as np
from numpy import cos, sin, pi, linalg 

from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.sensor import Camera
from omni.isaac.core.robots import Robot
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.wheeled_robots.robots.holonomic_robot_usd_setup import HolonomicRobotUsdSetup
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers import WheelBasePoseController
from omni.isaac.wheeled_robots.controllers.holonomic_controller import HolonomicController
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
import omni.isaac.core.utils.numpy.rotations as rot_utils

# Dynamically define paths for Isaac Sim and custom assets
home_dir = os.path.expanduser("~")  # User's home directory
isaacsim_path = os.path.join(home_dir, "isaacsim/exts/omni.isaac.examples/omni/isaac/examples/user_examples")
asset_dir = os.path.join(isaacsim_path, "asset")

# Append to Python path for module imports
if isaacsim_path not in sys.path:
    sys.path.append(isaacsim_path)

from crazyflie import Crazyflie, CrazyflieView, hat, vee, deriv_unit_vector, ensure_SO3, IntegralErrorVec3
from crazyflie import quaternion_to_rotation_matrix, quat_rotate_inverse, q_to_R

# Define paths for all necessary simulation assets (warehouse, robots, fire, etc.)
class Assets:
    def __init__(self):        
        self.background_asset_path = os.path.join(asset_dir, "warehouse_wall.usd")
        self.jetbot_asset_path = os.path.join(asset_dir, "jetbot_largex4.usd")
        self.jackal_asset_path = os.path.join(asset_dir, "jackal.usd")
        self.fire_asset_path = os.path.join(asset_dir, "fire.usd")
        self.smoke_asset_path = os.path.join(asset_dir, "smoke.usd")
        self.fire_source_asset_path = os.path.join(asset_dir, "fire_source.usd")
        self.yellow_safety_helmet_man_asset_path = os.path.join(asset_dir, "yellow_safety_helmet_man.usd")
        self.red_safety_helmet_man_asset_path = os.path.join(asset_dir, "red_safety_helmet_man.usd")

# Main class handling setup, robot definitions, controllers, and physics interactions
class HumanRobotCollaboration(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        # Physics and timing setup
        self.g = 9.81  # Standard gravity
        self.dt = 1/60  # Simulation time step; 60 Hz by default

        # UGV (Jetbot) properties
        self.scale_jetbot = 4  # Scale factor for Jetbot's model size
 
        # UAV (Crazyflie) properties and physical parameters
        self._num_envs = 1  # Number of UAVs
        self.crazyflie_x0 = np.array([-7., 13., 2.])  # Crazyflie's init position
        self.scale_uav = 6  # UAV scale factor
        self.m_uav = (0.025 + 4*0.0008)*self.scale_uav  # UAV mass
        self.d = 0.0438*self.scale_uav  # Rotor-arm distance
        self.J = np.diag([1.4e-5, 1.4e-5, 2.17e-5])*self.scale_uav  # Inertia matrix of quad, [kg m2]
        self.c_tf = 0.0135  # Torque-to-thrust coefficients
        self.c_tw = 2.25  # Thrust-to-weight coefficients
        self.hover_force = self.m_uav * self.g / 4.0  # Thrust magnitude of each motor, [N]
        self.min_force = -0.005  # Minimum thrust of each motor, [N]
        self.max_force = self.c_tw * self.hover_force  # Maximum thrust of each motor, [N]
        self.prop_max_rot = 433.3
        self.motor_assymetry = np.array([1.0, 1.0, 1.0, 1.0])
        self.motor_assymetry = self.motor_assymetry * 4.0 / np.sum(self.motor_assymetry) # re-normalizing to sum-up to 4
        self.thrusts_inertial = np.zeros((self._num_envs, 4, 3))
        self.thrust_cmds_damp = np.ones((self._num_envs, 4)) #np.zeros((self._num_envs, 4))
        self.fM = np.zeros((4, 1)) # Force-moment vector
        fM_to_forces = np.array([
            [1.0, 1.0, 1.0, 1.0],
            [self.d*cos(pi/4), self.d*cos(pi/4), -self.d*cos(pi/4), -self.d*cos(pi/4)],
            [self.d*sin(pi/4), -self.d*sin(pi/4), -self.d*sin(pi/4), self.d*sin(pi/4)],
            [-self.c_tf, self.c_tf, -self.c_tf, self.c_tf]
        ]) # Conversion matrix of force-moment to forces 
        self.fM_to_forces_inv = np.linalg.inv(fM_to_forces)

        # Isaac Sim uses ENU frame, but NED frame is used in FDCL.
        # Note that ENU and the NED here refer to their direction order.
        # ENU: E - axis 1, N - axis 2, U - axis 3
        # NED: N - axis 1, E - axis 2, D - axis 3
        self.R_fw = np.array([
            [1.,  0.,  0.],
            [0., cos(pi),-sin(pi)],
            [0., sin(pi), cos(pi)]
        ]) # Transformation from Isaac-world-fixed frame(ENU) to the FDCL-fixed frame(NED).
        self.R_bl = np.array([
            [1.,  0.,  0.], 
            [0., cos(pi),-sin(pi)],
            [0., sin(pi), cos(pi)]
        ]) # Transformation from Isaac-local-body frame(ENU) to the FDCL-body frame(NED).
 
        # PID control gains
        self.kX = 2.2*np.diag([0.4, 0.4, 1.25]) # position gains
        self.kV = 2.2*np.diag([0.2, 0.2, 0.5]) # velocity gains 
        self.kX[2,2] = 0.4
        self.kR = 0.2*np.diag([1.5, 1.5, 1.2]) # attitude gains 
        self.kW = 0.2*np.diag([0.5, 0.5, 0.3]) # angular velocity gains 
        # Integral control
        self.use_integral = False
        self.kIX = 0.5*np.diag([1.0, 1.0, 1.4])   # Position integral gains
        self.kIR = 0.05*np.diag([1.0, 1.0, 0.7]) # Attitude integral gain
        self.sat_sigma = 3.5
        self.eIX = IntegralErrorVec3() # Position integral error
        self.eIR = IntegralErrorVec3() # Attitude integral error
        self.eIX.set_zero() # Set all integrals to zero
        self.eIR.set_zero()

        # Geometric tracking trajectory
        self.xd, self.vd, self.Wd = np.zeros(3), np.zeros(3), np.zeros(3)
        self.b1d = np.array([1.,0.,0.])  # desired heading direction
        self.xd_2dot, self.xd_3dot, self.xd_4dot = np.zeros(3), np.zeros(3), np.zeros(3)
        self.b1d_dot, self.b1d_2dot = np.zeros(3), np.zeros(3)
        self.trajectory_started = False
        self.t0, self.t, self.t_traj = 0., 0., 0.

        # Waypoints and motion control flags 
        self.crazyflie_done, self.jetbot_done, self.jackal_done = False, False, False
        self.waypoint_crazyflie_index, self.waypoint_jetbot_index, self.waypoint_jackal_index = 0, 0, 0
        # Desired waypoints for crazyflie
        waypoints_crazyflie = np.array([
            np.array([-7., 12., self.crazyflie_x0[2]]),
            np.array([-7., 6., self.crazyflie_x0[2]]),
            np.array([-7., -1., self.crazyflie_x0[2]]),
            np.array([-1., -1., self.crazyflie_x0[2]]),
            # np.array([-1., 2., self.crazyflie_x0[2]]),
            # np.array([-1., 5., self.crazyflie_x0[2]]),
        ])
        self.interpolated_waypoints_crazyflie = self.interpolate_waypoints(waypoints_crazyflie, 
                                                                           num_intermediate_points=100) # smoothed_waypoints
        # Desired waypoints for jetbot
        waypoints_jetbot = np.array([
            np.array([-7., 12., 0.134]),
            np.array([-7., 6., 0.134]),
            np.array([-7., -1., 0.134]),
            np.array([-1., -1., 0.134]), 
            # np.array([-1., 2., 0.134]), 
        ])
        self.interpolated_waypoints_jetbot = self.interpolate_waypoints(waypoints_jetbot, 
                                                                        num_intermediate_points=100) # smoothed_waypoints

        # Desired waypoints for jackal
        waypoints_jackal = np.array([
            np.array([-1.5, 1.5, 0.07]),
            np.array([-1.5, -4.5, 0.07]),
            np.array([-1.5, 7.5, 0.07]),
            np.array([-1., 9.35, 0.07]),
            np.array([0, 9.35, 0.07]), 
            np.array([6.8, 15.8, 0.07]), 
        ])
        self.interpolated_waypoints_jackal = self.interpolate_waypoints(waypoints_jackal, 
                                                                        num_intermediate_points=100) # smoothed_waypoints

        return
    
    # This function is called to setup the assets in the scene for the first time
    # Class variables should not be assigned here, since this function is not called
    # after a hot-reload, its only called to load the world starting from an EMPTY stage
    def setup_scene(self):
        world = self.get_world()

        # Add warehouse background asset in the scene
        assets = Assets()
        add_reference_to_stage(usd_path=assets.background_asset_path, prim_path="/World/Background")
        background_prim = XFormPrim(
            prim_path="/World/Background", 
            position=[0.0, 0.0, 0.0], 
            # orientation=[0.7071, 0, 0, 0.7071]
        )

        # Add jetbot asset in the scene
        # This will create a new XFormPrim and point it to the usd file as a reference
        add_reference_to_stage(usd_path=assets.jetbot_asset_path, prim_path="/World/UGVs/Jetbot")
        jetbot_robot = world.scene.add(
            Robot(
                prim_path="/World/UGVs/Jetbot", # The prim path of the object in the USD stage
                name="jetbot", # The unique name used to retrieve the object from the scene later on
                position=np.array([-7., 13., 0.1]), # Using the current stage units which is in meters by default.
                orientation=np.array([0.66262, 0.0, 0.0, -0.74896]),
                scale=self.scale_jetbot*np.ones(3) # most arguments accept mainly numpy arrays.
                ))
        
        # Add jackal asset in the scene
        add_reference_to_stage(usd_path=assets.jackal_asset_path, prim_path="/World/UGVs/Jackal")
        jackal_robot = world.scene.add(
            Robot(
                prim_path="/World/UGVs/Jackal", # The prim path of the object in the USD stage
                name="jackal", # The unique name used to retrieve the object from the scene later on
                position=np.array([-7.5, 10.5, 0.07]), # Using the current stage units which is in meters by default.
                orientation=np.array([0.66262, 0.0, 0.0, -0.74896]),
                scale=np.ones(3) # most arguments accept mainly numpy arrays.
                ))
        
        # Add crazyflie asset in the scene
        self.default_zero_env_path = "/World/UAVs/UAV_0"
        crazyflie = Crazyflie(
            prim_path=self.default_zero_env_path + "/Crazyflie", # The prim path of the object in the USD stage
            name="crazyflie", # The unique name used to retrieve the object from the scene later on
            position=self.crazyflie_x0, # Using the current stage units which is in meters by default.
            orientation=np.array([0.70711, 0., 0., 0.70711]),
            scale=self.scale_uav*np.ones(3) # most arguments accept mainly numpy arrays.
        )
        self._crazyflie = CrazyflieView(prim_paths_expr="/World/UAVs/.*/Crazyflie", name="crazyflie_view")
        self._world.scene.add(self._crazyflie)
        self._world.scene.add(self._crazyflie.rigid_body)
        for i in range(4):
            self._world.scene.add(self._crazyflie.physics_rotors[i])

        # Add smoke asset in the scene
        add_reference_to_stage(usd_path=assets.smoke_asset_path, prim_path="/World/Flow/Smoke")
        smoke_scene = world.scene.add(
            XFormPrim(
                prim_path="/World/Flow/Smoke", # The prim path of the object in the USD stage
                name="smoke",
                position=[-5, 3., -1.],
                ))
        
        # Add fire and fire source assets in the scene
        add_reference_to_stage(usd_path=assets.fire_asset_path, prim_path="/World/Flow/Fire")
        fire_scene = world.scene.add(
            XFormPrim(
                prim_path="/World/Flow/Fire", # The prim path of the object in the USD stage
                name="fire",
                position=[2.5, 3.6, 0.],
                ))
        add_reference_to_stage(usd_path=assets.fire_source_asset_path, prim_path="/World/Flow/FireSource")
        fire_source_scene = world.scene.add(
            XFormPrim(
                prim_path="/World/Flow/FireSource", # The prim path of the object in the USD stage
                name="fire_source",
                position=[2.5, 3.6, 0.5],
                # scale=[0.1, 0.1, 0.1], 
                # orientation=np.array([1., 1., 0., 0.]),
                ))

        # Add people assets in the scene
        add_reference_to_stage(usd_path=assets.yellow_safety_helmet_man_asset_path, prim_path="/World/People/yellow_safety_helmet_man")
        yellow_safety_helmet_man_scene = world.scene.add(
            XFormPrim(
                prim_path="/World/People/yellow_safety_helmet_man", # The prim path of the object in the USD stage
                name="yellow_safety_helmet_man", # The unique name used to retrieve the object from the scene later on
                position=np.array([0., -5., 0.1]), # Using the current stage units which is in meters by default.
                orientation=np.array([0.66446, -0.66446, -0.24184, 0.24184]),
                ))
        add_reference_to_stage(usd_path=assets.red_safety_helmet_man_asset_path, prim_path="/World/People/red_safety_helmet_man")
        red_safety_helmet_man_scene = world.scene.add(
            XFormPrim(
                prim_path="/World/People/red_safety_helmet_man", # The prim path of the object in the USD stage
                name="red_safety_helmet_man", # The unique name used to retrieve the object from the scene later on
                position=np.array([6.5, 14.0, .1]), # Using the current stage units which is in meters by default.
                orientation=np.array([0.67438, -0.67438, 0.21263, -0.21263]),
                ))
        
        # Add human_camera in the scene
        camera = Camera(
            prim_path="/World/human_camera",
            name="human_camera",
            position=np.array([0.0, 0.0, 1.6]),
            orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, -90]), degrees=True),
        )
        # https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.sensor/docs/index.html
        camera.set_focal_length(1.)


        return
  
    async def setup_post_load(self):
        self._world = self.get_world()
 
        # Load jetbot and define its controller
        self._jetbot = self._world.scene.get_object("jetbot")
        # print("Num of degrees of freedom of UGV: " + str(self._jetbot.num_dof)) # prints 2
        # print("Joint Positions and Velocities of UGV: " + str(self._jetbot.get_joint_positions()) + ","
        #       + str(self._jetbot.get_joint_velocities())) 

        # Initialize jetbot controller after load and the first reset
        self._jetbot_controller  = WheelBasePoseController(name="jetbot_controller",
                                                           open_loop_wheel_controller=
                                                           DifferentialController(name="jetbot_control", 
                                                                                  wheel_radius=0.03*self.scale_jetbot, 
                                                                                  wheel_base=0.1125*self.scale_jetbot),
                                                           is_holonomic=False) 
        
        # Load jackal and define its controller
        self._jackal = self._world.scene.get_object("jackal")
        wheel_radius = [0.098, 0.098, 0.098, 0.098]
        wheel_orientations = [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]]
        wheel_positions = [
            [-0.131, -0.18779, 0.0345],
            [-0.131, 0.18779, 0.0345],
            [0.131, -0.18779, 0.0345],
            [0.131, 0.18779, 0.0345],
        ]
        mecanum_angles = [45, 45, 45, 45]
        self._jackal_controller  = WheelBasePoseController(name="jackal_controller",
                                                           open_loop_wheel_controller=
                                                           HolonomicController(
                                                                           name="jackal_controller",
                                                                           wheel_radius=wheel_radius,
                                                                           wheel_positions=wheel_positions,
                                                                           wheel_orientations=wheel_orientations,
                                                                           mecanum_angles=mecanum_angles,
                                                                           # wheel_axis=wheel_axis,
                                                                           # up_axis=up_axis
                                                                           ),
                                                           is_holonomic=True) 
                
        # Load people
        self._yellow_safety_helmet_man = self._world.scene.get_object("yellow_safety_helmet_man")
        self._red_safety_helmet_man = self._world.scene.get_object("red_safety_helmet_man")

        # Add a physics callback
        # In order to send the actions to apply actions with every physics step executed
        self._world.add_physics_callback("sending_actions", callback_fn=self.physics_step)

        return

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        return

    def world_cleanup(self):
        return

    def physics_step(self, step_size): 
        ''' apply actions to crazyflie '''
        # crazyflie_position, crazyflie_orientation = self.get_crazyflie_obs()
        # self.crazyflie_done = self.fly_to_waypoints(crazyflie_position, crazyflie_orientation, self.interpolated_waypoints_crazyflie)
        # # goal_position = np.array([1.0, -1.0, 1.5])
        # # goal_yaw_angle = 0.
        # # self.fly_to(crazyflie_position, crazyflie_orientation, goal_position, goal_yaw_angle)

        ''' When the UAV finishes scanning '''
        # if self.crazyflie_done == True: 
        #     # ''' apply actions to jetbot ''' 
        #     jetbot_position, jetbot_orientation = self.get_jetbot_obs()
        #     self.jetbot_done = self.jetbot_go_to_waypoints(jetbot_position, jetbot_orientation, self.interpolated_waypoints_jetbot)
        #     # goal_position, _ = self.get_yellow_safety_helmet_man_obs()
        #     # self.go_to(jetbot_position, jetbot_orientation, goal_position)

        ''' apply actions to jackal '''
        jackal_position, jackal_orientation = self.get_jackal_obs()
        self.jackal_done = self.jackal_go_to_waypoints(jackal_position, jackal_orientation, self.interpolated_waypoints_jackal)
        return
       
    def get_jetbot_obs(self):
        jetbot_position, jetbot_orientation = self._jetbot.get_world_pose()
        # print("Jetbot's position is : " + str(jetbot_position))
        # print("Jetbot's orientation is : " + str(jetbot_orientation))
        return jetbot_position, jetbot_orientation 

    def get_jackal_obs(self):
        jackal_position, jackal_orientation = self._jackal.get_world_pose()
        # print("Jackal's position is : " + str(jackal_position))
        # print("Jackal's orientation is : " + str(jackal_orientation))
        return jackal_position, jackal_orientation 
    
    def get_crazyflie_obs(self):
        crazyflie_position, crazyflie_orientation = self._crazyflie.get_world_poses(clone=False)
        # print("Crazyflie's position is : " + str(crazyflie_position))
        # print("Crazyflie's orientation is : " + str(crazyflie_orientation))
        return crazyflie_position[0], crazyflie_orientation[0]

    def get_yellow_safety_helmet_man_obs(self):
        yellow_safety_helmet_man_position, yellow_safety_helmet_man_orientation = self._yellow_safety_helmet_man.get_world_pose()
        # print("yellow_safety_helmet_man's position is : " + str(yellow_safety_helmet_man_position))
        # print("yellow_safety_helmet_man's orientation is : " + str(yellow_safety_helmet_man_orientation))
        return yellow_safety_helmet_man_position, yellow_safety_helmet_man_orientation

    def get_red_safety_helmet_man_obs(self):
        red_safety_helmet_man_position, red_safety_helmet_man_orientation = self._red_safety_helmet_man.get_world_pose()
        # print("red_safety_helmet_man's position is : " + str(red_safety_helmet_man_position))
        # print("red_safety_helmet_man's orientation is : " + str(red_safety_helmet_man_orientation))
        return red_safety_helmet_man_position, red_safety_helmet_man_orientation

    def go_to(self, jetbot_position, jetbot_orientation, goal_position):
        if linalg.norm(goal_position - jetbot_position, 2) < 1.35:
            is_done = True
            self._jetbot.apply_action(self._jetbot_controller.forward(start_position=jetbot_position,
                                                                      start_orientation=jetbot_orientation,
                                                                      goal_position=goal_position,
                                                                      lateral_velocity=0.,
                                                                      yaw_velocity=0.))
        else:
            is_done = False
            self._jetbot.apply_action(self._jetbot_controller.forward(start_position=jetbot_position,
                                                                      start_orientation=jetbot_orientation,
                                                                      goal_position=goal_position,
                                                                      lateral_velocity=0.5,
                                                                      yaw_velocity=0.6))
        return is_done
    
    def fly_to(self, crazyflie_position, crazyflie_orientation, goal_position, goal_yaw_angle):
        # goal_yaw_direction = self.get_current_b1()
        # goal_yaw_angle = 0.*pi/180 # deg
        goal_yaw_direction = np.array([cos(goal_yaw_angle), sin(goal_yaw_angle), 0])
        
        goal_position = np.array([self.crazyflie_x0[0]+5., self.crazyflie_x0[1], self.crazyflie_x0[2]+0.5])
        self.geometric_controller(crazyflie_position, crazyflie_orientation, goal_position, goal_yaw_direction)
        self.apply_forces_and_torques()

        if linalg.norm(goal_position - crazyflie_position, 2) < 0.5:
            is_done = True
        else:
            is_done = False
         
        return is_done

    def jetbot_go_to_waypoints(self, jetbot_position, jetbot_orientation, waypoints_position):
        if self.waypoint_jetbot_index < len(waypoints_position):
            goal_position = waypoints_position[self.waypoint_jetbot_index]
            # print(f"jetbot_position, {jetbot_position}, goal_position: {goal_position}, waypoint_index: {self.waypoint_jetbot_index}")
            self._jetbot.apply_action(self._jetbot_controller.forward(start_position=jetbot_position,
                                                                      start_orientation=jetbot_orientation,
                                                                      goal_position=goal_position,
                                                                      lateral_velocity=0.5,
                                                                      yaw_velocity=0.6))
            if np.linalg.norm(goal_position - jetbot_position) <= 0.5:  # Check if goal reached
                # print("Reached Goal:", goal_position)
                self.waypoint_jetbot_index += 1

        if self.waypoint_jetbot_index == len(waypoints_position):
            is_done = True
            goal_position = waypoints_position[-1]
            # print(f"jetbot_position, {jetbot_position}, goal_position: {goal_position}, waypoint_index: {self.waypoint_jetbot_index}")
            self._jetbot.apply_action(self._jetbot_controller.forward(start_position=jetbot_position,
                                                                      start_orientation=jetbot_orientation,
                                                                      goal_position=goal_position,
                                                                      lateral_velocity=0.,
                                                                      yaw_velocity=0.))
        else:
            is_done = False
        return is_done

    def jackal_go_to_waypoints(self, jackal_position, jackal_orientation, waypoints_position):
        if self.waypoint_jackal_index < len(waypoints_position):
            goal_position = waypoints_position[self.waypoint_jackal_index]
            # print(f"jackal_position, {jackal_position}, goal_position: {goal_position}, waypoint_index: {self.waypoint_jackal_index}")
            self._jackal.apply_action(self._jackal_controller.forward(start_position=jackal_position,
                                                                        start_orientation=jackal_orientation,
                                                                        goal_position=goal_position,
                                                                        lateral_velocity=7.,
                                                                        yaw_velocity=0.4))
            if np.linalg.norm(goal_position - jackal_position) <= 0.5:  # Check if goal reached
                # print("Reached Goal:", goal_position)
                self.waypoint_jackal_index += 1

        if self.waypoint_jackal_index == len(waypoints_position):
            is_done = True
            goal_position = waypoints_position[-1]
            # print(f"jackal_position, {jackal_position}, goal_position: {goal_position}, waypoint_index: {self.waypoint_jackal_index}")
            self._jackal.apply_action(self._jackal_controller.forward(start_position=jackal_position,
                                                                        start_orientation=jackal_orientation,
                                                                        goal_position=goal_position,
                                                                        lateral_velocity=0.,
                                                                        yaw_velocity=0.))
        else:
            is_done = False
        return is_done
    
    def fly_to_waypoints(self, crazyflie_position, crazyflie_orientation, waypoints_position):
        goal_yaw_direction = np.array([0., 1., 0.])#self.get_current_b1()
        
        if self.waypoint_crazyflie_index < len(waypoints_position):
            goal_position = waypoints_position[self.waypoint_crazyflie_index]
            if self.waypoint_crazyflie_index>=200 and self.waypoint_crazyflie_index<300:
                goal_yaw_direction = np.array([-1. ,0. ,0.])
            elif self.waypoint_crazyflie_index>=300:
                goal_yaw_direction = np.array([0., -1., 0.])
            # print(f"crazyflie_position, {crazyflie_position}, goal_position: {goal_position}, goal_yaw_direction: {goal_yaw_direction}, waypoint_index: {self.waypoint_crazyflie_index}")
            
            self.geometric_controller(crazyflie_position, crazyflie_orientation, goal_position, goal_yaw_direction)
            self.apply_forces_and_torques()
            if np.linalg.norm(goal_position - crazyflie_position) <= 1.0:  # Check if goal reached
                # print("Reached Goal:", goal_position)
                self.waypoint_crazyflie_index += 1

        if self.waypoint_crazyflie_index == len(waypoints_position):
            is_done = True
            # self._crazyflie.set_joint_velocities(np.zeros(4))
            goal_position = waypoints_position[-1]
            # print(f"crazyflie_position, {crazyflie_position}, goal_position: {goal_position}, waypoint_index: {self.waypoint_crazyflie_index}")
            self.geometric_controller(crazyflie_position, crazyflie_orientation, goal_position, goal_yaw_direction)
            self.apply_forces_and_torques()
        else:
            is_done = False
        return is_done
    
    def interpolate_waypoints(self, waypoints, num_intermediate_points):
        interpolated_waypoints = []
        for i in range(len(waypoints) - 1):
            for j in range(num_intermediate_points + 1):
                interpolated_point = (waypoints[i] * (num_intermediate_points - j) + waypoints[i + 1] * j) / num_intermediate_points
                interpolated_waypoints.append(interpolated_point)
        return np.array(interpolated_waypoints)

    def geometric_controller(self, crazyflie_position, crazyflie_orientation, goal_position, goal_yaw_direction):
        # States
        x_w = crazyflie_position # Pos in Isaac-world-fixed frame(ENU)
        v_w = self._crazyflie.get_velocities(clone=False)[:, :3][0] # Vel in Isaac-world-fixed frame(ENU) 
        R_wl = ensure_SO3(quaternion_to_rotation_matrix(crazyflie_orientation)) # Isaac-local-body frame(ENU) to Isaac-world-fixed frame(ENU)
        W_l = self._crazyflie.get_velocities(clone=False)[:, 3:][0] # Ang Vel in Isaac-world-local frame(ENU)
        W_l = R_wl.T@W_l
        # Desired commands
        xd_w = goal_position # Goal Pos in Isaac-world-fixed frame(ENU)
        vd_w = self.vd
        xd_2dot_w = self.xd_2dot 
        xd_3dot_w = self.xd_3dot 
        xd_4dot_w = self.xd_4dot 

        b1d_l = goal_yaw_direction #np.array([1., 0., 0.,]) 
        b1d_dot_l = self.b1d_dot  
        b1d_2dot_l = self.b1d_2dot 

        # Isaac-world-fixed frame to FDCL-fixed frame
        x_f = self.R_fw@x_w # Pos in FDCL-fixed frame(NED)
        v_f = self.R_fw@v_w # Vel in FDCL-fixed frame(NED)
        R_fb = self.R_fw@R_wl@self.R_bl.T # Att in FDCL-body frame(NED)
        W_b = self.R_bl@W_l # Ang Vel in FDCL-body frame(NED)

        xd_f = self.R_fw@xd_w # Pos Cmd in FDCL-fixed frame(NED)
        vd_f = self.R_fw@vd_w # Vel Cmd in FDCL-fixed frame(NED)
        xd_2dot_f = self.R_fw@xd_2dot_w # in FDCL-fixed frame(NED)
        xd_3dot_f = self.R_fw@xd_3dot_w # in FDCL-fixed frame(NED)
        xd_4dot_f = self.R_fw@xd_4dot_w # in FDCL-fixed frame(NED)

        # Isaac-world-local frame to FDCL-body frame(
        b1d_b = self.R_bl@b1d_l # Heading Cmd in FDCL-body frame(NED)
        b1d_dot_b = self.R_bl@b1d_dot_l # in FDCL-body frame(NED)
        b1d_2dot_b = self.R_bl@b1d_2dot_l # in FDCL-body frame(NED)
        # print(f"x_f: {x_f}, xd_f: {xd_f}, v_f: {v_f}, R_fb: {R_fb}, W_b: {W_b}")

        e3 = np.array([0., 0., 1.]) 
        R_T = R_fb.T
        hatW = hat(W_b)

        # Position control
        # Translational error functions
        eX = x_f - xd_f # position tracking errors 
        eV = v_f - vd_f # velocity tracking errors 
        
        # Position integral terms
        if self.use_integral:
            self.eIX.integrate(eX + eV, self.dt) 
            self.eIX.error = np.clip(self.eIX.error, -self.sat_sigma, self.sat_sigma)
        else:
            self.eIX.set_zero()

        # Force 'f' along negative b3-axis
        # This term equals to R_fb.e3
        A = - self.kX@eX \
            - self.kV@eV \
            - self.m_uav*self.g*e3 \
            + self.m_uav*xd_2dot_f
        if self.use_integral:
            A -= self.kIX@self.eIX.error

        b3 = R_fb@e3
        b3_dot = R_fb@hatW@e3
        f_total = -A@b3

        # Intermediate terms for rotational errors
        ea = self.g*e3 \
            - f_total/self.m_uav*b3 \
            - xd_2dot_f
        A_dot = - self.kX@eV \
                - self.kV@ea \
                + self.m_uav*xd_3dot_f  

        f_dot = - A_dot@b3 \
                - A@b3_dot
        eb = - f_dot/self.m_uav*b3 \
                - f_total/self.m_uav*b3_dot \
                - xd_3dot_f
        A_2dot = - self.kX@ea \
                    - self.kV@eb \
                    + self.m_uav*xd_4dot_f
        
        b3c, b3c_dot, b3c_2dot = deriv_unit_vector(-A, -A_dot, -A_2dot)

        hat_b1d = hat(b1d_b)
        hat_b1d_dot = hat(b1d_dot_b)
        hat_b2d_dot = hat(b1d_2dot_b)

        A2 = -hat_b1d@b3c
        A2_dot = - hat_b1d_dot@b3c - hat_b1d@b3c_dot
        A2_2dot = - hat_b2d_dot@b3c \
                    - 2.0*hat_b1d_dot@b3c_dot \
                    - hat_b1d@b3c_2dot

        b2c, b2c_dot, b2c_2dot = deriv_unit_vector(A2, A2_dot, A2_2dot)

        hat_b2c = hat(b2c)
        hat_b2c_dot = hat(b2c_dot)
        hat_b2c_2dot = hat(b2c_2dot)

        b1c = hat_b2c@b3c
        b1c_dot = hat_b2c_dot@b3c + hat_b2c@b3c_dot
        b1c_2dot = hat_b2c_2dot@b3c \
                    + 2.0*hat_b2c_dot@b3c_dot \
                    + hat_b2c@b3c_2dot

        Rd = np.vstack((b1c, b2c, b3c)).T
        Rd_dot = np.vstack((b1c_dot, b2c_dot, b3c_dot)).T
        Rd_2dot = np.vstack((b1c_2dot, b2c_2dot, b3c_2dot)).T

        Rd_T = Rd.T
        Wd = vee(Rd_T@Rd_dot)

        hat_Wd = hat(Wd)
        Wd_dot = vee(Rd_T@Rd_2dot - hat_Wd@hat_Wd)
        
        # Attitude control
        RdtR = Rd_T@R_fb
        eR = 0.5*vee(RdtR - RdtR.T) # attitude error vector
        eW = W_b - R_T@Rd@Wd # angular velocity error vector

        # Attitude integral terms
        if self.use_integral:
            self.eIR.integrate(eR + eW, self.dt) 
            self.eIR.error = np.clip(self.eIR.error, -self.sat_sigma, self.sat_sigma)
        else:
            self.eIR.set_zero()

        M = - self.kR@eR \
            - self.kW@eW \
            + hat(R_T@Rd@Wd)@self.J@R_T@Rd@Wd \
            + self.J@R_T@Rd@Wd_dot
        if self.use_integral:
            M -= self.kIR@self.eIR.error

        # Print error terms    
        # print(f"eX: {eX}, eV: {eV}, eR: {eR}, eW: {eW}")

        # Compute the thrust of each motor from the total force and moment
        f_total = np.clip(f_total, -self.c_tw*self.m_uav*self.g, self.c_tw*self.m_uav*self.g)
        self.fM[0] = f_total
        for i in range(3):
            self.fM[i + 1] = M[i]
        f_motor = (self.fM_to_forces_inv@self.fM).flatten()
        f_motor = np.clip(self.fM_to_forces_inv@self.fM, -self.max_force, self.min_force).flatten()
        thrusts_body = f_motor*np.ones((self._num_envs, 4))

        self.f_b = np.array([0, 0, f_total])
        self.f_l = self.R_bl.T@self.f_b
        self.f_w = R_wl@self.f_l 
        self.f_w = R_wl@self.f_b

        self.M_b = M
        self.M_l = self.R_bl.T@self.M_b
        self.M_w = R_wl@self.M_l

    def get_current_b1(self):
        _, crazyflie_orientation = self._crazyflie.get_world_poses(clone=False)
        R_wl = ensure_SO3(quaternion_to_rotation_matrix(crazyflie_orientation[0]))
        R_fb = self.R_fw@R_wl@self.R_bl.T # Att in FDCL-body frame(NED)
        self.R = R_fb

        e1 = np.array([1., 0., 0.])
        b1 = self.R.dot(e1)
        theta = np.arctan2(b1[1], b1[0])
        return np.array([cos(theta), sin(theta), 0.])

    def apply_forces_and_torques(self):
        self.dof_vel = self._crazyflie.get_joint_velocities()

        # spin spinning rotors
        prop_rot = self.thrust_cmds_damp * self.prop_max_rot
        self.dof_vel[:, 0] = prop_rot[:, 0]
        self.dof_vel[:, 1] = -1.0 * prop_rot[:, 1]
        self.dof_vel[:, 2] = prop_rot[:, 2]
        self.dof_vel[:, 3] = -1.0 * prop_rot[:, 3]
        self._crazyflie.set_joint_velocities(self.dof_vel)

        self.M_b = np.array([self.M_b[0], -self.M_b[1], -self.M_b[2]])
        self._crazyflie.rigid_body.apply_forces_and_torques_at_pos(self.f_b, self.M_b, is_global = False)
