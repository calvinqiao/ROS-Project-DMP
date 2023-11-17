# # Utils
# import math
# import numpy as np
# import copy
# import cPickle as pickle
# import dill
# import scipy.interpolate
# import scipy.io as sio
# import tf
# import os, time, os.path
# import matplotlib.pyplot as plt
# import enum
# from IPython import embed
# from catkin.find_in_workspaces import find_in_workspaces
# import object_tracker.tracker as tk
# import adapy, openravepy, rospy, rospkg
# from prpy.tsr.tsr import *
# from openravepy import *
# import prpy
# from prpy.planning.ik import *
# from ada_teleoperation import DataRecordingUtils as dataRecorder
# from ada_teleoperation.UserInputMapper import *
# from ada_teleoperation.RobotState import *
# from ada_teleoperation.input_handlers import *
# from ada_teleoperation.AdaTeleopHandler import *
# from adapy.tsr import *
# from adapy.controller_client import *
# from adapy.trajectory_client import *
# # from kinova_demo.pose_action_client import *

# class RobotStateDMP(object):
#     def __init__(self, ee_transform, dof_values):
#         self.ee_trans = ee_transform.copy()
#         self.dof_values = copy.deepcopy(dof_values)

#     def update(self, ee_transform, dof_values):
#         self.ee_trans = ee_transform.copy()
#         self.dof_values = copy.deepcopy(dof_values)


# def is_rotation_matrix(R):
#     Rt = np.transpose(R)
#     should_be_identity = np.dot(Rt, R)
#     I = np.identity(3, dtype=R.dtype)
#     n = np.linalg.norm(I - should_be_identity)
#     return n < 1e-6


# def rotation_matrix_to_euler_angles(R):
#     sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
#     singular = sy < 1e-6

#     if not singular:
#         x = math.atan2(R[2, 1], R[2, 2])
#         y = math.atan2(-R[2, 0], sy)
#         z = math.atan2(R[1, 0], R[0, 0])
#     else:
#         x = math.atan2(-R[1, 2], R[1, 1])
#         y = math.atan2(-R[2, 0], sy)
#         z = 0

#     return np.array([x, y, z])


# def euler_angles_to_rotation_matrix(theta):
#     R_x = np.array([[1, 0, 0],
#                     [0, math.cos(theta[0]), -math.sin(theta[0])],
#                     [0, math.sin(theta[0]), math.cos(theta[0])]
#                     ])

#     R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
#                     [0, 1, 0],
#                     [-math.sin(theta[1]), 0, math.cos(theta[1])]
#                     ])

#     R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
#                     [math.sin(theta[2]), math.cos(theta[2]), 0],
#                     [0, 0, 1]
#                     ])

#     R = np.dot(R_z, np.dot(R_y, R_x))

#     return R


# def get_formatted_trajs(dof_traj, ee_traj):
#     user_joint0 = [elem[0] for elem in dof_traj]
#     user_joint1 = [elem[1] for elem in dof_traj]
#     user_joint2 = [elem[2] for elem in dof_traj]
#     user_joint3 = [elem[3] for elem in dof_traj]
#     user_joint4 = [elem[4] for elem in dof_traj]
#     user_joint5 = [elem[5] for elem in dof_traj]
#     user_joint6 = [elem[6] for elem in dof_traj]
#     user_joint7 = [elem[7] for elem in dof_traj]
#     # All eight joint trajectories, change this to six joints later (first six joints)
#     user_dof_trajs = [user_joint0, user_joint1, user_joint2, user_joint3,
#                       user_joint4, user_joint5, user_joint6, user_joint7]

#     # ee_transform matrix to x, y, z, roll, pitch, yaw trajectories
#     user_x_traj = [elem[0, 3] for elem in ee_traj]
#     user_y_traj = [elem[1, 3] for elem in ee_traj]
#     user_z_traj = [elem[2, 3] for elem in ee_traj]

#     rot_matrix_traj = [elem[0:3, 0:3] for elem in ee_traj]
#     euler_angles_traj = [rotation_matrix_to_euler_angles(R) for R in rot_matrix_traj]

#     user_roll_traj = [elem[0] for elem in euler_angles_traj]
#     user_pitch_traj = [elem[1] for elem in euler_angles_traj]
#     user_yaw_traj = [elem[2] for elem in euler_angles_traj]

#     # x, y, z, roll, pitch, yaw trajectories
#     user_trajs = [user_x_traj, user_y_traj, user_z_traj, user_roll_traj, user_pitch_traj, user_yaw_traj]
#     return user_dof_trajs, user_trajs


# def load_data(save_path):
#     saved_file = open(save_path, 'rb')
#     saved_data = pickle.load(saved_file)
#     dof_traj = [elem[1]['robot_dof_values'] for elem in saved_data]
#     ee_traj = [elem[1]['robot_state'].ee_trans for elem in saved_data]
#     saved_file.close()
#     return get_formatted_trajs(dof_traj, ee_traj)


# def add_constraint_boxes(env, robot, handedness='right', name_base="constraint_boxes_", visible=False):
#     # Modifies environment to keep the robot inside a defined space
#     # Does so by adding invisible boxes around the robot, which planners
#     # avoid collisions with

#     # add a box behind the robot
#     box_behind = openravepy.RaveCreateKinBody(env, '')
#     box_behind.SetName(name_base + 'behind')
#     box_behind.InitFromBoxes(np.array([[0., 0., 0., 0.4, 0.1, 1.0]]), False)
#     env.Add(box_behind)
#     T = np.eye(4)
#     T[0:3, 3] = robot.GetTransform()[0:3, 3]
#     T[1, 3] = 0.57
#     if handedness == 'right':
#         T[0, 3] += 0.25
#     else:
#         T[0, 3] -= 0.25
#     box_behind.SetTransform(T)

#     # add a box above so we don't swing that way too high
#     box_above = openravepy.RaveCreateKinBody(env, '')
#     box_above.SetName(name_base + 'above')
#     box_above.InitFromBoxes(np.array([[0., 0., 0., 0.5, 0.5, 0.1]]), visible)
#     env.Add(box_above)
#     T = np.eye(4)
#     T[0:3, 3] = robot.GetTransform()[0:3,3]
#     T[0, 3] += 0.25
#     T[1, 3] -= 0.25
#     T[2, 3] += 0.90
#     box_above.SetTransform(T)

#     box_left = openravepy.RaveCreateKinBody(env, '')
#     box_left.SetName(name_base + 'left')
#     box_left.InitFromBoxes(np.array([[0., 0., 0., 0.1, 0.5, 1.0]]), visible)
#     env.Add(box_left)
#     T = np.eye(4)
#     T[0:3, 3] = robot.GetTransform()[0:3,3]
#     if handedness == 'right':
#         T[0, 3] += 0.9
#     else:
#         T[0, 3] += 0.25
#     T[1, 3] = 0.25
#     box_left.SetTransform(T)

#     box_right = openravepy.RaveCreateKinBody(env, '')
#     box_right.SetName(name_base + 'right')
#     box_right.InitFromBoxes(np.array([[0., 0., 0., 0.1, 0.5, 1.0]]), visible)
#     env.Add(box_right)
#     T = np.eye(4)
#     T[0:3, 3] = robot.GetTransform()[0:3, 3]
#     if handedness == 'right':
#         T[0, 3] -= 0.25
#     else:
#         T[0, 3] -= 0.9
#     T[1, 3] = 0.25
#     box_right.SetTransform(T)


# def read_from_camera():
#     x_tag = 10  # Where x axis is defined
#     y_tag = 17  # Where y axis is defined
#     origin_tag = 19  # Where origin point is defined
#     # For current framework,
#     obj_tags = [2]  # obj_tags is a list of integers that represent different april tags
#     positions = tk.get_object_positions(origin_tag, x_tag, y_tag, obj_tags, timeout=5)
#     pos_offset = np.array([1.050, 0.0175, 0])
#     positions = {k: positions[k] + pos_offset for k in positions.keys()}
#     print("I got positions")
#     print(positions)
#     return positions


# def add_kinbody(env, kinbody_name, _x, _y, num=0):
#     # check the last element of the string
#     if env.GetKinBody(kinbody_name + str(num)) is not None:
#         print(kinbody_name + str(num) + " already exists")
#         return
#     table = env.GetKinBody('table')
#     table_config = table.GetConfigurationValues()
#     table_height = table_config[3]
#     kinbody = env.ReadKinBodyURI('objects/' + kinbody_name + '.kinbody.xml')
#     # print(kinbody)
#     kinbody.SetName(kinbody_name + str(num))

#     kinbody_trans = kinbody.GetTransform()
#     kinbody_trans[0][3] = _x
#     kinbody_trans[1][3] = _y
#     kinbody_trans[2][3] = table_height + 0.042
#     kinbody.SetTransform(kinbody_trans)
#     env.AddKinBody(kinbody)
#     return kinbody, kinbody_name+str(num)


# def graspTSR_cylinder(robot, env, desired_kinbody):
#     # IPython.embed()
#     rospy.loginfo(" I AM HERE TSR CYLINDER")
#     # plan_toConfig('home', 0)
#     # robot.arm.hand.CloseHand(0)
#     kinbody = env.GetKinBody(desired_kinbody)
#     kinbody_trans = kinbody.GetTransform()
#     # endEffectorTrans = robot.arm.GetEndEffectorTransform()
#     T0_w = kinbody_trans
#     Tw_e = np.array([[0., 0., 1., -0.01],  # desired offset between end-effector and object along x-axis
#                     [1., 0., 0., 0],
#                     [0., 1., 0., 0.01],  # glass height
#                     [0., 0., 0., 1.]])

#     Bw = np.zeros((6,2))
#     if 'fuze_bottle' in desired_kinbody or 'glass' in desired_kinbody:
#         Bw[2, :] = [0.0, 0.06]  # Allow a little vertical movement
#     else:
#         Bw[2, :] = [0.0, 0.05]
#     Bw[5, :] = [-np.pi, np.pi]  # Allow any orientation about the z-axis of the glass
#     manip_idx = robot.GetActiveManipulatorIndex()
#     grasp_tsr = prpy.tsr.TSR(T0_w=T0_w, Tw_e=Tw_e, Bw=Bw, manipindex=manip_idx)
#     tsr_chain = prpy.tsr.TSRChain(sample_goal=True, sample_start=False, constrain=False, TSR=grasp_tsr)
#     print(" I AM IN MANIPULATION SERVER !!!!!! ")
#     plan = robot.arm.PlanToTSR([tsr_chain], execute=False, timelimit=10.0)

#     # ee_sample = grasp_tsr.sample()

#     # IPython.embed()
#     # plan = robot.arm.PlanToTSR([tsr_chain], execute = False)
#     if plan is None:
#         return False
#     else:
#         rospy.loginfo("I am executing the plan")
#         new_plan = robot.PostProcessPath(plan)
#         robot.ExecutePath(new_plan)
#         rospy.loginfo("I am gripping now")
#         return True


# def initialization():
#     project_name = 'adapy'

#     # Create environment path
#     data_base_path = find_in_workspaces(project=project_name, path='config', first_match_only=True)
#     if len(data_base_path) == 0:
#         raise Exception('Unable to find environment path. Did you source devel/setup.bash?')
#     env_path = os.path.join(data_base_path[0], 'tablewithobjects.env.xml')

#     # Find ordata and set home configuration
#     rospack = rospkg.RosPack()
#     ordata_root = rospack.get_path('pr_ordata')

#     # Initialize openrave
#     openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Debug)
#     openravepy.misc.InitOpenRAVELogging()

#     # Initialize robot and environment
#     env, robot = adapy.initialize(attach_viewer='rviz', sim=True, env_path=env_path)
#     manipulator = robot.arm
#     set_ada_onto_table(robot)
#     add_constraint_boxes(env, robot)
#     robot.arm.SetActive()
#     return rospack, ordata_root, env, robot, manipulator


# def set_ada_onto_table(robot):
#     # Set Ada onto table
#     robot_pose = np.array([[1., 0., 0., 0.409],
#                               [0., 1., 0., 0.338],
#                               [0., 0., 1., 0.740],
#                               [0., 0., 0., 1.000]])
#     robot.SetTransform(robot_pose)


# def reset_velocity_joint_mode_controller(robot):
#     print('Setting real robot controller')
#     controller_client = ControllerManagerClient(ns='/controller_manager')
#     velocity_joint_mode_controller = controller_client.request('velocity_joint_mode_controller')
#     velocity_joint_mode_controller.switch()
#     trajectory_switcher = controller_client.request('traj_controller')

#     # turn on velocity controllers for each arm
#     for controller_name in robot.arm.velocity_controller_names:
#         velocity_controller = controller_client.request(controller_name)
#         velocity_controller.switch()
#     # turn on velocity controller for each hand
#     for controller_name in robot.arm.hand.velocity_controller_names:
#         velocity_controller = controller_client.request(controller_name)
#         velocity_controller.switch()
#     print('Done')


# def reset_robot(robot):
#     if robot.simulated:
#         num_hand_dofs = len(robot.arm.hand.GetDOFValues())
#         inds, pos = robot.configurations.get_configuration('home')
#         with robot.GetEnv():
#             robot.SetDOFValues(pos, inds)
#             robot.arm.hand.SetDOFValues(np.ones(num_hand_dofs)*0.1)
#     else:
#         n = robot.GetDOF()
#         dof_lim = robot.GetDOFLimits()
#         vel_lim = robot.GetDOFVelocityLimits()
#         # robot.SetDOFLimits(-10 * np.ones(n), 10 * np.ones(n))
#         # robot.SetDOFVelocityLimits(100 * vel_lim)

#         inds, pos = robot.configurations.get_configuration('home')
#         # robot.arm.SetVelocityLimits()
#         return_traj = robot.arm.PlanToConfiguration(pos)
#         openravepy.planningutils.RetimeActiveDOFTrajectory(return_traj, robot,
#                                                            hastimestamps=False,
#                                                            maxvelmult=0.5, maxaccelmult=0.5,
#                                                            plannername='ParabolicTrajectoryRetimer')
#         robot.ExecuteTrajectory(return_traj)
#         reset_velocity_joint_mode_controller(robot)


# def get_formatted_trajs(dof_traj, ee_traj):
#     user_joint0 = [elem[0] for elem in dof_traj]
#     user_joint1 = [elem[1] for elem in dof_traj]
#     user_joint2 = [elem[2] for elem in dof_traj]
#     user_joint3 = [elem[3] for elem in dof_traj]
#     user_joint4 = [elem[4] for elem in dof_traj]
#     user_joint5 = [elem[5] for elem in dof_traj]
#     user_joint6 = [elem[6] for elem in dof_traj]
#     user_joint7 = [elem[7] for elem in dof_traj]
#     # All eight joint trajectories, change this to six joints later (first six joints)
#     user_dof_trajs = [user_joint0, user_joint1, user_joint2, user_joint3,
#                       user_joint4, user_joint5, user_joint6, user_joint7]

#     # ee_transform matrix to x, y, z, roll, pitch, yaw trajectories
#     user_x_traj = [elem[0, 3] for elem in ee_traj]
#     user_y_traj = [elem[1, 3] for elem in ee_traj]
#     user_z_traj = [elem[2, 3] for elem in ee_traj]

#     rot_matrix_traj = [elem[0:3, 0:3] for elem in ee_traj]
#     euler_angles_traj = [rotation_matrix_to_euler_angles(R) for R in rot_matrix_traj]

#     user_roll_traj = [elem[0] for elem in euler_angles_traj]
#     user_pitch_traj = [elem[1] for elem in euler_angles_traj]
#     user_yaw_traj = [elem[2] for elem in euler_angles_traj]

#     # x, y, z, roll, pitch, yaw trajectories
#     user_trajs = [user_x_traj, user_y_traj, user_z_traj, user_roll_traj, user_pitch_traj, user_yaw_traj]
#     return user_dof_trajs, user_trajs


# def test_dmp():
#     save_path = '/home/calvinqiao/fun-workspace/src/ada/adapy/scripts/user_data/dof_traj_another.pkl'
#     saved_file = open(save_path, 'rb')
#     saved_data = pickle.load(saved_file)
#     dof_traj = [elem[1]['robot_dof_values'] for elem in saved_data]
#     ee_traj = [elem[1]['robot_state'].ee_trans for elem in saved_data]
#     saved_file.close()
#     # scipy.io.savemat('/home/calvinqiao/Documents/data.mat', mdict={'traj_test': traj_test})

#     user_joint0 = [elem[0] for elem in dof_traj]
#     user_joint1 = [elem[1] for elem in dof_traj]
#     user_joint2 = [elem[2] for elem in dof_traj]
#     user_joint3 = [elem[3] for elem in dof_traj]
#     user_joint4 = [elem[4] for elem in dof_traj]
#     user_joint5 = [elem[5] for elem in dof_traj]
#     user_joint6 = [elem[6] for elem in dof_traj]
#     user_joint7 = [elem[7] for elem in dof_traj]
#     # All eight joint trajectories, change this to six joints later (first six joints)
#     user_dof_trajs = [user_joint0, user_joint1, user_joint2, user_joint3,
#                       user_joint4, user_joint5, user_joint6, user_joint7]

#     # ee_transform matrix to x, y, z, roll, pitch, yaw trajectories
#     user_x_traj = [elem[0, 3] for elem in ee_traj]
#     user_y_traj = [elem[1, 3] for elem in ee_traj]
#     user_z_traj = [elem[2, 3] for elem in ee_traj]

#     rot_matrix_traj = [elem[0:3, 0:3] for elem in ee_traj]
#     euler_angles_traj = [rotation_matrix_to_euler_angles(R) for R in rot_matrix_traj]

#     user_roll_traj = [elem[0] for elem in euler_angles_traj]
#     user_pitch_traj = [elem[1] for elem in euler_angles_traj]
#     user_yaw_traj = [elem[2] for elem in euler_angles_traj]

#     # x, y, z, roll, pitch, yaw trajectories
#     user_trajs = [user_x_traj, user_y_traj, user_z_traj, user_roll_traj, user_pitch_traj, user_yaw_traj]

#     plt.figure(1, figsize=(6, 4))

#     # set up number of basis functions
#     n_bfs = [5000]  # [30, 50, 100, 500]
#     dt = 0.001

#     # Initial tests
#     # create target trajectory 1
#     traj1 = np.sin(np.arange(0, 1, dt) * 2)
#     # create target trajectory 2
#     traj2 = np.zeros(traj1.shape)
#     traj2[int(len(traj2) / 2.0):] = 0.5
#     # create target trajectory 3
#     mjt = MinJerkTrajectory(dt)
#     # mjt.run_time = RUNTIME
#     traj3 = mjt.gen_trajectory()

#     # Test grasp from right side
#     # Target goal in x, y, z, yaw, pitch, roll respectively
#     goals_grasp0 = [7.87450418e-01, 1.20222077e-01, 8.05897859e-01, 1.5707, -0.0027, 2.0351]
#     # goals_grasp1 =

#     n_dofs = 6

#     x_track, dx_track, ddx_track = None, None, None
#     y_track, dy_track, ddy_track = None, None, None
#     z_track, dz_track, ddz_track = None, None, None
#     yaw_track, dyaw_track, ddyaw_track = None, None, None
#     pitch_track, dpitch_track, ddpitch_track = None, None, None
#     roll_track, droll_track, ddroll_track = None, None, None

#     for i in range(n_dofs):
#         # iterate through different number of basis functions
#         for ii, bfs in enumerate(n_bfs):
#             # initialize a DMP system
#             dmp = DMPDiscrete(dt=dt, n_bfs=bfs)
#             # dmp.cs.run_time = RUNTIME

#             # Learn the path
#             mjt.goal = goals_grasp0[i]
#             mjt_traj = mjt.gen_trajectory()
#             # traj_to_learn = np.copy(mjt_traj)
#             # dmp.learn_path(y_des=np.array([traj_to_learn]))
#             dmp.learn_path(y_des=np.array([user_dof_trajs[i]]))
#             # plt.plot(mjt_traj, 'r--', lw=2)

#             plt.plot(user_dof_trajs[i], 'r--', lw=2)

#             # Change the goal of the movement
#             # Set up target grasp
#             dmp.y_goal[0] = user_dof_trajs[i][-1]  # goals_grasp0[i]  # 0.5*(i+1)
#             # Test
#             # if i == 2:
#                 # dmp.y_goal[0] = dmp.y_goal[0] + 0.2

#             # construct corresponding trajectory
#             pos_track, vel_track, acc_track = dmp.construct_trajectory()
#             if i == 0:
#                 pass  # print(vel_track)
#             # save constructed trajectories
#             if i == 0:
#                 x_track, dx_track, ddx_track = np.copy(pos_track), np.copy(vel_track), np.copy(acc_track)
#             elif i == 1:
#                 y_track, dy_track, ddy_track = np.copy(pos_track), np.copy(vel_track), np.copy(acc_track)
#             elif i == 2:
#                 z_track, dz_track, ddz_track = np.copy(pos_track), np.copy(vel_track), np.copy(acc_track)
#             elif i == 3:
#                 yaw_track, dyaw_track, ddyaw_track = np.copy(pos_track), np.copy(vel_track), np.copy(acc_track)
#             elif i == 4:
#                 pitch_track, dpitch_track, ddpitch_track = np.copy(pos_track), np.copy(vel_track), np.copy(acc_track)
#             elif i == 5:
#                 roll_track, droll_track, ddroll_track = np.copy(pos_track), np.copy(vel_track), np.copy(acc_track)
#             print(len(vel_track))
#             '''
#             time_steps = len(vel_track)
#             time_steps_array = np.linspace(0, time_steps, time_steps)
#             # time_steps_list = [int(num) for num in time_steps_array]
#             new_time_steps_array = np.linspace(0, time_steps, 2000)
#             # new_time_steps_list = [int(num) for num in new_time_steps_array]
#             vel_track = vel_track[:, 0]

#             print(new_time_steps_array.shape, time_steps_array.shape, vel_track.shape)
#             new_vel_track = np.interp(new_time_steps_array, time_steps_array, vel_track)
#             print(len(new_vel_track))'''
#             # plot constructed trajectory using this number of bfs
#             plt.plot(pos_track[:, 0], lw=2)
#             # plt.plot(new_time_steps_array, new_vel_track, lw=2)

#             # t_track = np.linspace(0, 1, num=len(y_track[:, 0]))
#             # plt.plot(t_track, y_track[:, 0], lw=2)

#     '''
#     # plot target trajectory
#     plt.plot(path / path[-1] * dmp.y_goal[0], 'r--', lw=2)
#     plt.title('DMP learn path')
#     plt.xlabel('time (ms)')
#     plt.ylabel('constructed system trajectory')
#     # plt.legend([a[0]], ['desired path'], loc='upper right')
#     plt.legend(['%i BFs' % i for i in n_bfs], loc='lower right')
#     plt.show()'''

#     # Plot the target trajectory
#     # t_path = np.linspace(0, 1, num=len(path))
#     # print(t_path)
#     # print(dmp.y_goal[0])
#     # plt.plot(t_path, path / path[-1] * dmp.y_goal[0], 'r--', lw=2)

#     print('steps = ', len(x_track))
#     # plt.plot(path, 'r--', lw=2)
#     plt.title('DMP learn path')
#     plt.xlabel('time (ms)')
#     plt.ylabel('constructed system trajectory')
#     # plt.legend([a[0]], ['desired path'], loc='upper right')
#     # plt.legend(['%2f goal in each dof' % i for i in goals_grasp0], loc='lower right')
#     plt.legend(['%i BFs' % i for i in n_bfs], loc='lower right')
#     plt.show()


# def record_user_data(hydra_recording, robot, env, save_path):
#     # Record trajectories that correspond to user control - joint dof values and end effector transform
#     if hydra_recording:
#         print('Recording data')
#         traj_recorder = TrajectoryData(save_path)
#         ada_teleoperator = AdaTeleopHandler(env, robot, 'hydra', 2)
#         ada_teleoperator.ExecuteDirectTeleop(traj_data_recording=traj_recorder)
#     else:
#         robot_state = RobotStateDMP(robot.arm.GetEndEffectorTransform(), robot.GetDOFValues())
#         data_recorder = dataRecorder.TrajectoryData(save_path)
#         recording_start_time = time.time()
#         while time.time() - recording_start_time < 7:
#             robot_state.update(robot.arm.GetEndEffectorTransform(), robot.GetDOFValues())
#             data_recorder.add_datapoint(robot_state=copy.deepcopy(robot_state))
#             time.sleep(0.001)
#         data_recorder.tofile()


# def load_data(load_path):
#     with open(load_path, 'rb') as load_file:
#         saved_data = pickle.load(load_file)
#     dof_traj = [elem[1]['robot_dof_values'] for elem in saved_data]
#     ee_traj = [elem[1]['robot_state'].ee_trans for elem in saved_data]
#     load_file.close()
#     return get_formatted_trajs(dof_traj, ee_traj)


# def load_DMPs(load_path):
#     with open(load_path, 'rb') as load_file:
#         saved_data = dill.load(load_file)
#     load_file.close()
#     return saved_data


# def save_data(save_dmp_path, data):
#     with open(save_dmp_path, 'wb') as save_file:
#         dill.dump(data, save_file)
#     save_file.close()


# # Grasps need to be set relative to object
# def set_up_relative_grasps(object_translation, object_orientation,
#                            x_offset, y_offset, z_offset, rol_offset, pit_offset, yaw_offset):
#     object_orientation[0] =  np.pi/2  # Calibrate roll to zero pos according to the platform setup
#     # Set up different relative grasps
#     grasp_translation = np.copy(object_translation)
#     grasp_orientation = np.copy(object_orientation)
#     grasp_translation[0] += x_offset
#     grasp_translation[1] += y_offset
#     grasp_translation[2] += z_offset
#     grasp_orientation[0] += rol_offset
#     grasp_orientation[1] += pit_offset
#     grasp_orientation[2] += yaw_offset
#     print('rot ', grasp_orientation[2])
#     grasp_orientation_matrix = euler_angles_to_rotation_matrix(grasp_orientation)

#     grasp_transform = np.array([[0,   0,   0, grasp_translation[0]],
#                                 [0,   0,   0, grasp_translation[1]],
#                                 [0,   0,   0, grasp_translation[2]],
#                                 [0.00000000e+00,   0.00000000e+00,   0.00000000e+00, 1.00000000e+00]])
#     grasp_transform[0: 3, 0: 3] = grasp_orientation_matrix
#     return grasp_transform


# def compute_target_grasp_joint_values(robot, grasps):
#     # Compute target grasps joint dof values
#     target_grasps, target_grasp_joint_dofs, target_6dofs = [], [], []
#     ranker = prpy.ik_ranking.NominalConfiguration(robot.arm.GetArmDOFValues())
#     for idx_grasp in range(len(grasps)):
#         print('Grasp idx = ', idx_grasp)
#         with robot.GetEnv():
#             # print('Set up ik param')
#             ik_param = IkParameterization(grasps[idx_grasp], IkParameterizationType.Transform6D)
#             # print('Done')
#             print('Start finding IK solutions')
#             ik_solutions = robot.arm.FindIKSolutions(
#                 ik_param, IkFilterOptions.CheckEnvCollisions,
#                 ikreturn=False, releasegil=True
#             )
#             if len(ik_solutions) != 0:
#                 print('Found a feasible grasp')
#                 sol = ik_solutions[0]
#                 target_grasp_joint_dofs.append(sol)
#                 target_grasps.append(grasps[idx_grasp])
#                 target_translation = np.array([grasps[idx_grasp][0, 3],
#                                                grasps[idx_grasp][1, 3],
#                                                grasps[idx_grasp][2, 3]])
#                 target_orientation = rotation_matrix_to_euler_angles(grasps[idx_grasp][0:3, 0:3])
#                 target_6dofs.append([target_translation[0], target_translation[1], target_translation[2],
#                                      target_orientation[0], target_orientation[1], target_orientation[2]])
#                 scores = ranker(robot, ik_solutions)
#                 ranked_indices = np.argsort(scores)
#                 ranked_indices = ranked_indices[~np.isposinf(scores)]
#                 ranked_ik_solutions = ik_solutions[ranked_indices, :]
#                 if len(ranked_ik_solutions) != 0:
#                     sol = np.append(ranked_ik_solutions[0], [0, 0]).reshape((8, 1))
#                     target_grasp_joint_dofs.append(sol)
#                     target_grasps.append(grasps[idx_grasp])
#                     target_translation = np.array([grasps[idx_grasp][0, 3],
#                                                    grasps[idx_grasp][1, 3],
#                                                    grasps[idx_grasp][2, 3]])
#                     target_orientation = rotation_matrix_to_euler_angles(grasps[idx_grasp][0:3, 0:3])
#                     target_6dofs.append([target_translation[0], target_translation[1], target_translation[2],
#                                          target_orientation[0], target_orientation[1], target_orientation[2]])
#                 else:
#                     print('This is an infeasible grasp configuration')
#             else:
#                 print('No feasible grasp configuration found')
#     return target_grasps, target_grasp_joint_dofs, target_6dofs


# def get_current_configurations(robot):
#     home_ee_transform = robot.arm.GetEndEffectorTransform()
#     home_rotation_matrix = home_ee_transform[0:3, 0:3]
#     home_angles = rotation_matrix_to_euler_angles(home_rotation_matrix)
#     home_angles = [float(angle) for angle in home_angles]
#     home_position = [float(home_ee_transform[0, 3]), float(home_ee_transform[1, 3]), float(home_ee_transform[2, 3])]
#     # x, y, z, roll, pitch, yaw
#     home_6dofs = [home_position[0], home_position[1], home_position[2], home_angles[0], home_angles[1], home_angles[2]]
#     home_joints = robot.GetDOFValues()
#     return home_ee_transform, home_6dofs, home_joints


# def select_num_dofs(MODE):
#     if MODE == 'joint_min_jerk' or MODE == 'joint_user':  # Joints
#         n_dofs = 8  # 8
#     else:
#         n_dofs = 6
#     return n_dofs


# def get_nearest_grasp(MODE, home_6dofs, home_joint_values,
#                       target_grasp_joint_values, target_grasp_6dofs,
#                       grasps, num_dofs):
#     home_joint_values_list = home_joint_values.tolist()
#     nearest_grasp_idx = 0
#     grasp_distance = None
#     # Change it later to other distance, 6dof or x, y, z distance
#     if MODE == 'joint_min_jerk' or MODE == 'Joint_user':
#         for idx_grasp in range(len(grasps)):
#             temp_distance = 0
#             target_dof_list = target_grasp_joint_values[idx_grasp].tolist()
#             for i in range(num_dofs):
#                 temp_distance += (home_joint_values_list[i] - target_dof_list[i][0]) ** 2
#             # print('distance = ', temp_distance)
#             if grasp_distance is None or temp_distance < grasp_distance:
#                 grasp_distance = temp_distance
#                 nearest_grasp_idx = idx_grasp
#     else:
#         for idx_grasp in range(len(grasps)):
#             temp_distance = 0
#             target_dof_list = target_grasp_6dofs[idx_grasp]
#             for i in range(num_dofs):
#                 temp_distance += (home_6dofs[i] - target_dof_list[i]) ** 2
#             # print('distance = ', temp_distance)
#             if grasp_distance is None or temp_distance < grasp_distance:
#                 grasp_distance = temp_distance
#                 nearest_grasp_idx = idx_grasp
#     # else:
#     #     raise Exception('Invalid mode selection')

#     # print('Nearest grasp idx = ', nearest_grasp_idx)
#     return nearest_grasp_idx


# def compute_goals_to_learn_for_DMP(MODE, nearest_grasp_idx,
#                                    home_joint_values, home_6dofs,
#                                    target_grasp_joint_values, target_grasp_6dofs):
#     result = None
#     if MODE == 'joint_min_jerk' or MODE == 'joint_user':
#         # Compute joint offset values from home joint values
#         # for idx_grasp in range(len(grasps)):
#         goal_joint_values = [float(value) for value in target_grasp_joint_values[nearest_grasp_idx]]
#         # for idx_grasp in range(len(grasps)):
#         for i in range(len(home_joint_values)):
#             goal_joint_values[i] = goal_joint_values[i] - float(home_joint_values[i])
#         result = goal_joint_values
#     elif MODE == 'six_dofs_min_jerk':
#         goal_6dofs = copy.deepcopy(target_grasp_6dofs[nearest_grasp_idx])
#         # Compute offset between target pose and current pose
#         # for idx_grasp in range(len(grasps)):
#         for i in range(len(home_6dofs)):
#             # print('i = ', i)
#             goal_6dofs[i] = target_grasp_6dofs[nearest_grasp_idx][i] - home_6dofs[i]
#         result = goal_6dofs
#     elif MODE == 'user_translation_minjerk_orientation':
#         goal_6dofs = copy.deepcopy(target_grasp_6dofs[nearest_grasp_idx])
#         for i in range(len(home_6dofs)/2, len(home_6dofs)):
#             print('i = ', i)
#             goal_6dofs[i] = target_grasp_6dofs[nearest_grasp_idx][i] - home_6dofs[i]
#         goal_6dofs[0] = target_grasp_6dofs[nearest_grasp_idx][0]  # userx_6dofs_trajs[0][-1]  # x traj
#         goal_6dofs[1] = target_grasp_6dofs[nearest_grasp_idx][1]  # usery_6dofs_trajs[1][-1]  # y traj
#         goal_6dofs[2] = target_grasp_6dofs[nearest_grasp_idx][2]  # userz_6dofs_trajs[2][-1]  # z traj
#         result = goal_6dofs
#         # goal_6dofs = copy.deepcopy(target_grasp_6dofs[nearest_grasp_idx])
#         # for i in range(len(home_6dofs)):
#         #     print('i = ', i)
#         #     goal_6dofs[i] = target_grasp_6dofs[nearest_grasp_idx][i]
#         # result = goal_6dofs
#     else:
#         goal_6dofs = copy.deepcopy(target_grasp_6dofs[nearest_grasp_idx])
#         for i in range(len(target_grasp_6dofs[0])):
#             goal_6dofs[i] = target_grasp_6dofs[nearest_grasp_idx][i]
#         result = goal_6dofs
#     assert result is not None
#     return result


# def get_trajectory_to_learn(MODE, dt, goals_for_DMP,
#                             user_dof_trajs, user_trajs,
#                             userxyz_joint_dofs_trajs, userxyz_6dofs_trajs,
#                             userall_joint_dofs_trajs, userall_6dofs_trajs,
#                             userall_no_switch_joint_dofs_trajs, userall_no_switch_6dofs_trajs,
#                             idx_dof):
#     result = None
#     # Learn the path
#     if MODE == 'joint_min_jerk':
#         mjt = MinJerkTrajectory(dt)
#         mjt.goal = goals_for_DMP[idx_dof]
#         result = mjt.gen_trajectory()
#     elif MODE == 'joint_user':
#         result = user_dof_trajs[idx_dof]
#     elif MODE == 'six_dofs_user':
#         result = user_trajs[idx_dof]
#     elif MODE == 'six_dofs_min_jerk':
#         mjt = MinJerkTrajectory(dt)
#         mjt.goal = goals_for_DMP[idx_dof]
#         result = mjt.gen_trajectory()
#     elif MODE == 'user_translation_minjerk_orientation':
#         if idx_dof >= 3:
#             mjt = MinJerkTrajectory(dt)
#             mjt.goal = goals_for_DMP[idx_dof]
#             result = mjt.gen_trajectory()
#         elif idx_dof == 0:
#             result = userxyz_6dofs_trajs[idx_dof] # userx_6dofs_trajs[idx_dof]
#         elif idx_dof == 1:
#             result = userxyz_6dofs_trajs[idx_dof] # usery_6dofs_trajs[idx_dof]
#         elif idx_dof == 2:
#             result = userxyz_6dofs_trajs[idx_dof] # userz_6dofs_trajs[idx_dof]
#     elif MODE == 'user_all':
#         result = userall_6dofs_trajs[idx_dof]
#     elif MODE == 'user_translation_only_and_userall_orientation':
#         if idx_dof == 0:
#             result = userxyz_6dofs_trajs[idx_dof]  # userx_6dofs_trajs[idx_dof]  # userx_6dofs_trajs[idx_dof]
#         elif idx_dof == 1:
#             result = userxyz_6dofs_trajs[idx_dof] # usery_6dofs_trajs[idx_dof]   # usery_6dofs_trajs[idx_dof]
#         elif idx_dof == 2:
#             result = userxyz_6dofs_trajs[idx_dof] # userz_6dofs_trajs[idx_dof]  # userxyz_6dofs_trajs[idx_dof] # userz_6dofs_trajs[idx_dof]
#         else:
#             result = userall_6dofs_trajs[idx_dof]
#     elif MODE == 'user_translation_only_orientation_usrctrl_meanwhile':
#         result = userall_no_switch_6dofs_trajs[idx_dof]
#     assert result is not None
#     return result


# def learn_DMP(MODE, dt, n_bfses, num_dofs,
#                 user_joint_dofs_trajs, user_6dofs_trajs,
#                 userxyz_joint_dofs_trajs, userxyz_6dofs_trajs,
#                 userall_joint_dofs_trajs, userall_6dofs_trajs,
#                 userall_no_switch_joint_dofs_trajs, userall_no_switch_6dofs_trajs,
#                 vertical_joints_trajs, vertical_6dofs_trajs):
#     DMP_collections = []

#     for idx_dof in range(num_dofs):
#         traj_to_learn = get_trajectory_to_learn(MODE, dt,
#                                                 user_dof_trajs, user_trajs,
#                                                 userxyz_joint_dofs_trajs, userxyz_6dofs_trajs,
#                                                 userall_joint_dofs_trajs, userall_6dofs_trajs,
#                                                 userall_no_switch_joint_dofs_trajs, userall_no_switch_6dofs_trajs,
#                                                 vertical_joints_trajs, vertical_6dofs_trajs,
#                                                 idx_dof)
#         if idx_dof == 0:
#             text = 'trajs to learn'
#             plt.plot(traj_to_learn, 'r--', lw=2, label=text)
#         else:
#             plt.plot(traj_to_learn, 'r--', lw=2)

#         dmp_this_dof = DMPDiscrete(dt=dt, n_bfs=n_bfses[idx_dof])
#         dmp_this_dof.learn_path(y_des=np.array([traj_to_learn]))

#         print('Learning idx = ', idx_dof)
#         DMP_collections.append(dmp_this_dof)
#     return DMP_collections


# def construct_trajectories(MODE, DMP_collections_6dof_minjerk, num_dofs, goals_for_DMP):
#     if MODE == 'joint_min_jerk' or MODE == 'joint_user':
#         joint_tracks, djoint_tracks, ddjoint_tracks = [], [], []
#         joint0, djoint0, ddjoint0 = None, None, None
#         joint1, djoint1, ddjoint1 = None, None, None
#         joint2, djoint2, ddjoint2 = None, None, None
#         joint3, djoint3, ddjoint3 = None, None, None
#         joint4, djoint4, ddjoint4 = None, None, None
#         joint5, djoint5, ddjoint5 = None, None, None
#         joint6, djoint6, ddjoint6 = None, None, None
#         joint7, djoint7, ddjoint7 = None, None, None
#     else:  # elif MODE == Mode.six_dofs_min_jerk or MODE == Mode.six_dofs_user:
#         six_dof_tracks, dsix_dof_tracks, ddsix_dof_tracks = [], [], []
#         x_track, dx_track, ddx_track = None, None, None
#         y_track, dy_track, ddy_track = None, None, None
#         z_track, dz_track, ddz_track = None, None, None
#         roll_track, droll_track, ddroll_track = None, None, None
#         pitch_track, dpitch_track, ddpitch_track = None, None, None
#         yaw_track, dyaw_track, ddyaw_track = None, None, None
#     for idx_dof in range(num_dofs):
#         dmp_this_dof = DMP_collections_6dof_minjerk[idx_dof]
#         dmp_this_dof.y_goal[0] = goals_for_DMP[idx_dof]

#         # Construct corresponding trajectory
#         if idx_dof < 3:
#             dmp_this_dof.y[0] = 0.0
#             print('Constructing trjectory for DOF idx = ', idx_dof)
#             pos_track, vel_track, acc_track, cs_x_track = dmp_this_dof.construct_trajectory()
#         else:
#             print('Constructing trjectory for DOF idx = ', idx_dof)
#             pos_track, vel_track, acc_track, cs_x_track = dmp_this_dof.construct_trajectory()

#         # plot constructed trajectory using this number of bfs
#         if MODE != 'joint_min_jerk' or 'joint_user':
#             if idx_dof == 0:
#                 text = 'x'
#             elif idx_dof == 1:
#                 text = 'y'
#             elif idx_dof == 2:
#                 text = 'z'
#             elif idx_dof == 3:
#                 text = 'roll'
#             elif idx_dof == 4:
#                 text = 'pitch'
#             elif idx_dof == 5:
#                 text = 'yaw'
#             plt.plot(pos_track[:, 0], lw=2, label=text)
#         else:
#             plt.plot(pos_track[:, 0], lw=2)

#         # Save constructed trajectories)
#         if MODE == 'joint_min_jerk' or MODE == 'joint_user':
#             if idx_dof == 0:
#                 joint0, djoint0, ddjoint0 = np.copy(pos_track), np.copy(vel_track), np.copy(acc_track)
#             elif idx_dof == 1:
#                 joint1, djoint1, ddjoint1 = np.copy(pos_track), np.copy(vel_track), np.copy(acc_track)
#             elif idx_dof == 2:
#                 joint2, djoint2, ddjoint2 = np.copy(pos_track), np.copy(vel_track), np.copy(acc_track)
#             elif idx_dof == 3:
#                 joint3, djoint3, ddjoint3 = np.copy(pos_track), np.copy(vel_track), np.copy(acc_track)
#             elif idx_dof == 4:
#                 joint4, djoint4, ddjoint4 = np.copy(pos_track), np.copy(vel_track), np.copy(acc_track)
#             elif idx_dof == 5:
#                 joint5, djoint5, ddjoint5 = np.copy(pos_track), np.copy(vel_track), np.copy(acc_track)
#             elif idx_dof == 6:
#                 joint6, djoint6, ddjoint6 = np.copy(pos_track), np.copy(vel_track), np.copy(acc_track)
#             elif idx_dof == 7:
#                 joint7, djoint7, ddjoint7 = np.copy(pos_track), np.copy(vel_track), np.copy(acc_track)
#         else:  # if MODE == Mode.six_dofs_min_jerk or MODE == Mode.six_dofs_user:
#             if idx_dof == 0:
#                 x_track, dx_track, ddx_track = np.copy(pos_track), np.copy(vel_track), np.copy(acc_track)
#             elif idx_dof == 1:
#                 y_track, dy_track, ddy_track = np.copy(pos_track), np.copy(vel_track), np.copy(acc_track)
#             elif idx_dof == 2:
#                 z_track, dz_track, ddz_track = np.copy(pos_track), np.copy(vel_track), np.copy(acc_track)
#             elif idx_dof == 3:
#                 roll_track, droll_track, ddroll_track = np.copy(pos_track), np.copy(vel_track), np.copy(acc_track)
#             elif idx_dof == 4:
#                 pitch_track, dpitch_track, ddpitch_track = np.copy(pos_track), np.copy(vel_track), np.copy(acc_track)
#             elif idx_dof == 5:
#                 yaw_track, dyaw_track, ddyaw_track = np.copy(pos_track), np.copy(vel_track), np.copy(acc_track)

#     # Save constructed joint trajectories
#     if MODE == 'joint_min_jerk' or MODE == 'joint_user':
#         joint_tracks.append([joint0, joint1, joint2, joint3, joint4, joint5, joint6, joint7])
#         djoint_tracks.append([djoint0, djoint1, djoint2, djoint3, djoint4, djoint5, djoint6, djoint7])
#         ddjoint_tracks.append([ddjoint0, ddjoint1, ddjoint2, ddjoint3, ddjoint4, ddjoint5, ddjoint6, ddjoint7])
#         planned_trajectories = [joint_tracks, djoint_tracks, ddjoint_tracks]
#     else:
#         six_dof_tracks.append([x_track, y_track, z_track, roll_track, pitch_track, yaw_track])
#         dsix_dof_tracks.append([dx_track, dy_track, dz_track, droll_track, dpitch_track, dyaw_track])
#         ddsix_dof_tracks.append([ddx_track, ddy_track, ddz_track, ddroll_track, ddpitch_track, ddyaw_track])
#         planned_trajectories = [six_dof_tracks, dsix_dof_tracks, ddsix_dof_tracks]
#         return planned_trajectories, cs_x_track


# def get_joint_traj(time_steps, result, robot, dt):
#     joint_traj = []
#     for time_iter in range(time_steps / 2):
#         # Get dx vector, 6 by 1
#         vel_x = result[1][0][0][time_iter]
#         vel_y = result[1][0][1][time_iter]
#         vel_z = result[1][0][2][time_iter]
#         ang_vel_roll = result[1][0][3][time_iter]
#         ang_vel_pitch = result[1][0][4][time_iter]
#         ang_vel_yaw = result[1][0][5][time_iter]
#         dx = np.array([vel_x, vel_y, vel_z, ang_vel_roll, ang_vel_pitch, ang_vel_yaw])

#         # Compute Jacobian at this configuration
#         jacobian_spatial = robot.arm.CalculateJacobian()
#         jacobian_angular = robot.arm.CalculateAngularVelocityJacobian()
#         jacobian = np.vstack((jacobian_spatial, jacobian_angular))

#         # Compute dq at this time step, dq = inv(jacobian) * dx
#         dq = np.dot(np.linalg.inv(jacobian), dx)
#         # Compute dof values for next step
#         dof_current = robot.arm.GetDOFValues().reshape((6, 1))
#         print('dof current = ', np.transpose(dof_current))
#         delta_q = dq * dt
#         print('delta joint = ', np.transpose(delta_q))

#         dof_next = np.add(dof_current, delta_q)
#         dof_next = [float(num) for num in dof_next]
#         joint_traj.append(dof_next)
#         print('dof next = ', dof_next)
#         # robot.arm.SetDOFValues(dof_next)
#         time.sleep(dt)


# def quadraticObjective(dq, J, dx, *args):
#     """
#     Quadratic objective function for SciPy's optimization.
#     @param dq joint velocity
#     @param J Jacobian
#     @param dx desired twist
#     @return objective the objective function
#     @return gradient the analytical gradient of the objective
#     """
#     error = (np.dot(J, dq) - dx)
#     objective = 0.5 * np.dot(np.transpose(error), error)
#     gradient = np.dot(np.transpose(J), error)
#     return objective, gradient


# def execute_reconstructed_trajectories(MODE, joint_velocity_mode,
#                                        robot, env,
#                                        dt, time_intervals,
#                                        planned_trajectories, home_dofs):
#     # DOF and velocity limits
#     # http://wiki.ros.org/jaco
#     n = robot.GetDOF()
#     dof_lim = robot.GetDOFLimits()
#     vel_lim = robot.GetDOFVelocityLimits()
#     robot.SetDOFLimits(-10000 * np.ones(n), 10000 * np.ones(n))
#     robot.SetDOFVelocityLimits(100 * vel_lim)  # 100* original

#     joint_traj, dq_traj = [], []
#     x_traj, y_traj, z_traj = [], [], []
#     roll_traj, pitch_traj, yaw_traj = [], [], []
#     dx_traj, dy_traj, dz_traj = [], [], []
#     droll_traj, dpitch_traj, dyaw_traj = [], [], []
#     roll_roll_traj = []

#     time_steps = len(planned_trajectories[0][0][0])

#     # Execute learned and then reconstructed trajectories
#     if MODE == 'joint_user' or MODE == 'joint_min_jerk':
#         this_dof = [0, 0, 0, 0, 0, 0, 0, 0]
#         r_outer = rospy.Rate(int(1.0 / (dt * time_intervals)))
#         for time_iter in range(time_steps):
#             new_dofs = [float(planned_trajectories[0][0][0][time_iter]),
#                         float(planned_trajectories[0][0][1][time_iter]),
#                         float(planned_trajectories[0][0][2][time_iter]),
#                         float(planned_trajectories[0][0][3][time_iter]),
#                         float(planned_trajectories[0][0][4][time_iter]),
#                         float(planned_trajectories[0][0][5][time_iter]),
#                         float(planned_trajectories[0][0][6][time_iter]),
#                         float(planned_trajectories[0][0][7][time_iter])]

#             for i in range(len(this_dof)):
#                 if MODE == 'joint_min_jerk':
#                     this_dof[i] = new_dofs[i] + home_dofs.tolist()[i]
#                 elif MODE == 'joint_user':
#                     this_dof[i] = new_dofs[i]

#             robot.SetDOFValues(this_dof)
#             r_outer.sleep()
#     else:
#         curr_ee_transform, curr_6dofs, curr_joint_values = get_current_configurations(robot)
#         temp = copy.deepcopy(curr_6dofs)
#         r_outer = rospy.Rate(int(1.0 / (dt * time_intervals)))
#         for time_iter in range(0, time_steps-1, time_intervals):
#             # Get dx vector, 6 by 1
#             vel_x = planned_trajectories[1][0][0][time_iter]
#             vel_y = planned_trajectories[1][0][1][time_iter]
#             vel_z = planned_trajectories[1][0][2][time_iter]
#             vel_roll = planned_trajectories[1][0][3][time_iter]
#             vel_pitch = planned_trajectories[1][0][4][time_iter]
#             vel_yaw = planned_trajectories[1][0][5][time_iter]
#             # dx = np.array([vel_x, vel_y, vel_z, vel_roll, vel_pitch, vel_yaw])

#             # Compute Jacobian at this configuration
#             jacobian_spatial = robot.arm.CalculateJacobian()
#             jacobian_angular = robot.arm.CalculateAngularVelocityJacobian()
#             jacobian = np.vstack((jacobian_spatial, jacobian_angular))

#             if robot.simulated:
#                 current_joint_vel = robot.arm.servo_simulator.q_dot
#                 current_ee_vel = np.dot(jacobian, current_joint_vel)  # check format

#                 roll_roll_traj.append(temp[3])
#                 temp[3] = temp[3] + current_ee_vel[3]*dt*time_intervals

#                 dx_traj.append(current_ee_vel[0])
#                 dy_traj.append(current_ee_vel[1])
#                 dz_traj.append(current_ee_vel[2])
#                 droll_traj.append(current_ee_vel[3])
#                 dpitch_traj.append(current_ee_vel[4])
#                 dyaw_traj.append(current_ee_vel[5])

#                 # print('time iter = ', time_iter, (current_ee_vel[3], vel_roll[0]), (current_ee_vel[4], vel_pitch[0]),
#                 #       (current_ee_vel[5], vel_yaw[0]))
#                 # if abs(vel_roll-current_ee_vel[3])/vel_roll > 0.01:
#                 #     vel_roll = vel_roll*(1 - abs(vel_roll-current_ee_vel[3])/vel_roll)
#                 # if abs(vel_pitch-current_ee_vel[4])/vel_pitch > 0.01:
#                 #     vel_pitch = vel_pitch*(1 - abs(vel_pitch-current_ee_vel[4])/vel_pitch)
#                 # if abs(vel_yaw-current_ee_vel[5])/vel_yaw > 0.01:
#                 #     vel_yaw = vel_yaw*(1 - abs(vel_yaw-current_ee_vel[5])/vel_yaw)
#                 # print('time iter = ', time_iter, (abs(vel_roll-current_ee_vel[3])/vel_roll),
#                 #       (abs(vel_pitch-current_ee_vel[4])/vel_pitch),
#                 #       (abs(vel_yaw-current_ee_vel[5])/vel_yaw))

#             dx = np.array([vel_x, vel_y, vel_z, vel_roll, vel_pitch, vel_yaw])

#             # Compute dq at this time step, dq = inv(jacobian) * dx
#             dq = np.dot(np.linalg.inv(jacobian), dx)
#             dq_traj.append(np.transpose(dq))
#             # embed()
#             # print('timeiter ', time_iter)
#             # print('dq', np.transpose(dq).reshape([6,]))
#             # print('q_dot', robot.arm.servo_simulator.q_dot)
#             # make sure in velocity is in limits
#             joint_vel_limits = robot.arm.GetVelocityLimits()
#             ratio = np.absolute(dq / joint_vel_limits)
#             if np.max(ratio) > 0.95:
#                 # dq /= np.max(ratio) / 0.95
#                 raise Exception('Execeeding joint_vel_limits', np.max(ratio))

#             # Log state
#             dof_current = robot.arm.GetDOFValues().reshape((6, 1))
#             joint_traj.append(dof_current)
#             curr_ee_transform, curr_6dofs, curr_joint_values = get_current_configurations(robot)
#             # print('timeiter ', time_iter)
#             # print('droll ', current_ee_vel[3], 'roll ', curr_6dofs[3],
#             #       'dpit ', current_ee_vel[4], 'pit ', curr_6dofs[4],
#             #       'dyaw', current_ee_vel[5], 'yaw ', curr_6dofs[5])
#             x_traj.append(curr_6dofs[0])
#             y_traj.append(curr_6dofs[1])
#             z_traj.append(curr_6dofs[2])
#             roll_traj.append(curr_6dofs[3])
#             pitch_traj.append(curr_6dofs[4])
#             yaw_traj.append(curr_6dofs[5])

#             # print('time iter = ', time_iter)  # , 'roll = ', curr_6dofs[3])  # , ' droll = ', current_ee_vel[3])

#             if joint_velocity_mode:
#                 dq = np.array([float(num) for num in dq])
#                 robot.arm.Servo(dq)
#             else:
#                 delta_q = dq * dt * time_intervals
#                 dof_next = np.add(dof_current, delta_q)
#                 dof_next = [float(num) for num in dof_next]
#                 ##################### sth not correct with SetDOFValues ?????? roll channel is not set correctly #######
#                 robot.arm.SetDOFValues(dof_next)

#                 # dof_min, dof_max = robot.GetDOFLimits()
#                 # dof_max_vels = robot.GetDOFMaxVel()
#                 # arm_indices = robot.arm.GetArmIndices()
#                 # currentJoints = robot.GetDOFValues(arm_indices)
#                 # nextJoints = np.add(dof_current, delta_q)
#                 # nextJoints = [float(num) for num in nextJoints]
#                 # traj = openravepy.RaveCreateTrajectory(env, '')
#                 # spec = robot.GetActiveConfigurationSpecification()
#                 # timeoffset = spec.AddDeltaTimeGroup()
#                 # traj.Init(spec)
#                 # traj.Insert(0, np.concatenate((currentJoints, [0.])))
#                 # traj.Insert(1, np.concatenate((nextJoints, [0.])))
#                 # g = ConfigurationSpecification.Group()
#                 # g.name = 'deltatime'
#                 # g.dof = 1
#                 # traj.Insert(1, [dt * time_intervals], g, True)  # Set duration of traj
#                 # # openravepy.planningutils.RetimeActiveDOFTrajectory(traj, robot)
#                 # robot.ExecuteTrajectory(traj)

#             r_outer.sleep()

#         robot.arm.Servo(np.zeros(len(robot.arm.GetDOFValues())))

#         print('Executing reconstructed grasp done')
#         return joint_traj, dq_traj, x_traj, y_traj, z_traj, roll_traj, pitch_traj, yaw_traj, dx_traj, dy_traj, \
#             dz_traj, droll_traj, dpitch_traj, dyaw_traj, roll_roll_traj


# def execute_reconstructed_trajectories_close_loop(MODE, robot, env,
#                                                   dt, time_intervals,
#                                                   DMP_collections, num_dofs, time_steps,
#                                                   home_6dofs, goals_for_DMP,
#                                                   planned_trajectories, simulated):
#     n = robot.GetDOF()
#     dof_lim = robot.GetDOFLimits()
#     vel_lim = robot.GetDOFVelocityLimits()
#     robot.SetDOFLimits(-10000 * np.ones(n), 10000 * np.ones(n))
#     robot.SetDOFVelocityLimits(1 * vel_lim)  # 100* original

#     joint_traj, dq_traj = [], []
#     x_traj, y_traj, z_traj = [], [], []
#     roll_traj, pitch_traj, yaw_traj = [], [], []
#     cs_track = []  # canonical system
#     dx_traj, dy_traj, dz_traj = [], [], []
#     droll_traj, dpitch_traj, dyaw_traj = [], [], []

#     DMPs = []
#     for idx_dof in range(num_dofs):
#         dmp_this_dof = DMP_collections[idx_dof]
#         dmp_this_dof.y_goal[0] = goals_for_DMP[idx_dof]
#         dmp_this_dof.reset_state()
#         if idx_dof < 3:
#             dmp_this_dof.y[0] = 0.0
#         DMPs.append(dmp_this_dof)

#     # Execute learned and then reconstructed trajectories
#     r_outer = rospy.Rate(int(1.0 / (dt * time_intervals)))
#     for time_iter in range(0, time_steps-1, time_intervals):
#         # print('time iter = ', time_iter)
#         p_x, vel_x = planned_trajectories[0][0][0][time_iter], planned_trajectories[1][0][0][time_iter]
#         p_y, vel_y = planned_trajectories[0][0][1][time_iter], planned_trajectories[1][0][1][time_iter]
#         p_z, vel_z = planned_trajectories[0][0][2][time_iter], planned_trajectories[1][0][2][time_iter]
#         p_roll, vel_roll = planned_trajectories[0][0][3][time_iter], planned_trajectories[1][0][3][time_iter]
#         p_pitch, vel_pitch = planned_trajectories[0][0][4][time_iter], planned_trajectories[1][0][4][time_iter]
#         p_yaw, vel_yaw = planned_trajectories[0][0][5][time_iter], planned_trajectories[1][0][5][time_iter]
#         planned_pos = [p_x, p_y, p_z, p_roll, p_pitch, p_yaw]

#         current_ee_transform, current_6dofs, current_joint_values = get_current_configurations(robot)
#         jacobian_spatial = robot.arm.CalculateJacobian()
#         jacobian_angular = robot.arm.CalculateAngularVelocityJacobian()
#         jacobian = np.vstack((jacobian_spatial, jacobian_angular))

#         if robot.simulated:
#             current_joint_vel = robot.arm.servo_simulator.q_dot  # np array (6,)
#             current_ee_vel = np.dot(jacobian, np.array(current_joint_vel))  # check format
#             dx_traj.append(current_ee_vel[0])
#             dy_traj.append(current_ee_vel[1])
#             dz_traj.append(current_ee_vel[2])
#             droll_traj.append(current_ee_vel[3])
#             dpitch_traj.append(current_ee_vel[4])
#             dyaw_traj.append(current_ee_vel[5])
#             print('time iter = ', time_iter)
#             # print('time iter = ', time_iter, (current_ee_vel[3], vel_roll[0]), (current_ee_vel[4], vel_pitch[0]),
#             #                                  (current_ee_vel[5], vel_yaw[0]))

#         dx = []
#         for idx_dof in range(num_dofs):
#             if robot.simulated:
#                 y_feedback, dy_feedback = current_6dofs[idx_dof], current_ee_vel[idx_dof]
#             else:
#                 y_feedback, dy_feedback = current_6dofs[idx_dof], 0.
#             if MODE == 'six_dofs_min_jerk':
#                 y_feedback = y_feedback - home_6dofs[idx_dof]
#             if MODE == 'user_translation_minjerk_orientation':
#                 if idx_dof >= 3:  # Orientation is minjerk
#                     y_feedback = y_feedback - home_6dofs[idx_dof]

#             # Construct corresponding trajectory
#             for i in range(time_intervals):
#                 pos, vel, acc, cs_x = DMPs[idx_dof].construct_trajectory_one_step_with_feedback(y_feedback,
#                                                                                           dy_feedback,
#                                                                                           planned_pos[idx_dof][0],
#                                                                                           simulated)
#             dx.append(vel)

#         cs_track.append(cs_x)

#         dx = np.array(dx)

#         # Compute dq at this time step, dq = inv(jacobian) * dx

#         dq = np.dot(np.linalg.inv(jacobian), dx)
#         dq_traj.append(np.transpose(dq))

#         # # make sure in velocity is in limits
#         joint_vel_limits = robot.arm.GetVelocityLimits()
#         ratio = np.absolute(dq / joint_vel_limits)
#         if np.max(ratio) > 0.95:
#             # slow down while preserving the same final result in each joint
#             dq /= np.max(ratio) / 0.95
#             raise Exception('Execeeding joint_vel_limits', np.max(ratio))

#         dof_current = robot.arm.GetDOFValues().reshape((6, 1))
#         joint_traj.append(dof_current)
#         x_traj.append(current_6dofs[0])
#         y_traj.append(current_6dofs[1])
#         z_traj.append(current_6dofs[2])
#         roll_traj.append(current_6dofs[3])
#         pitch_traj.append(current_6dofs[4])
#         yaw_traj.append(current_6dofs[5])

#         dq = np.array([float(num) for num in dq])
#         robot.arm.Servo(dq)
#         print('\n')
#         r_outer.sleep()

#     robot.arm.Servo(np.zeros(len(robot.arm.GetDOFValues())))

#     return joint_traj, dq_traj, x_traj, y_traj, z_traj, roll_traj, pitch_traj, yaw_traj, cs_track, dx_traj, dy_traj, \
#             dz_traj, droll_traj, dpitch_traj, dyaw_traj


# def test_or_traj(robot, env):
#     arm_indices = robot.arm.GetArmIndices()
#     curLoc = robot.GetDOFValues(arm_indices)
#     goal_joint_values = copy.deepcopy(curLoc)
#     goal_joint_values = [float(num) for num in goal_joint_values]
#     goal_joint_values[4] -= 0.5
#     goal_joint_values = np.array(goal_joint_values)
#     traj = openravepy.RaveCreateTrajectory(env, '')
#     spec = robot.GetActiveConfigurationSpecification()
#     timeoffset = spec.AddDeltaTimeGroup()
#     traj.Init(spec)
#     traj.Insert(0, np.concatenate((curLoc, [0.])))
#     traj.Insert(1, np.concatenate((goal_joint_values, [0.])))
#     g = ConfigurationSpecification.Group()
#     g.name = 'deltatime'
#     g.dof = 1
#     traj.Insert(1, [5.0], g, True)  # Set duration of traj
#     # openravepy.planningutils.RetimeActiveDOFTrajectory(traj, robot)
#     # embed()
#     robot.ExecuteTrajectory(traj)


# def plotting(robot, planned_trajectories, joint_traj, dq_traj, x_traj, y_traj, z_traj,
#              roll_traj, pitch_traj, yaw_traj, cs_track, dx_traj, dy_traj,
#             dz_traj, droll_traj, dpitch_traj, dyaw_traj,
#              plot_sim, plot_comparison_sim_real, plot_full_dmp,
#              MODE, time_intervals, home_6dofs):
#     if plot_sim:
#         if robot.simulated:
#             save_joint_traj_path = '/home/calvinqiao/fun-workspace/src/ada/adapy/scripts/simulation_joint_traj.pkl'
#             save_dq_traj_path = '/home/calvinqiao/fun-workspace/src/ada/adapy/scripts/simulation_dq_traj.pkl'
#         else:
#             save_joint_traj_path = '/home/calvinqiao/fun-workspace/src/ada/adapy/scripts/real_robot_joint_traj.pkl'
#             save_dq_traj_path = '/home/calvinqiao/fun-workspace/src/ada/adapy/scripts/real_robot_dq_traj.pkl'
#         with open(save_joint_traj_path, 'wb') as save_file:
#             dill.dump(joint_traj, save_file)
#         save_file.close()
#         with open(save_dq_traj_path, 'wb') as save_file:
#             dill.dump(dq_traj, save_file)
#         save_file.close()

#     vel_x = planned_trajectories[1][0][0]
#     vel_y = planned_trajectories[1][0][1]
#     vel_z = planned_trajectories[1][0][2]
#     ang_vel_roll = planned_trajectories[1][0][3]
#     ang_vel_pitch = planned_trajectories[1][0][4]
#     ang_vel_yaw = planned_trajectories[1][0][5]
#     plt.figure(1, figsize=(6*3, 4*3))
#     plt.subplot(321)
#     if robot.simulated:
#         plt.plot(dx_traj, lw=2, label='dx')
#         plt.plot(dy_traj, lw=2, label='dy')
#         plt.plot(dz_traj, lw=2, label='dz')
#     plt.plot(vel_x[::time_intervals], 'r--', lw=1, label='target')
#     plt.plot(vel_y[::time_intervals], 'r--', lw=1)
#     plt.plot(vel_z[::time_intervals], 'r--', lw=1)
#     plt.legend(loc='lower left')
#     plt.subplot(322)
#     if robot.simulated:
#         plt.plot(droll_traj, lw=2, label='droll')
#         plt.plot(dpitch_traj, lw=2, label='dpitch')
#         plt.plot(dyaw_traj, lw=2, label='dyaw')
#     plt.plot(ang_vel_roll[::time_intervals], 'r--', lw=1, label='target')
#     plt.plot(ang_vel_pitch[::time_intervals], 'r--', lw=1)
#     plt.plot(ang_vel_yaw[::time_intervals], 'r--', lw=1)
#     plt.legend(loc='lower left')

#     print('Plotting joint trajectories and joint velocity trajectories')
#     # Plotting actual joint traj
#     if plot_comparison_sim_real:
#         if robot.simulated:
#             # Plot simulation joint trajectories
#             # plt.figure(3, figsize=(6, 4))
#             plt.subplot(323)
#             if plot_sim:
#                 for i in range(6):
#                     joint = [float(num[i]) for num in joint_traj]
#                     if i == 0:
#                         text = 'simulated'
#                         plt.plot(joint, 'r--', lw=2, label=text)
#                     else:
#                         plt.plot(joint, 'r--', lw=2)

#             save_joint_traj_path = '/home/calvinqiao/fun-workspace/src/ada/adapy/scripts/real_robot_joint_traj.pkl'
#             f = open(save_joint_traj_path, 'rb')
#             real_traj = dill.load(f)
#             f.close()

#             # Plot real robot joint trajectories
#             for i in range(6):
#                 joint = [float(num[i]) for num in real_traj]
#                 if i == 0:
#                     text = 'j0'
#                 elif i == 1:
#                     text = 'j1'
#                 elif i == 2:
#                     text = 'j2'
#                 elif i == 3:
#                     text = 'j3'
#                 elif i == 4:
#                     text = 'j4'
#                 elif i == 5:
#                     text = 'j5'
#                 plt.plot(joint, lw=2, label=text)
#             plt.legend(loc='upper right', ncol=3)

#             #plt.figure(4, figsize=(6, 4))
#             plt.subplot(324)
#             if plot_sim:
#                 for i in range(6):
#                     dq = [elem[0][i] for elem in dq_traj]
#                     if i == 0:
#                         text = 'simulated'
#                         plt.plot(dq, 'r--', lw=2, label=text)
#                     else:
#                         plt.plot(dq, 'r--', lw=2)

#             save_dq_traj_path = '/home/calvinqiao/fun-workspace/src/ada/adapy/scripts/real_robot_dq_traj.pkl'
#             f = open(save_dq_traj_path, 'rb')
#             real_dq_traj = dill.load(f)
#             f.close()

#             # Plot real robot joint trajectories
#             for i in range(6):
#                 dq = [elem[0][i] for elem in real_dq_traj]
#                 if i == 0:
#                     text = 'dj0'
#                 elif i == 1:
#                     text = 'dj1'
#                 elif i == 2:
#                     text = 'dj2'
#                 elif i == 3:
#                     text = 'dj3'
#                 elif i == 4:
#                     text = 'dj4'
#                 elif i == 5:
#                     text = 'dj5'
#                 plt.plot(dq, lw=2, label=text)
#             plt.legend(loc='lower right', ncol=2)
#         else:
#             plt.subplot(323)  # plt.figure(3, figsize=(6, 4))
#             # Plot real robot joint trajectories
#             for i in range(6):
#                 joint = [float(num[i]) for num in joint_traj]
#                 if i == 0:
#                     text = 'j0'
#                 elif i == 1:
#                     text = 'j1'
#                 elif i == 2:
#                     text = 'j2'
#                 elif i == 3:
#                     text = 'j3'
#                 elif i == 4:
#                     text = 'j4'
#                 elif i == 5:
#                     text = 'j5'
#                 plt.plot(joint, lw=2, label=text)

#             if plot_sim:
#                 save_joint_traj_path = '/home/calvinqiao/fun-workspace/src/ada/adapy/scripts/simulation_joint_traj.pkl'
#                 f = open(save_joint_traj_path, 'rb')
#                 sim_traj = dill.load(f)
#                 f.close()

#                 for i in range(6):
#                     joint = [float(num[i]) for num in sim_traj]
#                     if i == 0:
#                         text = 'simulated'
#                         plt.plot(joint, 'r--', lw=2, label=text)
#                     else:
#                         plt.plot(joint, 'r--', lw=2)

#             plt.legend(loc='upper right', ncol=3)

#             plt.subplot(324)  # plt.figure(4, figsize=(6, 4))
#             # Plot real robot joint trajectories
#             for i in range(6):
#                 dq = [elem[0][i] for elem in dq_traj]
#                 if i == 0:
#                     text = 'dj0'
#                 elif i == 1:
#                     text = 'dj1'
#                 elif i == 2:
#                     text = 'dj2'
#                 elif i == 3:
#                     text = 'dj3'
#                 elif i == 4:
#                     text = 'dj4'
#                 elif i == 5:
#                     text = 'dj5'
#                 plt.plot(dq, lw=2, label=text)

#             if plot_sim:
#                 save_dq_traj_path = '/home/calvinqiao/fun-workspace/src/ada/adapy/scripts/simulation_dq_traj.pkl'
#                 f = open(save_dq_traj_path, 'rb')
#                 sim_dq_traj = dill.load(f)
#                 f.close()

#                 for i in range(6):
#                     dq = [elem[0][i] for elem in sim_dq_traj]
#                     if i == 0:
#                         text = 'simulated'
#                         plt.plot(dq, 'r--', lw=2, label=text)
#                     else:
#                         plt.plot(dq, 'r--', lw=2)

#             plt.legend(loc='lower right', ncol=2)
#     else:
#         plt.subplot(323)  # plt.figure(3, figsize=(6, 4))
#         # Plot real robot joint trajectories
#         for i in range(6):
#             joint = [float(num[i]) for num in joint_traj]
#             if i == 0:
#                 text = 'j0'
#             elif i == 1:
#                 text = 'j1'
#             elif i == 2:
#                 text = 'j2'
#             elif i == 3:
#                 text = 'j3'
#             elif i == 4:
#                 text = 'j4'
#             elif i == 5:
#                 text = 'j5'
#             plt.plot(joint, lw=2, label=text)
#         plt.legend(loc='lower right', ncol=2)

#         plt.subplot(324)  # plt.figure(4, figsize=(6, 4))
#         # Plot real robot joint trajectories
#         for i in range(6):
#             dq = [elem[i] for elem in dq_traj]  # dq = [elem[0][i] for elem in dq_traj]
#             if i == 0:
#                 text = 'dj0'
#             elif i == 1:
#                 text = 'dj1'
#             elif i == 2:
#                 text = 'dj2'
#             elif i == 3:
#                 text = 'dj3'
#             elif i == 4:
#                 text = 'dj4'
#             elif i == 5:
#                 text = 'dj5'
#             plt.plot(dq, lw=2, label=text)
#         plt.legend(loc='lower right', ncol=2)

#     if plot_full_dmp:
#         target_x = planned_trajectories[0][0][0]
#         target_y = planned_trajectories[0][0][1]
#         target_z = planned_trajectories[0][0][2]
#         target_roll = planned_trajectories[0][0][3]
#         target_pitch = planned_trajectories[0][0][4]
#         target_yaw = planned_trajectories[0][0][5]
#     else:
#         target_x = planned_trajectories[0][0][0][-1]
#         target_y = planned_trajectories[0][0][1][-1]
#         target_z = planned_trajectories[0][0][2][-1]
#         target_roll = planned_trajectories[0][0][3][-1]
#         target_pitch = planned_trajectories[0][0][4][-1]
#         target_yaw = planned_trajectories[0][0][5][-1]

#     if MODE == 'six_dofs_min_jerk':
#         target_x += home_6dofs[0]
#         target_y += home_6dofs[1]
#         target_z += home_6dofs[2]
#         target_roll += home_6dofs[3]
#         target_pitch += home_6dofs[4]
#         target_yaw += home_6dofs[5]
#     elif MODE == 'user_translation_minjerk_orientation':
#         target_roll += home_6dofs[3]
#         target_pitch += home_6dofs[4]
#         target_yaw += home_6dofs[5]

#     plt.subplot(325)
#     plt.plot(x_traj, lw=2, label='x')
#     plt.plot(y_traj, lw=2, label='y')
#     plt.plot(z_traj, lw=2, label='z')
#     if plot_full_dmp:
#         plt.plot(target_x[::time_intervals], 'r--', lw=1, label='target')
#         plt.plot(target_y[::time_intervals], 'r--', lw=1)
#         plt.plot(target_z[::time_intervals], 'r--', lw=1)
#     else:
#         plt.plot((0, len(x_traj) - 1), (target_x, target_x), 'r--', lw=1, label='target')
#         plt.plot((0, len(y_traj) - 1), (target_y, target_y), 'r--', lw=1)
#         plt.plot((0, len(z_traj) - 1), (target_z, target_z), 'r--', lw=1)
#     plt.legend(loc='lower left', ncol=2)

#     plt.subplot(326)
#     plt.plot(roll_traj, lw=2, label='roll')
#     plt.plot(pitch_traj, lw=2, label='pitch')
#     plt.plot(yaw_traj, lw=2, label='yaw')
#     if plot_full_dmp:
#         plt.plot(target_roll[::time_intervals], 'r--', lw=1, label='target')
#         plt.plot(target_pitch[::time_intervals], 'r--', lw=1)
#         plt.plot(target_yaw[::time_intervals], 'r--', lw=1)
#     else:
#         plt.plot((0, len(roll_traj) - 1), (target_roll, target_roll), 'r--', lw=1, label='target')
#         plt.plot((0, len(pitch_traj) - 1), (target_pitch, target_pitch), 'r--', lw=1)
#         plt.plot((0, len(yaw_traj) - 1), (target_yaw, target_yaw), 'r--', lw=1)
#     plt.legend(loc='lower left', ncol=2)
#     # print('Plotting done')
#     # plt.figure(2, figsize=(6 * 3, 4 * 3))
#     # plt.plot(cs_track, lw=2)
#     plt.figure(2, figsize=(6 * 3, 4 * 3))
#     plt.plot(droll_traj, lw=2, label='droll')
#     plt.plot(ang_vel_roll[::time_intervals], 'r--', lw=1, label='target')
#     plt.legend()
#     plt.figure(3, figsize=(6 * 3, 4 * 3))
#     plt.plot(dpitch_traj, lw=2, label='dpitch')
#     plt.plot(ang_vel_pitch[::time_intervals], 'r--', lw=1, label='target')
#     plt.legend()
#     plt.figure(4, figsize=(6 * 3, 4 * 3))
#     plt.plot(dyaw_traj, lw=2, label='dyaw')
#     plt.plot(ang_vel_yaw[::time_intervals], 'r--', lw=1, label='target')
#     plt.legend()

#     plt.show()


# #
# def manipulateHandle(action):
#     # load correspoding DMP
#     pass