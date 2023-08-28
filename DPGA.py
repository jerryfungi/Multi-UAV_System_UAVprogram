import random
import math
import numpy as np
import threading
import queue
from matplotlib import pyplot as plt
import multiprocessing as mp
import dubins
from GA_SEAD_process import *
from communication_info import *


def task_allocation_process(targets_sites, time_interval, pop_size, ga2control_queue, control2ga_queue):
    ga_population, update = None, True
    sead_mission = GA_SEAD(targets_sites, pop_size)
    uavs = control2ga_queue.get()
    while True:
        ' Optimization (for time_interval seconds)'
        solution, fitness_value, ga_population = sead_mission.run_GA_time_period_version(time_interval, uavs,
                                                                                         ga_population, update)
        ' Transmit the best solution and the corrsponding fitness value to the main process '
        ga2control_queue.put([fitness_value, solution])
        if not control2ga_queue.empty():
            ' Obtain the information from the main process '
            uavs = control2ga_queue.get()
            update = True
            ' Terminate the task allocation process '
            if uavs == [44]:
                break
        else:
            update = False


class UAV_Simulator(object):
    def __init__(self, uav_id, type, frame_type, velocity, Rmin, initial_position, base):
        ' Configuration of UAV '
        self.id = uav_id
        self.type = type
        self.v = velocity
        self.Rmin = Rmin
        self.omega_max = self.v / self.Rmin
        self.initial_position = initial_position
        self.base = base
        self.sencing_range = 50
        self.frame_type = frame_type
        ' Altitude and position of UAV '
        self.local_pose = [initial_position[0], initial_position[1], 10]
        self.yaw = initial_position[2]
        self.local_velo = [0, 0, 0]
        self.yaw_rate = 0
        ' Information for GCS '
        self.mode = Mode.GUIDED.name
        self.armed = False
        self.battery_perc = 100

    def step(self, v_cmd, yaw_cmd, dt):
        '''
        P controller for yaw rate and speed
                [ x(k+1)     ]   [ x(k) + v(k)cos(theta(k))dt ]
        state:  [ y(k+1)     ] = [ y(k) + v(k)sin(theta(k))dt ]
                [ theta(k+1) ]   [ theta(k) + utheta x dt     ]
                [ v(k+1)     ]   [ v(k) + us x dt             ]
        '''     
        self.local_pose[0] += self.local_velo[0] * np.cos(self.yaw) * dt
        self.local_pose[1] += self.local_velo[0] * np.sin(self.yaw) * dt
        self.yaw = pf.PlusMinusPi(self.yaw + self.yaw_rate * dt)
        self.local_velo[0] += 2 * (v_cmd - self.local_velo[0]) * dt
        self.yaw_rate = self.yaw_rate + 5 * (yaw_cmd - self.yaw_rate) * dt

    def set_mode(self, mode):
        self.mode = mode


class main_process(object):
    def __init__(self, targets_sites, unknown_targets, base_config, u2u_communication, ga2control_queue, control2ga_queue):
        ' SEAD mission '
        self.targets_set = targets_sites
        self.base = base_config
        ' Communication port '
        self.u2u = u2u_communication
        self.ga2control_queue = ga2control_queue
        self.control2ga_queue = control2ga_queue
        ' communication interval '
        self.T, self.T_comm = 2, 0.5
        ' PID controller '
        self.pre_error = None
        self.Kp = 3
        self.Kd = 7
        ' Predefine variables '
        self.packet, self.pos = [], []
        self.target = None
        self.fitness, self.best_solution = 1e-5, []
        self.terminated_tasks, self.new_targets = [], []
        self.previous_time_u2u, self.previous_time_control = 0, 0
        self.task_locking = False
        self.update = True
        self.into = False
        self.AT, self.NT = [], []
        self.back_to_base = False
        ' Path following '
        self.path_following = pf.CraigReynolds_Path_Following(pathFollowingMethod.dubinsPath_following_velocityBody_PID, 1.5, [], 3, self.Kp, self.Kd)
        self.intial_windowIndex = 0
        ' Dynamic environments '
        self.unknown_targets_set = unknown_targets
        self.sencing_range = 50

    def generate_path(self, chromosome, id, v, Rmin):
        path_route, task_sequence_state = [], []
        if chromosome and not self.back_to_base:
            for p in range(len(chromosome[0])):
                if chromosome[3][p] == id:
                    assign_target = chromosome[1][p]
                    assign_heading = chromosome[4][p] * 10
                    task_sequence_state.append([self.targets_set[assign_target - 1][0],
                                                self.targets_set[assign_target - 1][1], assign_heading,
                                                assign_target, chromosome[2][p]])
            task_sequence_state.append(self.base)
            for state in task_sequence_state[:-1]:
                state[2] *= np.pi / 180
            dubins_path = dubins.shortest_path(self.pos, task_sequence_state[0][:3], Rmin)
            path_route.extend(dubins_path.sample_many(v / 10)[0])
            for p in range(len(task_sequence_state)-1):
                sp = task_sequence_state[p][:3]
                gp = task_sequence_state[p+1][:3] if task_sequence_state[p][:3] != task_sequence_state[p+1][:3] else \
                    [task_sequence_state[p+1][0], task_sequence_state[p+1][1], task_sequence_state[p+1][2] - 1e-5]
                dubins_path = dubins.shortest_path(sp, gp, Rmin)
                path_route.extend(dubins_path.sample_many(v / 10)[0][1:])
            self.path_following.path, self.target = path_route, task_sequence_state
            ' Initiate the index on path'
            self.intial_windowIndex = 0

    def run_quadcopter(self, xbee, comm_info, uav_ros, new_timer, gcs, height, waypoint_radius):
        ' <<<<<<<<<< Communication Layer >>>>>>>>>> '
        
        ' Receive the solution from the task allocation process'
        while not self.ga2control_queue.empty():
            self.fitness, self.best_solution = self.ga2control_queue.get()

        ' Broadcast every T seceods'
        if new_timer.check_timer(self.T, self.previous_time_u2u, -0.1) and not self.back_to_base:
            self.previous_time_u2u = time.time()
            self.packet, self.pos = comm_info.pack_SEAD_packet(uav_ros.type, uav_ros.v, uav_ros.Rmin, 
                                                                  [uav_ros.local_pose[0], uav_ros.local_pose[1], uav_ros.yaw], 
                                                                  self.base, self.task_locking, 1/self.fitness, self.best_solution, 
                                                                  self.terminated_tasks, self.new_targets)
            # for agent in self.u2u:
            #     xbee.send_data_async(agent, self.packet)
            xbee.send_data_broadcast(self.packet)
            # self.terminated_tasks, self.new_targets = [], []

        ' Receive the information of UAVs after Tcomm seconds '
        if new_timer.check_period(self.T_comm, self.previous_time_u2u) and self.packet:
            self.AT.extend(comm_info.uavs_info[8])
            self.NT.extend(comm_info.uavs_info[9])
            comm_info.uavs_info[8], comm_info.uavs_info[9] = self.AT, self.NT
            for target_found in self.NT:
                if target_found not in self.targets_set:
                    self.targets_set.append(target_found)

            ' Check the task-locking mechanism is activate or not '
            if not any(comm_info.task_locking):
                print(comm_info.uavs_info)
                ' transmit the information of UAVs(packets received) to the task allocation process'
                self.control2ga_queue.put(comm_info.uavs_info)

                self.AT, self.NT = [], []
                if self.update:
                    '''
                    => Choose the solution with the lowest cost value and generate the path corresponding to the UAV ID.
                            (When it comes to the same fitness value, choose the solution with the smaller UAV ID)
                    '''
                    self.generate_path(sorted(sorted(zip(comm_info.uavs_info[0], comm_info.uavs_info[6], comm_info.uavs_info[7]), key=lambda x: x[0]), key=lambda x: x[1])[0][-1], 
                                       comm_info.uav_id, uav_ros.v, uav_ros.Rmin)
                self.update = True
            else:
                self.update = False
            self.packet = []
            comm_info.SEAD_info_clear()

        ' <<<<<<<<<< Control Layer >>>>>>>>>> '
        if self.path_following.path:
            ' Identify the target is reached or not '
            if np.linalg.norm([self.target[0][0] - uav_ros.local_pose[0], self.target[0][1] - uav_ros.local_pose[1]]) <= waypoint_radius and not self.into:
                ' Back to the base or not '
                if self.target[:-1] == []:
                    xbee.send_data_async(gcs, comm_info.pack_record_time_packet(f"mission complete!", new_timer.t()))
                    ' Shutdown the task allocation process '
                    self.control2ga_queue.put([44])
                self.into = True

            ' 做完任務之條件 '
            if np.linalg.norm([self.target[0][0] - uav_ros.local_pose[0], self.target[0][1] - uav_ros.local_pose[1]]) >= waypoint_radius and self.into:
                if self.target[0][3:]:
                    self.terminated_tasks.append(self.target[0][3:])
                    xbee.send_data_async(gcs, comm_info.pack_record_time_packet(f"task: {self.target[0][3:]} finished", new_timer.t()))
                    del self.target[0]
                    self.task_locking = False
                    self.into = False
                    
            ' Task-locking mechanism '
            if np.linalg.norm([self.target[0][0] - uav_ros.local_pose[0], self.target[0][1] - uav_ros.local_pose[1]]) <= 2*uav_ros.Rmin:
                if self.target[0][3:]:
                    self.task_locking = True
                else:
                    self.back_to_base = True
            
            '''
            Path-following Algorithm
                    Steering Control (10 Hz)
            '''
            if new_timer.check_period(0.1, self.previous_time_control):
                desirePoint, self.intial_windowIndex, _, error_of_distance = self.path_following.get_desirePoint_withWindow(uav_ros.v, uav_ros.local_pose[0], uav_ros.local_pose[1], uav_ros.yaw, self.intial_windowIndex)
                if error_of_distance <= 0 and self.back_to_base:
                    ' Stop when arriving at the base '
                    target_V, u = 0, 0
                elif self.back_to_base:
                    ' Decrease the velocity when approching the base => (P control)'
                    pid_velo = 0.8 * np.linalg.norm([self.base[0] - uav_ros.local_pose[0], self.base[1] - uav_ros.local_pose[1]])
                    target_V = pid_velo if pid_velo < uav_ros.v else uav_ros.v
                    u, self.pre_error = self.path_following.PID_control(uav_ros.v, uav_ros.Rmin, uav_ros.local_pose, uav_ros.yaw, desirePoint, self.pre_error)
                else:
                    ' Vx(Body frame) and Yaw rate comands => (PD control)'
                    u, self.pre_error = self.path_following.PID_control(uav_ros.v, uav_ros.Rmin, uav_ros.local_pose, uav_ros.yaw, desirePoint, self.pre_error)
                    target_V = uav_ros.v
                ' Altitude hold => (P control)'
                v_z = 0.3 * (height - uav_ros.local_pose[2])
                uav_ros.velocity_bodyFrame_control(target_V, u, v_z)
                self.previous_time_control = time.time()

            ' Unknown targets detection '
            for t in self.unknown_targets_set:
                if np.linalg.norm([uav_ros.local_pose[0] - t[0], uav_ros.local_pose[1] - t[1]]) <= self.sencing_range and t not in self.targets_set and uav_ros.type != 3:
                    self.targets_set.append(t)
                    self.new_targets.append(t)
                    xbee.send_data_async(gcs, comm_info.pack_record_time_packet(f"discover unknown target: {t}", new_timer.t()))
    
    def run_fixedWing(self, xbee, comm_info, uav_ros, new_timer, gcs, height, waypoint_radius):
        while not self.ga2control_queue.empty():
            self.fitness, self.best_solution = self.ga2control_queue.get()

        if new_timer.check_timer(self.T, self.previous_time_u2u, -0.1) and not self.back_to_base:
            self.previous_time_u2u = time.time()
            self.packet, self.pos = comm_info.pack_SEAD_packet(uav_ros.type, uav_ros.v, uav_ros.Rmin, 
                                                                  [uav_ros.local_pose[0], uav_ros.local_pose[1], uav_ros.yaw], 
                                                                  self.base, self.task_locking, 1/self.fitness, self.best_solution, 
                                                                  self.terminated_tasks, self.new_targets)
            # for agent in self.u2u:
            #     xbee.send_data_async(agent, self.packet)
            xbee.send_data_broadcast(self.packet)
            # self.terminated_tasks, self.new_targets = [], []

        if new_timer.check_period(self.T_comm, self.previous_time_u2u) and self.packet:
            self.AT.extend(comm_info.uavs_info[8])
            self.NT.extend(comm_info.uavs_info[9])
            comm_info.uavs_info[8], comm_info.uavs_info[9] = self.AT, self.NT
            for target_found in self.NT:
                if target_found not in self.targets_set:
                    self.targets_set.append(target_found)
            if not any(comm_info.task_locking):
                print(comm_info.uavs_info)
                self.control2ga_queue.put(comm_info.uavs_info)
                self.AT, self.NT = [], []
                if self.update:
                    self.generate_path(sorted(sorted(zip(comm_info.uavs_info[0], comm_info.uavs_info[6], comm_info.uavs_info[7]), key=lambda x: x[0]), key=lambda x: x[1])[0][-1], 
                                       comm_info.uav_id, uav_ros.v, uav_ros.Rmin)
                self.update = True
            else:
                self.update = False
            self.packet = []
            comm_info.SEAD_info_clear()

        if self.path_following.path:
            if np.linalg.norm([self.target[0][0] - uav_ros.local_pose[0], self.target[0][1] - uav_ros.local_pose[1]]) <= waypoint_radius and not self.into:
                if self.target[:-1] == []:
                    xbee.send_data_async(gcs, comm_info.pack_record_time_packet(f"mission complete!", new_timer.t()))
                    self.control2ga_queue.put([44])
                self.into = True
            if np.linalg.norm([self.target[0][0] - uav_ros.local_pose[0], self.target[0][1] - uav_ros.local_pose[1]]) >= waypoint_radius and self.into:
                if self.target[0][3:]:
                    self.terminated_tasks.append(self.target[0][3:])
                    xbee.send_data_async(gcs, comm_info.pack_record_time_packet(f"task: {self.target[0][3:]} finished", new_timer.t()))
                    del self.target[0]
                    self.task_locking = False
                    self.into = False
            if np.linalg.norm([self.target[0][0] - uav_ros.local_pose[0], self.target[0][1] - uav_ros.local_pose[1]]) <= 2*uav_ros.Rmin:
                if self.target[0][3:]:
                    self.task_locking = True
                else:
                    self.back_to_base = True
            
            if new_timer.check_period(0.1, self.previous_time_control):
                desirePoint, self.intial_windowIndex, _, error_of_distance = self.path_following.get_desirePoint_withWindow(uav_ros.v, uav_ros.local_pose[0], uav_ros.local_pose[1], uav_ros.yaw, self.intial_windowIndex)
                if error_of_distance <= 0 and self.back_to_base:
                    uav_ros.set_mode(Mode.LOITER.name)
                else:
                    uav_ros.position_control(desirePoint[0], desirePoint[1], height)
                self.previous_time_control = time.time()

            for t in self.unknown_targets_set:
                if np.linalg.norm([uav_ros.local_pose[0] - t[0], uav_ros.local_pose[1] - t[1]]) <= self.sencing_range and t not in self.targets_set and uav_ros.type != 3:
                    self.targets_set.append(t)
                    self.new_targets.append(t)
                    xbee.send_data_async(gcs, comm_info.pack_record_time_packet(f"discover unknown target: {t}", new_timer.t()))

    def run_simulation(self, xbee, comm_info, uav_ros, new_timer, gcs, waypoint_radius):
        while not self.ga2control_queue.empty():
            self.fitness, self.best_solution = self.ga2control_queue.get()

        if new_timer.check_timer(self.T, self.previous_time_u2u, -0.1) and not self.back_to_base:
            self.previous_time_u2u = time.time()
            self.packet, self.pos = comm_info.pack_SEAD_packet(uav_ros.type, uav_ros.v, uav_ros.Rmin, 
                                                                [uav_ros.local_pose[0], uav_ros.local_pose[1], uav_ros.yaw], 
                                                                self.base, self.task_locking, 1/self.fitness, self.best_solution, 
                                                                self.terminated_tasks, self.new_targets)
            # for agent in self.u2u:
            #     xbee.send_data_async(agent, self.packet)
            xbee.send_data_broadcast(self.packet)
            # self.terminated_tasks, self.new_targets = [], []

        if new_timer.check_period(self.T_comm, self.previous_time_u2u) and self.packet:
            self.AT.extend(comm_info.uavs_info[8])
            self.NT.extend(comm_info.uavs_info[9])
            comm_info.uavs_info[8], comm_info.uavs_info[9] = self.AT, self.NT
            for target_found in self.NT:
                if target_found not in self.targets_set:
                    self.targets_set.append(target_found)
            if not any(comm_info.task_locking):
                self.control2ga_queue.put(comm_info.uavs_info)
                print(comm_info.uavs_info)
                self.AT, self.NT = [], []
                if self.update:
                    self.generate_path(sorted(sorted(zip(comm_info.uavs_info[0], comm_info.uavs_info[6], comm_info.uavs_info[7]), key=lambda x: x[0]), key=lambda x: x[1])[0][-1], 
                                       comm_info.uav_id, uav_ros.v, uav_ros.Rmin)
                self.update = True
            else:
                self.update = False
            self.packet = []
            comm_info.SEAD_info_clear()

        if self.path_following.path:
            if np.linalg.norm([self.target[0][0] - uav_ros.local_pose[0], self.target[0][1] - uav_ros.local_pose[1]]) <= waypoint_radius and not self.into:
                if self.target[:-1] == []:
                    xbee.send_data_async(gcs, comm_info.pack_record_time_packet(f"mission complete!", new_timer.t()))
                    self.control2ga_queue.put([44])
                self.into = True
            if np.linalg.norm([self.target[0][0] - uav_ros.local_pose[0], self.target[0][1] - uav_ros.local_pose[1]]) >= waypoint_radius and self.into:
                if self.target[0][3:]:
                    self.terminated_tasks.append(self.target[0][3:])
                    xbee.send_data_async(gcs, comm_info.pack_record_time_packet(f"task: {self.target[0][3:]} finished", new_timer.t()))
                    del self.target[0]
                    self.task_locking = False
                    self.into = False
            if np.linalg.norm([self.target[0][0] - uav_ros.local_pose[0], self.target[0][1] - uav_ros.local_pose[1]]) <= 2*uav_ros.Rmin:
                if self.target[0][3:]:
                    self.task_locking = True
                else:
                    self.back_to_base = True
            
            if new_timer.check_period(0.1, self.previous_time_control):
                desirePoint, self.intial_windowIndex, _, error_of_distance = self.path_following.get_desirePoint_withWindow(uav_ros.v, uav_ros.local_pose[0], uav_ros.local_pose[1], uav_ros.yaw, self.intial_windowIndex)
                u, self.pre_error = self.path_following.PID_control(uav_ros.v, uav_ros.Rmin, uav_ros.local_pose, uav_ros.yaw, desirePoint, self.pre_error)
                if error_of_distance <= 0 and self.back_to_base:
                    target_V, u = 0, 0
                elif self.back_to_base:
                    pid_velo = 0.8 * np.linalg.norm([self.base[0] - uav_ros.local_pose[0], self.base[1] - uav_ros.local_pose[1]])
                    target_V = pid_velo if pid_velo < uav_ros.v else uav_ros.v
                else:
                    target_V = uav_ros.v
                dt = time.time() - self.previous_time_control if not self.previous_time_control == 0 else 0
                uav_ros.step(target_V, u, dt)
                self.previous_time_control = time.time()

            for t in self.unknown_targets_set:
                if np.linalg.norm([uav_ros.local_pose[0] - t[0], uav_ros.local_pose[1] - t[1]]) <= self.sencing_range and t not in self.targets_set and uav_ros.type != 3:
                    self.targets_set.append(t)
                    self.new_targets.append(t)
                    xbee.send_data_async(gcs, comm_info.pack_record_time_packet(f"discover unknown target: {t}", new_timer.t()))


