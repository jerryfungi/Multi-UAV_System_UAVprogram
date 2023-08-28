from struct import pack, unpack
from time import time, sleep
from enum import Enum
import numpy as np
import dubins
import pathFollowing as pf
from GA_SEAD_process import *


class packet_processing(object):
    def __init__(self, uav_id):
        self.uav_id = uav_id
        ' information among UAVs (SEAD)'
        self.uavs_info = [[] for _ in range(10)] # 10 info
        self.task_locking = []
      
    def pack_u2g_packet_default(self, mission, frameType, mode, armed, battery, timestamp, position, roll, pitch, yaw, speed):
        if type(mission) == Message_ID:
            msg_id = mission.value
        else:
            msg_id = 0
        return pack('<BBBBBBdiiiiiii', msg_id, self.uav_id, frameType.value, Mode[f"{mode}"].value, int(armed), int(battery), timestamp, 
                    int(position[0]*1e3), int(position[1]*1e3), int(position[2]*1e3), int(roll*1e6), int(pitch*1e6), int(yaw*1e6), int(speed*1e3))

    def pack_info_packet(self, statement):
        string_bytes = statement.encode()
        packet = bytearray([Message_ID.info.value, self.uav_id, len(string_bytes)])
        return packet + string_bytes

    def pack_record_time_packet(self, statement, time):
        string_bytes = statement.encode()
        packet = pack('<BBdB', Message_ID.Record_Time.value, self.uav_id, time, len(string_bytes))
        return packet + string_bytes
    
    def pack_SEAD_packet(self, uav_type, uav_velocity, uav_Rmin, UAV_pos, base_config, fix, cost, chromosome, tasks_completed, taregts_found):
        '''
            [id, type, v, Rmin, UAV pos, base config, Xtf, Nt, Nct, NnT, cost, chromosome, tasks completed, taregts found]
            UAV pos : (x,y,yaw)
            base config : (x_base, y_base, angle_base)
            Xtf : variable for task fixing 
            Nt : tasks number
            Nct : completed tasks number
            NnT : newly found targets number
            cost : objective value of the chromosome
            chromosome : current best solution
        '''
        if chromosome:
            Nt = len(chromosome[0])
        else:
            Nt = 0
        Nct = len(tasks_completed)
        NnT = len(taregts_found)
        packet = pack('<BBBiiiiiiiiBBBBi', Message_ID.SEAD.value, self.uav_id, uav_type, int(uav_velocity*1e3), int(uav_Rmin*1e3), 
                      int(UAV_pos[0]*1e3), int(UAV_pos[1]*1e3), int(UAV_pos[2]*1e3), 
                      int(base_config[0]*1e3), int(base_config[1]*1e3), int(base_config[2]*1e3), int(fix), Nt, Nct, NnT, int(cost*1e3))
        for gene in chromosome:
            packet += bytearray(gene)
        for task in tasks_completed:
            packet += bytearray(task)
        for target in taregts_found:
            packet += pack('<ii', int(target[0]*1e3), int(target[1]*1e3))
        ' Add the information of the UAV itself '
        self.uavs_info[0].append(self.uav_id)
        self.uavs_info[1].append(uav_type)
        self.uavs_info[2].append(uav_velocity)
        self.uavs_info[3].append(uav_Rmin)
        self.uavs_info[4].append(UAV_pos)
        self.uavs_info[5].append(base_config)
        self.uavs_info[6].append(cost)
        self.uavs_info[7].append(chromosome)
        self.uavs_info[8].extend(tasks_completed)
        self.uavs_info[9].extend(taregts_found)
        self.task_locking.append(fix)
        return packet, UAV_pos

    def SEAD_info_clear(self):
        self.uavs_info = [[] for _ in range(10)]
        self.task_locking = []

    def unpack_packet(self, packet):
        try:
            msg_id = Message_ID(packet[0])
        except ValueError:
            return Message_ID.info, "invalid message ID"

        ' Commands (G2U) '
        if msg_id == Message_ID.Arm:
            if packet[1] == self.uav_id:
                return Message_ID.Arm, Armed(packet[2])
            else:
                return Message_ID.info, f"Wrong UAV delegation on {Armed(packet[2]).name} command"
        elif msg_id == Message_ID.Mode_Change:
            if packet[1] == self.uav_id:
                return Message_ID.Mode_Change, Mode(packet[2])
            else:
                return Message_ID.info, "Wrong UAV delegation on change mode command"
        elif msg_id == Message_ID.Time_Synchromize:
            if packet[1] == self.uav_id:
                return Message_ID.Time_Synchromize, None
            else:
                return Message_ID.info, "time sync fail"
        elif msg_id == Message_ID.Takeoff:
            if packet[1] == self.uav_id:
                return Message_ID.Takeoff, packet[2]
            else:
                return Message_ID.info, "Wrong UAV delegation on takeoff command"
        elif msg_id == Message_ID.Mission_Abort:
            if packet[1] == self.uav_id:
                return Message_ID.Mission_Abort, None
            else:
                return Message_ID.info, "Wrong UAV delegation on mission abort command"
        elif msg_id == Message_ID.Origin_Correction:
            if packet[1] == self.uav_id:
                return Message_ID.Origin_Correction, packet[2]
            else:
                return Message_ID.info, "Wrong UAV delegation on origin correction"
        elif msg_id == Message_ID.Waypoints:
            if packet[1] == self.uav_id:
                try:
                    type = WaypointMissionMethod(packet[2])
                except ValueError:
                    return Message_ID, "invalid waypoints mission method"

                waypoint_radius = packet[3]
                if type == WaypointMissionMethod.guide_waypoint:
                    waypoint = np.multiply(unpack('iii', packet[4:]), 1e-3)
                    return Message_ID.Waypoints, [WaypointMissionMethod.guide_waypoint, waypoint_radius, waypoint]
                elif type == WaypointMissionMethod.guide_WPwithHeading:
                    waypoint = np.multiply(unpack('iiii', packet[4:]), 1e-3)
                    return Message_ID.Waypoints, [WaypointMissionMethod.guide_WPwithHeading, waypoint_radius, waypoint]
                elif type == WaypointMissionMethod.guide_waypoints:
                    WPs_num = packet[4]
                    wps_index = 5
                    waypoints = []
                    for i in range(WPs_num):
                        waypoints.append(np.multiply(unpack('iii', packet[wps_index+12*i:wps_index+12+12*i]), 1e-3))
                    return Message_ID.Waypoints, [WaypointMissionMethod.guide_waypoints, waypoint_radius, waypoints]    
                elif type == WaypointMissionMethod.CraigReynolds_Path_Following:
                    WPs_num = packet[4]
                    try:
                        method = pathFollowingMethod(packet[5])
                    except ValueError:
                        return Message_ID.info, "invalid path following method"
                        
                    recedingHorizon = packet[6] / 10
                    velocity = unpack('i', packet[7:11])[0] * 1e-3
                    Rmin = unpack('i', packet[11:15])[0] * 1e-3
                    path = []    
                    if method == pathFollowingMethod.path_following_velocityBody_PID:
                        Kp = unpack('i', packet[15:19])[0] * 1e-3
                        Kd = unpack('i', packet[19:23])[0] * 1e-3
                        wps_index = 23
                        for i in range(WPs_num):
                            wp = np.multiply(unpack('iii', packet[wps_index+12*i:wps_index+12+12*i]), 1e-3)
                            path.append(wp)
                        return Message_ID.Waypoints, [WaypointMissionMethod.CraigReynolds_Path_Following, waypoint_radius, 
                                                    pf.CraigReynolds_Path_Following(pathFollowingMethod.dubinsPath_following_velocityBody_PID, recedingHorizon, path, Kp=Kp, Kd=Kd), [velocity, Rmin]]
                    elif method == pathFollowingMethod.dubinsPath_following_velocityBody_PID:
                        Kp = unpack('i', packet[15:19])[0] * 1e-3
                        Kd = unpack('i', packet[19:23])[0] * 1e-3
                        wps_index = 23
                        for i in range(WPs_num):
                            wp = np.multiply(unpack('iiii', packet[wps_index+16*i:wps_index+16+16*i]), 1e-3)
                            path.append(wp)
                        path = generate_dubinsPath([[p[0], p[1], p[3]*np.pi/180] for p in path], Rmin, velocity/10)
                        return Message_ID.Waypoints, [WaypointMissionMethod.CraigReynolds_Path_Following, waypoint_radius, 
                                                    pf.CraigReynolds_Path_Following(pathFollowingMethod.dubinsPath_following_velocityBody_PID, recedingHorizon, path, Kp=Kp, Kd=Kd), [velocity, Rmin]]
                    else:
                        wps_index = 15
                        for i in range(WPs_num+1):
                            wp = np.multiply(unpack('iii', packet[wps_index+12*i:wps_index+12+12*i]), 1e-3)
                            path.append(wp)
                        return Message_ID.Waypoints, [WaypointMissionMethod.CraigReynolds_Path_Following, waypoint_radius, 
                                                      pf.CraigReynolds_Path_Following(method, recedingHorizon, path), [velocity, Rmin]]
            else:
                return Message_ID.info, f"Wrong UAV delegate on {WaypointMissionMethod(packet[3])} command"
        
        elif msg_id == Message_ID.Comm_u2gFreq:
            if packet[1] == self.uav_id:
                frequency = unpack('i', packet[2:])[0] * 1e-2
                return Message_ID.Comm_u2gFreq, frequency

        elif msg_id == Message_ID.SEAD_mission: 
            if packet[1] == self.uav_id:
                uav_type = packet[2]
                velocity = unpack('i', packet[3:7])[0] * 1e-3
                Rmin = unpack('i', packet[7:11])[0] * 1e-3
                waypoint_radius = packet[11]
                init_pos = list(np.multiply(unpack('iii', packet[12:24]), 1e-3))
                init_pos[2] = pf.PlusMinusPi(init_pos[2]*np.pi/180)
                end = list(np.multiply(unpack('iii', packet[24:36]), 1e-3))
                end[2] = end[2]*np.pi/180
                taregt_num, unknown_target_num = packet[36], packet[37]
                targets = [list(np.multiply(unpack('ii', packet[38+8*i:46+8*i]), 1e-3)) for i in range(taregt_num)]
                unknown_targets = [list(np.multiply(unpack('ii', packet[38+taregt_num*8+8*i:46+taregt_num*8+8*i]), 1e-3)) for i in range(unknown_target_num)]
                return Message_ID.SEAD_mission, [targets, unknown_targets, init_pos, end, [uav_type, velocity, Rmin], waypoint_radius]
            else:
                return Message_ID.info, "Wrong UAV delegate on SEAD mission"
        
        elif msg_id == Message_ID.SEAD:
            if packet[1] in self.uavs_info[0]:
                index = self.uavs_info[0],index(packet[1])
                self.uavs_info[4][index] = list(np.multiply(unpack('iii', packet[11:23]), 1e-3)) # uav config
                fix, Nt, Nct, NnT = bool(packet[35]), packet[36], packet[37], packet[38]
                self.uavs_info[6][index] = unpack('i', packet[39:43])[0]*1e-3 # cost
                chromosome = []
                if not Nt == 0:
                    for i in range(5):
                        chromosome.append(list(packet[43+i*Nt:43+Nt*(i+1)]))
                self.uavs_info[7][index] = chromosome
                for i in range(Nct):
                    self.uavs_info[8][index] = list(packet[43+Nt*5+i*2:43+Nt*5+(i+1)*2])
                for i in range(NnT):
                    self.uavs_info[9][index] = list(np.multiply(unpack('ii', packet[43+Nt*5+Nct*2+i*8:43+Nt*5+Nct*2+(i+1)*8]), 1e-3))
                self.task_locking[index] = fix
            else:
                self.uavs_info[0].append(packet[1]) # id
                self.uavs_info[1].append(packet[2]) # type
                self.uavs_info[2].append(unpack('i', packet[3:7])[0]*1e-3) # v
                self.uavs_info[3].append(unpack('i', packet[7:11])[0]*1e-3) # Rmin
                self.uavs_info[4].append(list(np.multiply(unpack('iii', packet[11:23]), 1e-3))) # uav config
                self.uavs_info[5].append(list(np.multiply(unpack('iii', packet[23:35]), 1e-3))) # base config
                fix, Nt, Nct, NnT = bool(packet[35]), packet[36], packet[37], packet[38]
                self.uavs_info[6].append(unpack('i', packet[39:43])[0]*1e-3) # cost
                chromosome = []
                if not Nt == 0:
                    for i in range(5):
                        chromosome.append(list(packet[43+i*Nt:43+Nt*(i+1)]))
                self.uavs_info[7].append(chromosome)
                for i in range(Nct):
                    self.uavs_info[8].append(list(packet[43+Nt*5+i*2:43+Nt*5+(i+1)*2]))
                for i in range(NnT):
                    self.uavs_info[9].append(list(np.multiply(unpack('ii', packet[43+Nt*5+Nct*2+i*8:43+Nt*5+Nct*2+(i+1)*8]), 1e-3)))
                self.task_locking.append(fix)
            return Message_ID.SEAD, None            


class Message_ID(Enum):
    Default = 0
    Mode_Change = 1
    Arm = 2
    Takeoff = 3
    Waypoints = 5
    Time_Synchromize = 4
    Comm_u2gFreq = 6
    Record_Time = 7
    Mission_Abort = 8
    Origin_Correction = 9
    info = 44
    SEAD = 17          # (U2U)
    SEAD_mission = 18  # (G2U)


class Mode(Enum):
    STABILIZE = 0
    ARCO = 1
    ALT_HOD = 2
    AUTO = 3
    GUIDED = 4
    LOITER = 5
    RTL = 6
    CIRCLE = 7
    POSITION = 8
    LAND = 9
    OF_LOITER = 10
    DRIFT = 11
    SPORT = 12
    FLIP = 14
    AUTOTUNE = 15
    POSHOLD = 16
    BRAKE = 17
    THROW = 18
    AVOID_ADSB = 19
    GUIDED_NOGPS = 20


class Armed(Enum):
    armed = 1
    disarmed = 0


class FrameType(Enum):
    Quad = 0
    Fixed_wing = 1


class WaypointMissionMethod(Enum):
    guide_waypoint = 0
    guide_WPwithHeading = 3
    guide_waypoints = 1
    CraigReynolds_Path_Following = 2


class pathFollowingMethod(Enum):
    path_following_position = 0
    path_following_position_yaw = 1
    path_following_velocityLocal =  2
    path_following_velocityBody_PID = 3
    dubinsPath_following_velocityBody_PID = 4


class XBee_Devices(Enum):
    # XBee Pro S3B (900mHz)
    UAV1 = "0013A20040D8DCD5"
    UAV2 = "0013A20040F5C61B"
    UAV3 = "0013A20040D8DCE4"
    UAV4 = "0013A20040F5C5E5"
    UAV5 = "0013A20040F5C5CA"
    UAV6 = "0013A20040F5C5DB"
    UAV7 = "0013A20040C19B66"


def generate_dubinsPath(points, radius, interval):
    dubins_path = [points[0]]
    for i in range(len(points)-1):
        if points[i][2] == points[i+1][2]:
            points[i+1][2] += 1e-3
        dubins_path.extend(dubins.shortest_path(points[i], points[i+1], radius).sample_many(interval)[0][1:])
    return dubins_path
