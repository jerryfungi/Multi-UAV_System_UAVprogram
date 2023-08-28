import numpy as np
from numpy import sin, cos, sqrt, pi, arctan2
import scipy.linalg as la
from communication_info import *


def PlusMinusPi(theta):
    if theta > pi:
        return theta - 2*pi
    elif theta < -pi:
        return theta + 2*pi
    else:
        return theta


class CraigReynolds_Path_Following(object):
    def __init__(self, method, recedingHorizon, path, path_window=3, Kp=1, Kd=5):
        self.method = method
        self.recedingHorizon = recedingHorizon  # (sec)
        self.pathWindow = path_window
        ' PID parameters '
        self.Kp = Kp
        self.Kd = Kd
        ' Path '
        self.path = path 


    def get_desirePoint_withWindow(self, v, x, y, theta, start_index):
        futurePoint = np.array([x + v*cos(theta)*self.recedingHorizon, y + v*sin(theta)*self.recedingHorizon])
        end_index = start_index + self.pathWindow if start_index + self.pathWindow < len(self.path) else len(self.path)-1
        d_optimal, desirePoint = 1e5, 0
        for i in range(start_index, end_index):
            a = np.array([self.path[i][0], self.path[i][1]])
            b = np.array([self.path[i+1][0], self.path[i+1][1]])

            va = futurePoint - a
            vb = b - a
            projection = np.dot(va, vb)/np.dot(vb, vb)*vb
            normalPoint = a + projection

            if not max(self.path[i][0], self.path[i + 1][0]) >= normalPoint[0] >= min(self.path[i][0], self.path[i+1][0]) or \
                    not max(self.path[i][1], self.path[i + 1][1]) >= normalPoint[1] >= min(self.path[i][1], self.path[i+1][1]):
                normalPoint = b
            d = np.linalg.norm(va - (normalPoint - a))
            if d < d_optimal:
                d_optimal = d
                desirePoint = normalPoint
                direct_projection = np.dot(vb, desirePoint - np.array([x, y]))
                index = i
        return desirePoint, index, d_optimal, direct_projection
    

    def get_desirePoint(self, v, x, y, theta):
        futurePoint = np.array([x + v*cos(theta)*self.recedingHorizon, y + v*sin(theta)*self.recedingHorizon])
        d_optimal = 1e5
        for i in range(len(self.path)-1):
            a = np.array([self.path[i][0], self.path[i][1]])
            b = np.array([self.path[i+1][0], self.path[i+1][1]])
            
            va = futurePoint - a
            vb = b - a
            projection = np.dot(va, vb)/np.dot(vb, vb)*vb
            normalPoint = a + projection

            if not max(self.path[i][0], self.path[i + 1][0]) >= normalPoint[0] >= min(self.path[i][0], self.path[i+1][0]) or \
                    not max(self.path[i][1], self.path[i + 1][1]) >= normalPoint[1] >= min(self.path[i][1], self.path[i+1][1]):
                normalPoint = b
            d = np.linalg.norm(va - (normalPoint -a))
            if d < d_optimal:
                d_optimal = d
                desirePoint = normalPoint
                index = i
        return desirePoint, index, d_optimal
    

    def get_desireVelocity(self, v, x, y, vx, vy):
        v_unitVector = np.array(vx, vy) / np.linalg.norm([vx, vy])
        futurePoint = np.add([x, y], v*v_unitVector*self.recedingHorizon)
        d_optimal = 1e5
        for i in range(len(self.path)-1):
            a = np.array([self.path[i][0], self.path[i][1]])
            b = np.array([self.path[i+1][0], self.path[i+1][1]])
            
            va = futurePoint - a
            vb = b - a
            projection = np.dot(va, vb)/np.dot(vb, vb)*vb
            normalPoint = a + projection

            if not max(self.path[i][0], self.path[i + 1][0]) > normalPoint[0] > min(self.path[i][0], self.path[i+1][0]) or \
                    not max(self.path[i][1], self.path[i + 1][1]) > normalPoint[1] > min(self.path[i][1], self.path[i+1][1]):
                normalPoint = b
            d = np.linalg.norm(va - (normalPoint -a))
            if d < d_optimal:
                d_optimal = d
                desirePoint = normalPoint
                index = i
        
        desire_v = np.substract(futurePoint, [x, y]) + np.substract(desirePoint, [x, y])
        return desire_v, index, d_optimal

    def bang_bang_control(self, v, Rmin, currentPosition, currentHeading, desirePoint, c=0):
        desireHeading = arctan2(desirePoint[1] - currentPosition[1], desirePoint[0] - currentPosition[0])
        relativeAngle = desireHeading - currentHeading  # Unit: [+- pi
        error_of_heading = relativeAngle if 2*abs(relativeAngle) <= 2*pi else -(relativeAngle/abs(relativeAngle))*(2*pi - abs(relativeAngle))
        if error_of_heading > c:
            u = 1
        elif error_of_heading < -c:
            u = -1
        else:
            u = 0
        return u * v * Rmin**-1, error_of_heading
    

    def PID_control(self, v, Rmin, currentPosition, currentHeading, desirePoint, pre_error=None):
        desireHeading = arctan2(desirePoint[1] - currentPosition[1], desirePoint[0] - currentPosition[0])
        relativeAngle = desireHeading - currentHeading  # Unit: [+- pi]
        error_of_heading = relativeAngle if abs(relativeAngle) <= pi else -(relativeAngle/abs(relativeAngle))*(2*pi - abs(relativeAngle))
        if not pre_error:
            ' P control '
            u = self.Kp * error_of_heading
        else:
            ' PD control '
            u = self.Kp * error_of_heading + self.Kd * (error_of_heading - pre_error)
        
        omega_max = v * Rmin**-1
        if u > omega_max:
            u = omega_max
        elif u < -omega_max:
            u = -omega_max
        return u, error_of_heading
 

    def LQR_control(self, position, heading, desirePoint, Q, R, Rmin, e, pe, pth_e, tv, v, dt=0.1):
        '''
        Return utheta, us, pe, pth_e
            referance: atsushisakai.github.io/PythonRobotics/
        '''
        desireHeading = arctan2(desirePoint[1] - position[1], desirePoint[0] - position[0])
        relativeAngle = desireHeading - heading  # Unit: [+- pi]
        th_e = relativeAngle if 2*abs(relativeAngle) <= 2*pi else -(relativeAngle/abs(relativeAngle))*(2*pi - abs(relativeAngle))
        
        # A = [1.0, dt , 0.0, 0.0, 0.0
        #      0.0, 0.0, v  , 0.0, 0.0
        #      0.0, 0.0, 1.0, dt , 0.0
        #      0.0, 0.0, 0.0, 0.0, 0.0
        #      0.0, 0.0, 0.0, 0.0, 1.0]
        A = np.zeros((5, 5))
        A[0, 0] = 1.0
        A[0, 1] = dt
        A[1, 2] = v
        A[2, 2] = 1.0
        A[2, 3] = dt
        A[4, 4] = 1.0

        # B = [0.0, 0.0
        #      0.0, 0.0
        #      0.0, 0.0
        #      v/R, 0.0
        #      0.0, dt ]
        B = np.zeros((5, 2))
        B[3, 0] = v / Rmin
        B[4, 1] = dt

        K, _, _ = dlqr(A, B, Q, R)

        # state vector
        # x = [e, dot_e, th_e, dot_th_e, delta_v]
        # e: lateral distance to the path
        # dot_e: derivative of e
        # th_e: angle difference to the path
        # dot_th_e: derivative of th_e
        # delta_v: difference between current speed and target speed
        x = np.zeros((5, 1))
        x[0, 0] = e
        x[1, 0] = (e - pe) / dt
        x[2, 0] = th_e
        x[3, 0] = (th_e - pth_e) / dt
        x[4, 0] = v - tv

        # input vector
        # u = [delta, accel]
        # delta: steering angle
        # accel: acceleration
        ustar = -K @ x

        # steering input
        delta = ustar[0, 0]

        # accel input
        accel = ustar[1, 0]

        return delta, accel, e, th_e


def solve_dare(A, B, Q, R):
        """
        solve a discrete time_Algebraic Riccati equation (DARE)
        """
        x = Q
        x_next = Q
        max_iter = 150
        eps = 0.01

        for i in range(max_iter):
            x_next = A.T @ x @ A - A.T @ x @ B @ \
                    la.inv(R + B.T @ x @ B) @ B.T @ x @ A + Q
            if (abs(x_next - x)).max() < eps:
                break
            x = x_next
        return x_next



def dlqr(A, B, Q, R):
        """Solve the discrete time lqr controller.
        x[k+1] = A x[k] + B u[k]
        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
        # ref Bertsekas, p.151
        """

        # first, try to solve the ricatti equation
        X = solve_dare(A, B, Q, R)

        # compute the LQR gain
        K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)

        eig_result = la.eig(A - B @ K)

        return K, X, eig_result[0]
