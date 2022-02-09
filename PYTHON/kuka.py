from pybricksdev import Motor
from math import *

class Robot:
    def __init__(self):
        self.joint1 = Motor('A')
        self.joint2 = Motor('C')
        self.joint3 = Motor('E')
        self.joint4 = Motor('D')
        self.joint5 = Motor('B')
        self.joint6 = Motor('F')

        self.joints = [self.joint1, self.joint2, self.joint3, self.joint4, self.joint5, self.joint6]

    def get_motor_rotations(self, config):
        m1 = 37.5 * config[0]
        m2 = 67.5 * config[1]
        m3 = 52.5 * config[2]
        m4 = 21 * config[3]
        m5 = 22.5 * config[4] + 5 * config[3]
        m6 = 12 * config[5] - 3 * config[3] + 5 * config[4]
        return list(map(round,[m1, m2, m3, m4, m5, m6]))

    def set_configuration(self, config, speed):
        m_rots = self.get_motor_rotations(config)
        max_rots = max(list(map(abs, m_rots)))
        for i, m_rot in enumerate(m_rots):
            m_speed = round(speed * m_rot / max_rots)
            m_rot = abs(m_rot) # direction already gived by sign of m_speed.
            print("Moving joint {} for {} rotations with {}% speed".format(i, m_rot, m_speed))
            self.joints[i].run_for_degrees(m_rot, m_speed)

if __name__ == '__main__':
    robot = Robot()
    config = [90,10,10,10,10,10]
    robot.set_configuration(config, 50)