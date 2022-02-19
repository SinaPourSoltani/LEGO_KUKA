import matplotlib.pyplot as plt
import numpy as np
import roboticstoolbox as rtb
from spatialmath import *
from spatialmath.base import *
from roboticstoolbox import DHRobot, RevoluteDH
from sympy import im

#from kuka import Robot

def plot_trajectory(robot, qs):
    xs = []
    ys = []
    zs = []
    rs = []
    ps = []
    ws = []
    for i, q in enumerate(qs):
        T = robot.fkine(q)
        print(f"Step #{i+1}\n{T}")
        r, p, w = T.eul()
        x, y, z = T.t
        
        xs.append(x)
        ys.append(y)
        zs.append(z)
        rs.append(r)
        ps.append(p)
        ws.append(w)
    
    fig, axs = plt.subplots(3,2)
    fig.suptitle('Trajectory')

    data = [[xs, ys, zs], [rs, ps, ws]]
    y_label_units = ['[m]', '[rad]']
    titles = [['X', 'Y', 'Z'], ['Roll', 'Pitch', 'Yaw']]
    for j, col in enumerate(data):
        for i, el in enumerate(col):
            axs[i, j].set_title(titles[j][i])
            axs[i, j].set_ylabel('Displacement' + y_label_units[j])
            axs[i, j].plot(range(len(el)), el)
            if i == len(col)-1: 
                axs[i, j].set_xlabel('Steps')
            else:
                plt.setp(axs[i, j].get_xticklabels(), visible=False)
    plt.savefig('traj.png')
    plt.show()

m = 10
class LegoKuka(DHRobot):
    def __init__(self):
        super().__init__(
            [
                #d, a, alpha, theta
                RevoluteDH(d=m*0.0877, a=m*0.0159, alpha=np.deg2rad(0), offset=0),
                RevoluteDH(d=m*0.0, a=m*0.1116, alpha=np.deg2rad(180), offset=np.deg2rad(180)),
                RevoluteDH(d=m*0.0956, a=m*0.0399, alpha=np.deg2rad(-90), offset=0),
                RevoluteDH(d=m*0.0638, a=m*0.0, alpha=np.deg2rad(90), offset=0),
                RevoluteDH(d=m*0.0, a=m*0.0, alpha=np.deg2rad(-90), offset=0),
                RevoluteDH(d=m*0.0478, a=m*0.0, alpha=np.deg2rad(0), offset=0)
            ], name="LegoKuka"
        )

#rob = Robot()
#config = [90,10,10,10,10,10]
#rob.set_configuration(config, 50)

robot = LegoKuka()
home = np.array([0]*6) #[0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0])

traj0 = rtb.jtraj(robot.q, home, 30)

T1 = SE3(m*0.01,m*0.04,m*0.03) * SE3.Rz(90, unit='deg') * SE3.Ry(90, unit='deg')
sol1 = robot.ikine_LM(T1)
traj1 = rtb.jtraj(home, sol1.q, 30)

T2 = SE3(m*(0.07),m*(-0.06),m*0.02) * SE3.Ry(90, unit='deg')
sol2 = robot.ikine_LM(T2)
traj2 = rtb.jtraj(sol1.q, sol2.q, 30)

qs = np.concatenate((traj0.q, traj1.q, traj2.q))

s = np.array(list(map(lambda x: x*m, [-0.2, 0.2, -0.2, 0.2, 0, 0.4])))
plot_trajectory(robot, qs)

robot.plot(qs, limits = s, movie="kuka.gif")
