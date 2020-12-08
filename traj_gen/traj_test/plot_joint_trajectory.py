"""
1) read generated json file and plot all seven joints
2) separate file 
- subscribe to robot message and save as json (message name should be the 
same for the simulation and the real hardware)
3) read three different json files for joint trajectories and plot
"""

import json
import sys
import os
import copy
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from mpl_toolkits import mplot3d
        
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.common import FindResourceOrThrow
from pydrake.all import RigidTransform, RollPitchYaw

# from pathlib import Path
drake_path = "/home/zhigen/code/drake/"
iiwa_path = drake_path + "manipulation/models/iiwa_description/urdf/iiwa7_no_world_joint.urdf"
wsg_path = drake_path + "manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf"


JSON_FILENAME = drake_path + "traj_gen/trajectory_data/14_cmd_rd_0.json"
JSON_FILENAME_SIM = drake_path + "traj_gen/trajectory_data/14_rd_0.json"
JSON_FILENAME_HW = drake_path + "traj_gen/trajectory_data/14_vel_100_pos_f_5000.json"

REMOVE_LAST_STEP = False
USE_TORQUE = False
OFFSET = 1

class TrajPlotter(object):
    def __init__(self):
        with open(JSON_FILENAME) as fp:
            self.plan = json.load(fp)
        with open(JSON_FILENAME_SIM) as fpsim:
            self.plansim = json.load(fpsim)
        with open(JSON_FILENAME_HW) as fphw:
            self.planhw = json.load(fphw)

        self.time = self.plan[0]["times_sec"]
        self.joint = self.plan[0]["torques"]
        self.timesim = self.plansim["times_sec"]
        self.jointsim = self.plansim["joint_velocity_estimated"] #joint_position_measured
        self.timesimc = self.plansim["times_sec"]
        self.jointsimc = self.plansim["joint_position_commanded"]
        self.timehw = self.planhw["times_sec"]
        self.jointhw = self.planhw["joint_velocity_estimated"] #joint_position_measured
        self.timehwc = self.planhw["times_sec"]
        self.jointhwc = self.planhw["joint_position_measured"]

        # traj this is only the first motion so 3s
        j0, j1, j2, j3, j4, j5, j6 = ([] for i in range(7))
        self.jdict = {0:j0, 1:j1, 2:j2, 3:j3, 4:j4, 5:j5, 6:j6}
        self.tarray = []

        # hardware measured
        j0hw, j1hw, j2hw, j3hw, j4hw, j5hw, j6hw = ([] for i in range(7))
        self.jdicthw = {0:j0hw, 1:j1hw, 2:j2hw, 3:j3hw, 4:j4hw, 5:j5hw, 6:j6hw}
        self.tarrayhw = []

        # hardware commanded
        j0hwc, j1hwc, j2hwc, j3hwc, j4hwc, j5hwc, j6hwc = ([] for i in range(7))
        self.jdicthwc = {0:j0hwc, 1:j1hwc, 2:j2hwc, 3:j3hwc, 4:j4hwc, 5:j5hwc, 6:j6hwc}
        self.tarrayhwc = []

        # simulation measured
        j0sim, j1sim, j2sim, j3sim, j4sim, j5sim, j6sim = ([] for i in range(7))
        self.jdictsim = {0:j0sim, 1:j1sim, 2:j2sim, 3:j3sim, 4:j4sim, 5:j5sim, 6:j6sim}
        self.tarraysim = []

        # simulation commanded
        j0simc, j1simc, j2simc, j3simc, j4simc, j5simc, j6simc = ([] for i in range(7))
        self.jdictsimc = {0:j0simc, 1:j1simc, 2:j2simc, 3:j3simc, 4:j4simc, 5:j5simc, 6:j6simc}
        self.tarraysimc = []

        for t, j in zip(self.time,self.joint):
            if t <= OFFSET:
                self.tarray.append(t)    
                for i, nj in enumerate(j):
                    self.jdict[i].append(nj)

        for t, j in zip(self.timehw,self.jointhw):
            if t <= OFFSET:
                self.tarrayhw.append(t)    
                for i, nj in enumerate(j):
                    self.jdicthw[i].append(nj)

        for t, j in zip(self.timehwc,self.jointhwc):
            if t <= OFFSET:
                self.tarrayhwc.append(t)    
                for i, nj in enumerate(j):
                    self.jdicthwc[i].append(nj)

        for t, j in zip(self.timesim,self.jointsim):
            if t <= OFFSET:
                self.tarraysim.append(t)    
                for i, nj in enumerate(j):
                    self.jdictsim[i].append(nj)

        for t, j in zip(self.timesimc,self.jointsimc):
            if t <= OFFSET:
                self.tarraysimc.append(t)    
                for i, nj in enumerate(j):
                    self.jdictsimc[i].append(nj)

        self.tarrayhw[:] = [tahw - self.tarrayhw[0] for tahw in self.tarrayhw]
        self.tarrayhwc[:] = [tahwc - self.tarrayhwc[0] for tahwc in self.tarrayhwc]
        self.tarraysim[:] = [tasim - self.tarraysim[0] for tasim in self.tarraysim]
        self.tarraysimc[:] = [tasimc - self.tarraysimc[0] for tasimc in self.tarraysimc]


    def plotFK(self):
        # Create MBP model
        plant = MultibodyPlant(0.001)
        parser = Parser(plant)
        iiwa_model = parser.AddModelFromFile(iiwa_path)
        wsg_model = parser.AddModelFromFile(wsg_path)

        X_WI = RigidTransform.Identity()
        plant.WeldFrames(
            plant.world_frame(), 
            plant.GetFrameByName("iiwa_link_0", iiwa_model),
            X_WI
        )

        rpy = RollPitchYaw(np.pi/2, 0, np.pi/2)
        xyz = np.array([0, 0, 0.053]).astype(np.float64)
        X_IG = RigidTransform(rpy, xyz)
        plant.WeldFrames(
            plant.GetFrameByName("iiwa_link_7", iiwa_model),
            plant.GetFrameByName("body", wsg_model),
            X_IG
        )
        plant.Finalize()
        context = plant.CreateDefaultContext()

        # Set pose and get EE value
        # plant.SetPositions(
        #     context, 
        #     iiwa_model, 
        #     [-1.05973, 0.241475, 0.661869, -1.86468, 0.0, 0.0, 0.0]
        # )
        # ee_pose = plant.EvalBodyPoseInWorld(
        #     context,
        #     plant.GetBodyByName("iiwa_link_ee", iiwa_model)#wsg_model)
        # )
        # print (ee_pose.translation(), RollPitchYaw(ee_pose.rotation()).vector())
        # while True:
        #     a=1

        # =================================================
        # FK & truncate FK data for a timespan of interest
        # =================================================
        tlist, xlist, ylist, zlist = ([] for i in range(4))
        for t, j in zip(self.time, self.joint):
            plant.SetPositions(
                context, 
                iiwa_model, 
                j
            )
            ee_pose = plant.EvalBodyPoseInWorld(
                context,
                plant.GetBodyByName("left_finger", wsg_model)
            )
            tlist.append(t)
            xlist.append(ee_pose.translation()[0])
            ylist.append(ee_pose.translation()[1])
            zlist.append(ee_pose.translation()[2])

        tlistsim, xlistsim, ylistsim, zlistsim = ([] for i in range(4))
        for t, j in zip(self.timesim, self.jointsim):
            plant.SetPositions(
                context, 
                iiwa_model, 
                j
            )
            ee_pose = plant.EvalBodyPoseInWorld(
                context,
                plant.GetBodyByName("left_finger", wsg_model)
            )
            tlistsim.append(t)
            xlistsim.append(ee_pose.translation()[0])
            ylistsim.append(ee_pose.translation()[1])
            zlistsim.append(ee_pose.translation()[2])

        tlistsimc, xlistsimc, ylistsimc, zlistsimc = ([] for i in range(4))
        for t, j in zip(self.timesim, self.jointsimc):
            plant.SetPositions(
                context, 
                iiwa_model, 
                j
            )
            ee_pose = plant.EvalBodyPoseInWorld(
                context,
                plant.GetBodyByName("left_finger", wsg_model)
            )
            tlistsimc.append(t)
            xlistsimc.append(ee_pose.translation()[0])
            ylistsimc.append(ee_pose.translation()[1])
            zlistsimc.append(ee_pose.translation()[2])

        tlisthw, xlisthw, ylisthw, zlisthw = ([] for i in range(4))
        for t, j in zip(self.timehw, self.jointhw):
            if t>=OFFSET:
                plant.SetPositions(
                    context, 
                    iiwa_model, 
                    j
                )
                ee_pose = plant.EvalBodyPoseInWorld(
                    context,
                    plant.GetBodyByName("left_finger", wsg_model)
                )
                tlisthw.append(t)
                xlisthw.append(ee_pose.translation()[0])
                ylisthw.append(ee_pose.translation()[1])
                zlisthw.append(ee_pose.translation()[2])

        tlisthwc, xlisthwc, ylisthwc, zlisthwc = ([] for i in range(4))
        for t, j in zip(self.timehwc, self.jointhwc):
            if t>=OFFSET:
                plant.SetPositions(
                    context, 
                    iiwa_model, 
                    j
                )
                ee_pose = plant.EvalBodyPoseInWorld(
                    context,
                    plant.GetBodyByName("left_finger", wsg_model)
                )
                tlisthwc.append(t)
                xlisthwc.append(ee_pose.translation()[0])
                ylisthwc.append(ee_pose.translation()[1])
                zlisthwc.append(ee_pose.translation()[2])

        tlistsim[:] = [tasim - tlistsim[0] for tasim in tlistsim]
        tlistsimc[:] = [tasimc - tlistsimc[0] for tasimc in tlistsimc]
        tlisthw[:] = [tahw - tlisthw[0] for tahw in tlisthw]
        tlisthwc[:] = [tahwc - tlisthwc[0] for tahwc in tlisthwc]

        #Plot 2D
        # Cartesion position of EE 2D plot
        legend_properties = {'size': 18}#{'weight':'bold'}

        plt.figure()
        plt.subplots_adjust(left=None, bottom=None, right=None, top=None, wspace=None, hspace=0.4)
        # plt.suptitle("Desired vs Simulated vs Harware Trajectories of the End-Effector", fontsize=22)#, fontweight="bold")
        plt.subplot(311)
        plt.plot(tlist, xlist, label="traj")
        plt.plot(tlistsim, xlistsim, label="sim")
        plt.plot(tlistsimc, xlistsimc, label="simc")
        plt.plot(tlisthw, xlisthw, label="hw")
        plt.plot(tlisthwc, xlisthwc, label="hwc")
        plt.ylim([-1.5,1.5])
        plt.legend(loc="upper left", prop=legend_properties)
        plt.xlabel("seconds", fontsize=16)#, fontweight="bold")
        plt.ylabel("x [m]", fontsize=16)#, fontweight="bold")
        plt.xticks(fontsize=14)#, rotation=90)
        plt.yticks(fontsize=14)#, rotation=90)
        # plt.show()
        plt.subplot(312)
        plt.plot(tlist, ylist, label="traj")
        plt.plot(tlistsim, ylistsim, label="sim")
        plt.plot(tlistsimc, ylistsimc, label="simc")
        plt.plot(tlisthw, ylisthw, label="hw")
        plt.plot(tlisthwc, ylisthwc, label="hwc")
        plt.ylim([-1.5,1.5])
        plt.legend(loc="upper left", prop=legend_properties)
        plt.xlabel("seconds", fontsize=16)#, fontweight="bold")
        plt.ylabel("y [m]", fontsize=16)#, fontweight="bold")
        plt.xticks(fontsize=14)#, rotation=90)
        plt.yticks(fontsize=14)#, rotation=90)
        # plt.show()
        plt.subplot(313)
        plt.plot(tlist, zlist, label="traj")
        plt.plot(tlistsim, zlistsim, label="sim")
        plt.plot(tlistsimc, zlistsimc, label="simc")
        plt.plot(tlisthw, zlisthw, label="hw")
        plt.plot(tlisthwc, zlisthwc, label="hwc")
        plt.ylim([-1.5,1.5])
        plt.legend(loc="upper left", prop=legend_properties)
        plt.xlabel("seconds", fontsize=16)#, fontweight="bold")
        plt.ylabel("z [m]", fontsize=16)#, fontweight="bold")
        plt.xticks(fontsize=14)#, rotation=90)
        plt.yticks(fontsize=14)#, rotation=90)
        plt.show()

        # 3D Plot
        fig = plt.figure()
        # plt.suptitle("Desired vs Simulated vs Experiment Trajectories 3D", fontsize=22)#, fontweight="bold")
        ax = plt.axes(projection='3d')
        ax.plot3D(xlistsim, ylistsim, zlistsim, 'orange', label="simulation")
        ax.plot3D(xlistsimc, ylistsimc, zlistsimc, 'blue', label="simulation c")
        ax.plot3D(xlisthw, ylisthw, zlisthw, 'orange', label="experiment")
        ax.plot3D(xlisthwc, ylisthwc, zlisthwc, 'blue', label="commanded")
        ax.plot3D(xlist, ylist, zlist, 'red', label="desired traj")

        plt.legend(loc="upper right", prop=legend_properties)
        ax.set_xlabel('x [m]', fontsize=20, labelpad=18)#, fontweight="bold")
        ax.set_ylabel('y [m]', fontsize=20, labelpad=18)#, fontweight="bold")
        ax.set_zlabel('z [m]', fontsize=20, labelpad=18)#, fontweight="bold")
        ax.xaxis.set_tick_params(labelsize=18, pad=5)
        ax.yaxis.set_tick_params(labelsize=18, pad=5)
        ax.zaxis.set_tick_params(labelsize=18, pad=5)
        plt.locator_params(nbins=3)
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.xaxis._axinfo["grid"]['linewidth'] = 0.3
        ax.yaxis._axinfo["grid"]['linewidth'] = 0.3
        ax.zaxis._axinfo["grid"]['linewidth'] = 0.3
        # ax.zaxis._axinfo["grid"]['color'] = "#ee0009"
        plt.grid(which = 'minor')
        plt.show()

    def plotJoint(self):
        # ===================
        # Plot Joints
        # ===================
        plt.figure()
        plt.subplots_adjust(left=None, bottom=None, right=None, top=None, wspace=None, hspace=0.4)
        plt.subplot(421)
        # plt.plot(self.tarray, self.jdict[0], label="traj0")
        plt.plot(self.tarrayhw, self.jdicthw[0], label="rd=1 v=100 pos_f=5000 vel 0")
        plt.plot(self.tarrayhwc, self.jdicthwc[0], label="rd=1 v=100 pos_f=5000 pos 0")
        plt.plot(self.tarraysim, self.jdictsim[0], label="rd= v=10 pos_f=1000 vel 0")
        plt.plot(self.tarraysimc, self.jdictsimc[0], label="rd= v=10 pos_f=1000 pos 0")
        # plt.ylim([-1.5,1.5])
        # plt.ylim([-3.2,3.2])
        plt.legend(loc="upper right")
        plt.grid(True)
        plt.xlabel("s")
        plt.ylabel("rad/s, rad")
        # plt.show()

        plt.subplot(422)
        # plt.plot(self.tarray, self.jdict[1], label="traj1")
        plt.plot(self.tarrayhw, self.jdicthw[1], label="rd=1 v=100 pos_f=5000 vel 1")
        plt.plot(self.tarrayhwc, self.jdicthwc[1], label="rd=1 v=100 pos_f=5000 pos 1")
        plt.plot(self.tarraysim, self.jdictsim[1], label="rd= v=10 pos_f=1000 vel 1")
        plt.plot(self.tarraysimc, self.jdictsimc[1], label="rd= v=10 pos_f=1000 pos 1")
        # plt.ylim([-1.5,1.5])
        # plt.ylim([-3.2,3.2])
        plt.legend(loc="upper right")
        plt.grid(True)
        plt.xlabel("s")
        plt.ylabel("rad/s, rad")
        # plt.show()

        plt.subplot(423)
        # plt.plot(self.tarray, self.jdict[2], label="traj2")
        plt.plot(self.tarrayhw, self.jdicthw[2], label="rd=1 v=100 pos_f=5000 vel 2")
        plt.plot(self.tarrayhwc, self.jdicthwc[2], label="rd=1 v=100 pos_f=5000 pos 2")
        plt.plot(self.tarraysim, self.jdictsim[2], label="rd= v=10 pos_f=1000 vel 2")
        plt.plot(self.tarraysimc, self.jdictsimc[2], label="rd= v=10 pos_f=1000 pos 2")
        # plt.ylim([-1.5,1.5])
        # plt.ylim([-3.2,3.2])
        plt.legend(loc="upper right")
        plt.grid(True)
        plt.xlabel("s")
        plt.ylabel("rad/s, rad")
        # plt.show()

        plt.subplot(424)
        # plt.plot(self.tarray, self.jdict[3], label="traj3")
        plt.plot(self.tarrayhw, self.jdicthw[3], label="rd=1 v=100 pos_f=5000 vel 3")
        plt.plot(self.tarrayhwc, self.jdicthwc[3], label="rd=1 v=100 pos_f=5000 pos 3")
        plt.plot(self.tarraysim, self.jdictsim[3], label="rd= v=10 pos_f=1000 vel 3")
        plt.plot(self.tarraysimc, self.jdictsimc[3], label="rd= v=10 pos_f=1000 pos 3")
        # plt.ylim([-1.5,1.5])
        # plt.ylim([-3.2,3.2])
        plt.legend(loc="upper right")
        plt.grid(True)
        plt.xlabel("s")
        plt.ylabel("rad/s, rad")
        # plt.show()

        plt.subplot(425)
        # plt.plot(self.tarray, self.jdict[4], label="traj4")
        plt.plot(self.tarrayhw, self.jdicthw[4], label="rd=1 v=100 pos_f=5000 vel 4")
        plt.plot(self.tarrayhwc, self.jdicthwc[4], label="rd=1 v=100 pos_f=5000 pos 4")
        plt.plot(self.tarraysim, self.jdictsim[4], label="rd= v=10 pos_f=1000 vel 4")
        plt.plot(self.tarraysimc, self.jdictsimc[4], label="rd= v=10 pos_f=1000 pos 4")
        # plt.ylim([-1.5,1.5])
        # plt.ylim([-3.2,3.2])
        plt.legend(loc="upper right")
        plt.grid(True)
        plt.xlabel("s")
        plt.ylabel("rad/s, rad")
        # plt.show()

        plt.subplot(426)
        # plt.plot(self.tarray, self.jdict[5], label="traj5")
        plt.plot(self.tarrayhw, self.jdicthw[5], label="rd=1 v=100 pos_f=5000 vel 5")
        plt.plot(self.tarrayhwc, self.jdicthwc[5], label="rd=1 v=100 pos_f=5000 pos 5")
        plt.plot(self.tarraysim, self.jdictsim[5], label="rd= v=10 pos_f=1000 vel 5")
        plt.plot(self.tarraysimc, self.jdictsimc[5], label="rd= v=10 pos_f=1000 pos 5")
        # plt.ylim([-1.5,1.5])
        # plt.ylim([-3.2,3.2])
        plt.legend(loc="upper right")
        plt.xlabel("s")
        plt.ylabel("rad/s, rad")
        plt.grid(True)
        # plt.show()

        plt.subplot(427)
        # plt.plot(self.tarray, self.jdict[6], label="traj6")
        plt.plot(self.tarrayhw, self.jdicthw[6], label="rd=1 v=100 pos_f=5000 vel 6")
        plt.plot(self.tarrayhwc, self.jdicthwc[6], label="rd=1 v=100 pos_f=5000 pos 6")
        plt.plot(self.tarraysim, self.jdictsim[6], label="rd= v=10 pos_f=1000 vel 6")
        plt.plot(self.tarraysimc, self.jdictsimc[6], label="rd= v=10 pos_f=1000 pos 6")
        # plt.ylim([-1.5,1.5])
        # plt.ylim([-3.2,3.2])
        plt.legend(loc="upper right")
        plt.grid(True)
        plt.xlabel("s")
        plt.ylabel("rad/s, rad")
        plt.show()

def main():
    plotter = TrajPlotter()
    # plotter.plotFK()
    plotter.plotJoint()

if __name__ == "__main__":
    main()

