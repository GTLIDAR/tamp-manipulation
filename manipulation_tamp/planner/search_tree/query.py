import json
import numpy as np

from pydrake.math import RollPitchYaw, RigidTransform, RotationMatrix
from pydrake.common.eigen_geometry import Quaternion

class GeometryQueryInterface(object):
    """ informal interface for geometry query object
    """

    def get_object_state(self, object_name, t):
        """ returns state of object

        Args:
            object_name: [string]
            t: [double] time
        Returns:
            q: position vector [x y z r p y]
        """

        return [0, 0, 0, 0, 0, 0]

    def get_q_object_init(self, object_name, t, xyz_iiwa):
        """ Returns vector of length 7 [quaternion, xyz]
        """
        q = self.get_object_state(object_name, t)
        quaternion = RollPitchYaw(q[3], q[4], q[5]).ToQuaternion()

        wxyz = quaternion.wxyz()

        xyz_obj = np.array(q[:3]) - np.array(xyz_iiwa)

        q_object_init = []
        for i in range(4):
            q_object_init.append(wxyz[i])
        
        for i in range(3):
            q_object_init.append(xyz_obj[i])

        return q_object_init
    
    def get_q_object_final(self, q_object_init, ee_init, ee_final):
        """
        Input:
            q_object_init: [quaternion, xyz]
            ee_init: [x y z r p y]
            ee_final: [x y z r p y]
            xyz_iiwa: [x, y, z]
        Return:
            q_object_final: [quaternion, xyz]
        """
        xyz_obj = np.array(q_object_init[4:]).transpose()
        # xyz_obj[0] -= np.array(xyz_iiwa) # convert translation from world frame to robot base frame
        quaternion_obj = Quaternion(np.array(q_object_init[:4]))
        rot_obj = RotationMatrix(quaternion_obj)

        X_WO = RigidTransform(rot_obj, xyz_obj)

        xyz_ee_init = np.array(ee_init[:3]).transpose()
        rot_ee_init = RollPitchYaw(ee_init[3], ee_init[4], ee_init[5]).ToRotationMatrix()

        X_WE1 = RigidTransform(rot_ee_init, xyz_ee_init)
        
        xyz_ee_final = np.array(ee_final[:3]).transpose()
        rot_ee_final = RollPitchYaw(ee_final[3], ee_final[4], ee_final[5]).ToRotationMatrix()

        X_WE2 = RigidTransform(rot_ee_final, xyz_ee_final)

        X_EO = X_WE1.inverse().multiply(X_WO)
        X_WO2 = X_WE2.multiply(X_EO)

        wxyz_obj_final = X_WO2.rotation().ToQuaternion().wxyz()
        xyz_obj_final = X_WO2.translation()
    
        # xyz_ee_obj_init = xyz_obj - xyz_ee_init
        # rot_ee_init_final = rot_ee_init.multiply(rot_ee_final.transpose())
        # xyz_ee_obj_final = np.matmul(rot_ee_init_final.matrix(), xyz_ee_obj_init)
        # # xyz_ee_obj_final += np.array(xyz_iiwa) # convert translation from robot base frame to world frame
        # xyz_obj_final = xyz_ee_final + xyz_ee_obj_final

        # rot_obj_final = rot_ee_init_final.multiply(rot_obj)
        # quaternion_obj_final = rot_obj_final.ToQuaternion()
        # wxyz_obj_final = quaternion_obj_final.wxyz()
        
        q_object_final = []
        for i in range(4):
            q_object_final.append(wxyz_obj_final[i])
        
        for i in range(3):
            q_object_final.append(xyz_obj_final[i])

        return q_object_final

class ConveyorBeltManipQuery(GeometryQueryInterface):
    """ Computes the geometry info for conveyor belt grasping domain

        Attributes:
        setup: geometry setup information read from a setup json file
        op_name: name of op to determine ee pose
    """
    def __init__(self, setup_file): 
        with open(setup_file) as fp:
            self._setup = json.load(fp)

    def get_object_state(self, object_name, t):
        """ computes state of object on conveyor belt

        Args:
            object_name: [string]
            t: [double] time
        Returns:
            q: position vector [x y z r p y]
            v: velocity vector [vx vy vz wr wp wy]
        """

        q = (np.array(self._setup["object_init_pos"][object_name]) 
            + np.array(self._setup["belt_vel"])*t).tolist()

        q[1] += self._setup["yInitOffset"]

        return q
    
    def get_bin_pos(self, bin_name):
        """ get the location of bin
        """

        return self._setup["bin_pos"][bin_name]
    
    def get_desired_ee_pos(self, node):
        op = node.action.name[1:-1]
        op_name = op.split(" ")[0]
        
        try:
            ee_offset = self._setup["ee_offset"][op_name]
        except:
            ee_offset = [0, 0, 0, 0, 0, 0]
        
        if op_name == "move-to-object-front" or op_name == "move-to-object-top":
            object_name = op.split(" ")[2]
            target_pos = self.get_object_state(object_name, node.time)
        elif op_name == "move-to-bin":
            bin_name = op.split(" ")[3]
            target_pos = self.get_bin_pos(bin_name)
        
        return (np.array(ee_offset) + np.array(target_pos)).tolist()
    
    def get_push_wait_time(self, node):
        op = node.action.name[1:-1]
        object_name = op.split(" ")[2]
        bin_name = op.split(" ")[3]

        object_pos = self.get_object_state(object_name, node.time)
        bin_pos = self.get_bin_pos(bin_name)

        dy = bin_pos[1] - object_pos[1]

        vel = self._setup["belt_vel"][1]
        
        return dy/vel


class ConveyorBeltManipReactiveQuery(ConveyorBeltManipQuery):
    """ Computes the geometry info for conveyor belt grasping domain

        Attributes:
        setup: geometry setup information read from a setup json file
        op_name: name of op to determine ee pose
    """
    def __init__(self, setup_file): 
        super().__init__(setup_file)
        
        self._object_state = None

    def get_object_state(self, object_name, t):
        """ computes state of object on conveyor belt

        Args:
            object_name: [string]
            t: [double] time
        Returns:
            q: position vector [x y z r p y]
            v: velocity vector [vx vy vz wr wp wy]
        """
        if self._object_state is None:
            return super().get_object_state(object_name, t)
        else:
            object_id = int(object_name.split("_")[-1])
            t_cur = float(self._object_state.utime) / 1e6

            v = self._object_state.v[object_id]
            q = (np.array(self._object_state.q[object_id]) 
                 + np.array(v)*(t-t_cur)).tolist()

        return (q, v)

    def update_object_state(self, object_state):
        """ update object state from simulation
        """
        self._object_state = object_state

class MultiWPConveyorBeltManipQuery(ConveyorBeltManipQuery):
    def get_desired_ee_pos(self, node):
        """ Returns a list of waypoints

        Overrides the method in parent class
        """
        op = node.action.name[1:-1]
        op_name = op.split(" ")[0]
        
        try:
            ee_offsets = self._setup["ee_offset"][op_name]
        except:
            ee_offsets = [[0, 0, 0, 0, 0, 0]]
        
        if op_name.startswith("move-to-object"):
            object_name = op.split(" ")[2]
            target_pos = self.get_object_state(object_name, node.time)
        elif op_name == "move-to-bin" or op_name == "move-to-push" or op_name == "push":
            bin_name = op.split(" ")[3]
            target_pos = self.get_bin_pos(bin_name)
            if op_name == "move-to-push" or op_name == "push": # set push "yaw angle to 0"
                target_pos[-1] = 0
        
        wps = []
        
        for ee_offset in ee_offsets:
            wps.append((np.array(ee_offset) + np.array(target_pos)).tolist())
        
        return wps


class StationaryMultiWPManipQuery(GeometryQueryInterface):
    """ Computes the geometry info for conveyor belt grasping domain

        Attributes:
        setup: geometry setup information read from a setup json file
        op_name: name of op to determine ee pose
    """
    def __init__(self, setup_file): 

        with open(setup_file) as fp:
            self._setup = json.load(fp)

    def get_object_state(self, object_name, t):
        """ computes state of object on conveyor belt

        Args:
            object_name: [string]
            t: [double] time
        Returns:
            q: position vector [x y z r p y]
            v: velocity vector [vx vy vz wr wp wy]
        """

        q = self._setup["object_init_pos"][object_name]

        return q
    
    def get_table_pos(self, table_name):
        """ get the location of table
        """

        # x_offset = np.random.uniform(low=-self._setup["random_offset"], 
        #                              high=self._setup["random_offset"])
        # y_offset = np.random.uniform(low=-self._setup["random_offset"], 
        #                              high=self._setup["random_offset"])

        table_pos = self._setup["table_pos"][table_name]
        # table_pos[0] += x_offset
        # table_pos[1] += y_offset

        return table_pos

    def get_desired_ee_pos(self, node):
        """ Returns a list of waypoints

        Overrides the method in parent class
        """
        op = node.action.name[1:-1]
        op_name = op.split(" ")[0]

        try:
            ee_offsets = self._setup["ee_offset"][op_name]
        except:
            ee_offsets = [[0, 0, 0, 0, 0, 0]]
        
        if op_name == "move-to-object-side" or op_name == "move-to-object-top":
            object_name = op.split(" ")[2]
            target_pos = self.get_object_state(object_name, node.time)
        elif op_name == "move-to-goal-table" or op_name == "move-to-free-table":
            table_name = op.split(" ")[3]
            if table_name not in node.placed_object:
                node.placed_object[table_name] = 0

            print("Placing object on "+table_name)
            print("location index:", node.placed_object[table_name])

            if -0.1<node.parent.final_ee[-2]<0.1:
                node_offset = [0, 0, -0.1, 0, -1.5, 0]
            else:
                node_offset = [0, 0, 0, 0, 0, 0]
            
            if op_name == "move-to-goal-table":
                offset_pos = self._setup["goal_table_offset_list"][node.placed_object[table_name]]
                # offset_pos = self._setup["goal_table_offset_list"][0]
            elif op_name == "move-to-free-table":
                offset_pos = self._setup["free_table_offset_list"][node.placed_object[table_name]]
                # offset_pos = self._setup["free_table_offset_list"][0]
            node.placed_object[table_name] += 1
            if node.placed_object[table_name] >= len(self._setup["free_table_offset_list"]):
                node.placed_object = 0

            table_pos = self.get_table_pos(table_name)
            target_pos = (np.array(node_offset) + np.array(offset_pos) + np.array(table_pos)).tolist()

        
        wps = []
        
        for ee_offset in ee_offsets:
            wps.append((np.array(ee_offset) + np.array(target_pos)).tolist())
        
        return wps

class StationaryMultiWPManipReactiveQuery(StationaryMultiWPManipQuery):
    def __init__(self, setup_file): 
        super().__init__(setup_file)
        
        self._object_state = None
    
    def get_object_state(self, object_name, t):
        """ computes state of object on conveyor belt

        Args:
            object_name: [string]
            t: [double] time
        Returns:
            q: position vector [x y z r p y]
            v: velocity vector [vx vy vz wr wp wy]
        """
        if self._object_state is None:
            return super().get_object_state(object_name, t)
        else:
            object_id = int(object_name.split("_")[-1])
            q = self._object_state.q[object_id]
        
        return q

    def update_object_state(self, object_state):
        """ update object state from simulation
        """
        self._object_state = object_state