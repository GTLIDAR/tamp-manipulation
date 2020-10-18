from functools import total_ordering
import math
from utils.traj_utils import lcmt_multi_wp_manip_query_to_dict

@total_ordering
class PddlNode(object):
    """ Node for PDDL state space search tree

    This node provides discrete state space search tools in PDDL.

    Attributes:
        parent: (PDDLNode) parent node
        state: (list: string) list of predicates that are true
        tree: (PDDLTree) the tree this node belongs to
        children: (list: PDDLNode) list of child nodes
        action: (task.Operator) action which produced the state
        depth: (integer) The depth of the node on the tree
        goal_count: logs how many goal nodes are in the expanded subtree under
            this node
        is_goal: whether this node is a goal node, initialized to be None
    """
    def __init__(self, parent, state, action, depth, tree):
        # data struct
        self.parent = parent
        self.state = state
        self.action = action
        self.tree = tree
        self.children = []
        self.goal_count = 0
        self.is_goal = None

        # costs
        self.depth = depth

        self._expanded = False

    def __eq__(self, other):
        self_path = self.extract_path()
        other_path = other.extract_path()

        if len(self_path) != len(other_path):
            return False

        for i in range(len(self_path)):
            if self_path[i].state != other_path[i].state:
                return False

        return True

    def __lt__(self, other):
        return self.get_cost() < other.get_cost()

    def expand(self):
        """ explore allowed actions to populate self.children
        """
        if not self._expanded:
            for (op, state) in self.tree.task.get_successor_states(self.state):
                self.children.append(self.make_child_node(op, state))

            self._expanded = True

    def goal_reached(self):
        """ Check if goal is reached

        Returns:
            a boolean of whether goal is reached
        """
        if self.is_goal is None:
            self.is_goal = self.tree.task.goal_reached(self.state)

            if (self.is_goal):
                self._increment_goal_count()
        return self.is_goal

    def extract_path(self):
        """ returns list of nodes

        Returns:
            path: (list: PddlNode) a list of nodes from root to current
        """
        path = []
        cur = self
        while cur.parent is not None:
            path.append(cur)
            cur = cur.parent
        path.reverse()
        return path

    def get_cost(self):
        """ gets cost to node, which equals to the length of path to this node
        """
        return self.depth

    def make_child_node(self, action, state):
        """ construct a new child node containing the state and applied action

        Args:
            action: (task.Operation)
            state: (list: string) state of child node
        """
        return PddlNode(self, state, action, self.depth+1, self.tree)

    def _increment_goal_count(self):
        cur = self
        while cur.parent is not None:
            cur.goal_count += 1
            cur = cur.parent

    @staticmethod
    def make_root_node(init_state):
        """ creates a root node with initial state with no parents and no tree

        Args:
            init_state: (list: string) initial state
        Returns:
            root: (PDDLNode)
        """
        return PddlNode(None, init_state, None, 0, None)

@total_ordering
class PddlTampNode(PddlNode):
    """
    This Node provides tools for PDDL forward state space search with stored
    trajectory.

    Attributes
        g: (double) trajectory cost from root to current node
        action_cost: (double) trajectory cost of the action that produced current state
        time: (double) time in second when the state is arrived
        traj: (dict) trajectory information
        option: (string) motion plan options. currently supports "ddp" and "admm"
    """
    def __init__(self, parent, state, action, depth, tree, option="ddp"):
        super().__init__(parent, state, action, depth, tree)

        if self.parent is not None:
            # these attributes are not computed if traj is not calculated
            self.g = None
            self.action_cost = None
            self.time = None
            self.parent_traj = self.parent.traj
            self.discrete_cost = None
        else:
            self.g = 0
            self.action_cost = 0
            self.time = 0
            self.parent_traj = None
            self.discrete_cost = 0

        self.placed_object = {}
        self.traj = None
        self.option = option
        self.final_ee = None
        self.move_query = None
        self.processed_object = []

    def __lt__(self, other):
        # if self.get_cost() == None or other.get_cost()==None:
        #     if self.goal_count==other.goal_count==0:
        #         return self.depth < other.depth
        #     return self.goal_count >= other.goal_count
        # else:
        #     return super().__lt__(self, other)
        if self.goal_count > 0: # if there is goal under current node
            if other.goal_count == 0:
                return True
        else:
            if other.goal_count > 0:
                return False

        return self.get_cost() >= other.get_cost()

    def get_cost(self):
        """ Returns path cost to current node. if not available, calculate it

        overrides the parent class implementation.
        """
        if self.g is None:
            return self.parent.get_cost()
        return self.g

    def expand(self):
        """ explore allowed actions to populate self.children
        """
        if not self._expanded:
            for (op, state) in self.tree.task.get_successor_states(self.state):
                self.children.append(self.make_child_node(op, state, self.option))
                self.tree.node_list.extend(self.children)

            self._expanded = True

    def make_child_node(self, action, state, option="ddp"):
        """ construct a new child node containing the state and applied action

        Overrides the parent class implementation.

        Args:
            action: (task.Operation)
            state: (list: string) state of child node
        """
        return PddlTampNode(self, state, action, self.depth+1, self.tree, option)

    @staticmethod
    def make_root_node(init_state, option="ddp"):
        """ creates a root node with initial state with no parents and no tree

        Overrides the parent class implementation.

        Args:
            init_state: (list: string) initial state
        Returns:
            root: (PDDLNode)
        """
        return PddlTampNode(None, init_state, None, 0, None, option)

    def calc_traj(self):
        """ Calculates trajectory using a PddlQuery object

        Returns:
            True if trajectory feasible, False if not
        """
        self.placed_object = self.parent.placed_object
        self.tree.motion_plan_runner.run(self)
        self.traj = self.tree.motion_plan_runner.results
        self.action_cost = self.traj["cost"]

        self.g = self.parent.g + self.action_cost + self.discrete_cost

        if self._expanded:
            for ch in self.children:
                ch.parent_traj = self.traj
        
        if self.traj["cost"]==float('inf') or math.isnan(self.traj["cost"]):
            return False
        return True
    
    def refine_traj(self):
        """ Refine ddp trajectory into admm
        """
        if self.option == "admm":
            print("ADMM trajectory already generated")
            return
        
        self._set_traj_option("admm")
        self.tree.motion_plan_runner.run(self)
        traj = self.tree.motion_plan_runner.results
        
        # if ADMM fails return false
        if traj["cost"]==float('inf') or math.isnan(traj["cost"]) or traj["cost"]==0:
            return False

        else:
            self.traj = traj
            self.action_cost = self.traj["cost"]
            self.g = self.parent.g + self.action_cost

            if self._expanded:
                for ch in self.children:
                    ch.parent_traj = self.traj

            return True

    def _set_traj_option(self, option="admm"):
        self.option = option
        if self.move_query is not None:
            try:
                self.move_query.option=option
            except:
                if option=="admm":
                    self.move_query.level=2
                elif option=="ddp":
                    self.move_query.level=1
                else:
                    print("Invalid Traj Option")
            
            if option == "admm":
                self.move_query.time_step = 0.005

    def get_move_query_for_admm(self):
        if self.move_query is not None:
            query_dict = lcmt_multi_wp_manip_query_to_dict(self.move_query)
            query_dict["option"] = "admm"
        else:
            query_dict = None
            
        return query_dict
        
    