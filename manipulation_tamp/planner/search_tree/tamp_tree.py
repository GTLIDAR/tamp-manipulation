import time
from datetime import datetime
import os
import json
import networkx as nx
from matplotlib import pyplot as plt

from utils.priority_queue import PriorityQueue

class PddlTree(object):
    """ state space search tree for PDDL

    This tree contains the state space search nodes for PDDL and provides various
    search algorithms. The nodes within this tree should be child class of PddlNode.
    Query object is not required if PddlNode.get_cost() does not require query.

    Attributes:
        root: (PddlNode) root node of search tree
        task: (pyperplan.task) grounded task translated from PDDL
        goals: (list: PddlNode) list of nodes that reached goal condition
        motion_plan_runner: (BasicMotionPlanRunner) Query object for geometry/motion planning interfacing

    """
    def __init__(self, root, task, motion_plan_runner=None):
        self.root = root
        self.root.tree = self
        self.task = task
        self.motion_plan_runner = motion_plan_runner

        self.goals = []

        self.placed_object = {}
        self.refinement_time = 0
        self.node_list = [root]

    def dls(self, start, depth_limit=-1, n_sols=1):
        """ Depth limited search

        Performs discrete depth limited search, the result is saved in self.goals

        Args:
            start: (PddlNode) root node for the search
            depth_limit : int depth limit, if negative, no limit
            n_sols: int desired number of solution, if negative, find all
        
        Returns:
            nodes_visited: (integer) number of nodes visited
        """

        (goals, nodes_visited) = self._dls_internal(
            start, [], 0, depth_limit, n_sols)
        print("DLS visited:", nodes_visited)
        return (goals, nodes_visited)
    
    def _dls_internal(self, cur, goal_list, nodes_visited, depth_limit, n_sols):
        """ recursive helper method for dls
        """
        if (cur.depth > depth_limit >= 0) or (len(goal_list) >= n_sols >= 0):
            return (goal_list, nodes_visited)

        nodes_visited += 1

        if cur.goal_reached():
            goal_list.append(cur)
            return (goal_list, nodes_visited)
        
        cur.expand()

        if nodes_visited%200==0:
            print("DLS visited:", nodes_visited)
            print("n_child:", len(cur.children))

        for ch in cur.children:
            (goal_list, nodes_visited) = self._dls_internal(
                ch, goal_list, nodes_visited, depth_limit, n_sols
            )
        
        return (goal_list, nodes_visited)
    
    def bfs(self, depth_limit=-1, n_sols=1):
        """ Best first search with depth limit

        Performs discrete best first search, the result is saved in self.goals

        Args:
            depth_limit : int depth limit, if negative, no limit
            n_sols: int desired number of solution, if negative, find all
        
        Returns:
            nodes_visited: (integer) number of nodes visited
        """
        frontier = PriorityQueue()
        frontier.push(self.root)
        nodes_visited = 0
        goals = []

        while len(frontier):
            cur = frontier.pop()
            nodes_visited += 1
            print("Visiting Node", nodes_visited)
            print("Depth", cur.depth)

            if cur.parent is not None:
                print(cur.action)
                cur.calc_traj()
                print(cur.get_cost())

            if cur.g < 1e16:
                if cur.goal_reached():
                    goals.append(cur)
                    print("Found Goal!", len(goals))
                    # if desired number of goals reached, end loop
                    if len(goals) >= n_sols >= 0:
                        break
                
                if not (cur.depth >= depth_limit >= 0):
                    cur.expand()
                    for ch in cur.children:
                        frontier.push(ch)

        return (goals, nodes_visited)

    def hybrid_search(self, total_depth_limit=-1, explore_depth_limit=15, 
        n_sols=1, explore_n_sols=-1, option="ddp"):
        """ performs hybrid search

        Depth limited discrete search will be applied to populate the tree and 
        find discrete solutions. Best first search will then run DDP using goal_count
        as heuristics

        Args:
            total_depth_limit : int depth limit, if negative, no limit
            explore_depth_limit: int depth limit for dls
            n_sols: int desired number of solution, if negative, find all
            explore_n_sols: desired number of sols for dls explore
        
        Returns:
            nodes_visited: (integer) number of nodes visited
        """
        frontier = PriorityQueue()
        frontier.push(self.root)
        nodes_visited = 0
        goals = []
        finished = False

        while (len(frontier)):
            cur = frontier.pop()
            if option == "refine":
                cur.option = "ddp"
            else:
                cur.option = option
            nodes_visited += 1
            print("Visiting Node", nodes_visited)
            print("Frontier size:", len(frontier))
            print("Depth", cur.depth)
            print("SubTree Goal Count:", cur.goal_count)

            if cur.parent is not None:
                print(cur.action.name)
                cur.calc_traj()
                print("Action cost:", cur.action_cost)

            if cur.action_cost < 1e16:
                if cur.goal_reached():
                    goals.append(cur)
                    print("Found Goal!", len(goals))

                    if option == "refine":
                        refine_start = time.time()
                        finished = self.refine_goal_traj(cur)
                        
                        self.refinement_time += time.time() - refine_start

                        if not finished:
                            goals.pop()
                    else:
                        finished = (len(goals) >= n_sols >= 0)
                        
                    # if desired number of goals reached, end loop
                    if finished:
                        break
                else:
                    if not (cur.depth >= total_depth_limit >= 0):
                        if total_depth_limit < 0 or total_depth_limit > explore_depth_limit+cur.depth:
                            dls_depth_limit = explore_depth_limit+cur.depth
                        else:
                            dls_depth_limit = total_depth_limit

                        self.dls(cur, dls_depth_limit, explore_n_sols)

                        if dls_depth_limit==total_depth_limit and cur.goal_count==0:
                            continue

                        frontier.heapify()
                        print("New Nodes Added:", len(cur.children))
                        for ch in cur.children:
                            frontier.push(ch)
                            print(ch.action.name)
                        
            print("\n\n\n")
            
        return (goals, nodes_visited)

    def save_traj(self, n_trajs=1, foldername=None, filename=None):
        """ save trajectory
        """
        if not self.goals:
            print("No available trajectory.")
            return
        if foldername is None:
            file_dir = os.getcwd()
            foldername = file_dir+"/results/"+datetime.now().strftime("%Y%m%dT%H%M%S")+"/"
            os.makedirs(foldername)

        print("Goal Num:", len(self.goals))
        print("Saving:", n_trajs)

        for i in range(n_trajs):
            if i >= len(self.goals):
                print("No available trajectory")
                return
            if filename is None:
                filename = "traj"+str(i)+".json"
            goal_node = self.goals[i]

            data = self.get_traj(goal_node)
        
        with open(foldername+filename, 'w') as out:
            json.dump(data, out)
    
    def get_move_query_sequence(self, goal, save_results=True):
        """ save discrete solution with motion plan query
        """
        path = goal.extract_path()

        data = []

        for node in path:
            if node == self.root:
                continue
            
            data.append(node.get_move_query_for_admm())
        
        if save_results:
            filename = "~/code/drake/conveyor_belt_tamp/results/move_query"+datetime.now().strftime("%Y%m%dT%H%M%S")+".json"
            with open(filename, 'w') as out:
                json.dump(data, out)

        return data

    def get_traj(self, goal):
        """ extract trajectory from tree

        Args:
            goal: PddlTampNode
        Returns:
            data: list of dict, containing traj of each node
        """
        path = goal.extract_path()

        data = []

        for node in path:
            if node == self.root:
                continue
            data.append(node.traj)
        
        return data

    def get_sol(self, goal):
        """ extract action sequence
        Args:
            goal: PddlTampNode
        Returns:
            sol: list of pyperplan.Operation
        """
        path = goal.extract_path()

        sol = []

        for node in path:
            if node == self.root:
                continue
            sol.append(node.action.name)
        
        return sol
    
    def get_cost_seq(self, goal):
        path = goal.extract_path()

        sol = []

        for node in path:
            if node == self.root:
                continue
            sol.append(node.g)
        
        return sol

    def refine_goal_traj(self, goal):
        """ Refine trajectory to goal node using ADMM
        """
        path = goal.extract_path()
        
        for i in range(len(path)):
            node = path[i]
            if node is not self.root:
                print("Node "+str(i)+"/"+str(len(path))+": "+node.action.name)
                start = time.time()
                if not node.refine_traj(): 
                    print("Trajectory is not feasible for refinement, Keeping DDP Trajectory")
                    try:
                        print("q_init:", node.move_query.prev_q)
                        print("ee_goal:", node.move_query.desired_ee)
                    except:
                        pass
                print("Trajectory computation time:", time.time()-start, "sec\n")
            
        return True
    
    def get_branching_factor(self):
        """ Returns branching factor
        """
        n = 0
        ch = 0

        for node in self.node_list:
            if len(node.children):
                n += 1
                ch += len(node.children)
        
        return (len(self.node_list), ch/n)