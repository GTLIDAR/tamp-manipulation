import sys
import os
import logging
import copy
import networkx as nx
from matplotlib import pyplot as plt

def build_causal_graph(task, show=True, add_cooccuring_edges=True):
    """ Build a causal graph from a task object, modified from original ptg.py
    to use networkx structure

    Args:
        task: Task object
        show_graph: Boolean, whether to show graph
        add_cooccuring_edges: Boolean, whether to add causal edges due to 
            co-occuring effects
    Returns:
        graph: 
            node attributes:
                stuffs for plotting
            edge attributes:
                ops

    """
    graph = nx.DiGraph()

    for u in task.facts:
        for v in task.facts:
            for op in task.operators:
                if u==v:
                    continue
                elif u in op.preconditions:
                    if v in op.add_effects or v in op.del_effects:
                        if (u, v) not in graph.edges:
                            graph.add_edge(u, v, ops=set([op]))
                        else:
                            graph.edges[u, v]["ops"].add(op)

                elif add_cooccuring_edges and (u in op.add_effects or u in op.del_effects):
                    if v in op.add_effects or v in op.del_effects:
                        if (u, v) not in graph.edges:
                            graph.add_edge(u, v, ops=set([op]))
                        else:
                            graph.edges[u, v]["ops"].add(op)
                        
    if show:
        show_graph(graph)

    return graph

def get_subproblems(graph, task, show=False):
    """ get list of subgoals from disconnected components of causal graph

    Args:
        graph: (networkx DiGraph) causal graph
        task: (pyperplan.Task) task object translated from PDDL

    Returns:
        a list with (subgoals, ops) pair where "subgoals" is a set of subgoals 
        in a disconnected component, and "ops" is a set of operators related to
        the component
    """
    pruned_graph, _ = _prune_manip_causal_graph(graph, task.initial_state)
    if show:
        show_graph(pruned_graph)
    components = list(nx.weakly_connected_components(pruned_graph))

    subproblems = []

    for cc in components:
        s = graph.subgraph(cc)
        
        subgoals = set()
        ops = set()
        for node in s.nodes:
            if node in task.goals and node not in task.initial_state:
                subgoals.add(node)
        
        # related operations are listed in the in_edges of each node
        for edge in graph.in_edges(cc):
            ops |= graph.get_edge_data(*edge)["ops"]
        
        if len(subgoals):
            subproblems.append((subgoals, ops))
    
    return subproblems

def generate_subtask(task, subproblem, state):
    """ Get subtask from full task

    Args:
        task: (pyperplan.Task) Original task object
        subproblem: (subgoals, ops) pair where "subgoals" is a set of subgoals 
            in a disconnected component, and "ops" is a set of operators related 
            to the component
        state: (list of string) state at the beginning of the subtask
    """
    subtask = copy.deepcopy(task)
    (subtask.goals, subtask.operators) = subproblem
    subtask.initial_state = state

    return subtask
            
def _prune_manip_causal_graph(graph, state):
    prune_set = set(["(free iiwa)", "(ready-to-move iiwa)"])

    for node in graph.nodes:
        if node[:10]=="(unblocked" or node.startswith("(unobstructed"):
            if node in state:
                prune_set.add(node)
    
    pruned_graph = _prune_graph(graph, prune_set)

    return pruned_graph, prune_set

def _prune_graph(graph, del_set):
    """ created a pruned graph with specified nodes deleted
    
    Args: 
        graph: (networkx DiGraph) original graph
        dek_set: (set of nodes) nodes to be deleted
    Returns:
        pruned_graph: (networkx DiGraph) pruned graph
    """
    pruned_graph = copy.deepcopy(graph)

    for node in del_set:
        try:
            pruned_graph.remove_node(node)
        except:
            print("Warning: node not found", node)

    return pruned_graph

def show_graph(graph):
    plt.subplot(111)
    nx.draw(graph, with_labels=True)
    plt.show()

