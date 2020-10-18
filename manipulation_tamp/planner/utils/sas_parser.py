from __future__ import print_function
import os
import re
import sys
from collections import namedtuple

try:
    # python 3
    from builtins import open
except ImportError:
    # python 2
    from codecs import open

########################### classes ###########################################

Variable = namedtuple("Variable", ["name", "axiom_layer", "atom_type", "range", "values"])

Mutex = namedtuple("Mutex", ["n_facts", "values"])

State = namedtuple("State", ["values"])

Goal = namedtuple("Goal", ["n_goals", "values"])

Operator = namedtuple("Operator", ["name", "op_vars", "preconds", "effects", "cost"])

Axiom = namedtuple("Axiom", ["conditions", "effects"])

class HashableState(State):
    """ Implemented customized hash function for State
    """
    def __hash__(self):
        h = 0
        for var in self.values:
            h += hash(var)*hash(self.values[var])
        return h

############################## functions #######################################
def parse_sas(filename):
    """ Read and parse sas file

    Args: string filename

    Returns:
        trans_dict: dictionary containing the following fields
            version: int, version number, should be 3
            metric: int, 1 if metric is used, 0 if not
            variable: {str: Variable} dict mapping string variable name to Variable namedtuple
            mutex_group: list of Mutex namedtuple
            init_state: State namedtuple
            goal: Goal namedtuple
            operator: list of Operator namedtuple
            axiom: list of Axiom namedtuple
    """
    trans_dict = {}
    trans_dict["variable"] = {}
    trans_dict["mutex_group"] = []
    trans_dict["operator"] = []
    trans_dict["axiom"] = []

    with open(filename, "r") as fh:
        lines = fh.readlines()
        cur = 0
        begin = 0
        end = 0
        section = None

        while cur < len(lines):
            line = lines[cur]
            if line[:5]=="begin":
                begin = cur
                section = line[6:len(line)-1]
            elif line[:3]=="end":
                end = cur
                if section == "version":
                    trans_dict["version"] = int(lines[begin+1])

                elif section == "metric":
                    trans_dict["metric"] = int(lines[begin+1])

                elif section == "variable":
                    var_lines = lines[begin+1:end]
                    var = parse_variable(var_lines)
                    trans_dict["variable"][var.name] = var
                
                elif section == "mutex_group":
                    mutex_lines = lines[begin+1:end]
                    mutex = parse_mutex_group(mutex_lines)
                    trans_dict["mutex_group"].append(mutex)

                elif section == "state":
                    # init state
                    state_lines = lines[begin+1:end]
                    trans_dict["init_state"] = parse_state(state_lines)

                elif section == "goal":
                    goal_lines = lines[begin+1:end]
                    trans_dict["goal"] = parse_goal(goal_lines)
                
                elif section == "operator":
                    op_lines = lines[begin+1:end]
                    trans_dict["operator"].append(parse_operator(op_lines))

                elif section == "rule":
                    ax_lines = lines[begin+1:end]
                    trans_dict["axiom"].append(parse_axiom(ax_lines))
                
                else:
                    print("You shouldn't be here. Something's wrong.")
                    print(section)           
                    
            cur += 1

    return trans_dict

def parse_variable(lines):
    """ parse single variable section from sas file

    Args: Lines of strings containing the variable

    Returns: 
        Variable: named tuple with the following fields
            name: string variable name
            axiom_layer: int axiom layer
            atom_type: string "Atom" or "NegatedAtom"
            range: int range for the variable
            values: (str, [str, ...]) atom name and variables
    """
    var_name = lines[0][-2]
    axiom_layer = int(lines[1])
    var_range = int(lines[2])
    atom_vals = []
    for line in lines[3:]:
        # each line of format "Atom at(iiwa, t1)\n"
        i = 0
        while line[i] != " ":
            i += 1
        atom_type = line[:i]
        atom_fields = re.split(', |\(|\)', line[i+1:])

        atom_name = atom_fields[0]
        atom_vars = atom_fields[1:len(atom_fields)-1]
        atom_vals.append((atom_name, atom_vars))
    
    return Variable(name=var_name, 
                    axiom_layer=axiom_layer, 
                    atom_type=atom_type,
                    range=var_range,
                    values=atom_vals)

def parse_mutex_group(lines):
    """ parse single mutex group section

    Args: Lines of strings

    Returns:
        Mutex: namedtuple that contains the following fields
            n_facts: number of facts in the mutex group
            values: {str: int} dict mapping from var name to value
    """
    n_facts = int(lines[0])
    facts_dict = {}
    for line in lines[1:]:
        mutex_fields = re.split(" |\n", line)
        
        var_name = mutex_fields[0]
        var_val = int(mutex_fields[1])
        facts_dict[var_name] = var_val
    
    return Mutex(n_facts=n_facts, values=facts_dict)

def parse_state(lines):
    """ parse state section

    Args: Lines of strings

    Returns:
        HashableState: namedtuple that contains the following fields
            values: {str: int} dict mapping 
    """
    var_idx = 0
    var_dict = {}
    for line in lines:
        var_name = str(var_idx)
        var_dict[var_name] = int(line)
        var_idx += 1
    
    return HashableState(values=var_dict)

def parse_goal(lines):
    """ parse goal

    Args: Lines of strings

    Returns:
        Goal: namedtuple that contains the following fields
            n_goals: int number of goals
            values: {str: int} dict mapping variable to value
    """
    n_goals = int(lines[0])
    goal_dict = {}
    for line in lines[1:]:
        goal_fields = re.split(" |\n", line)
        var_name = goal_fields[0]
        var_val = int(goal_fields[1])
        goal_dict[var_name] = var_val
    
    return Goal(n_goals=n_goals, values=goal_dict)

def parse_operator(lines):
    """ parse single operator section

    Args: Lines of strings

    Returns:
        Operator: namedtuple that contains the following fields
            name: string name of the operator
            op_vars: list of strings, names of operator's variables
            preconds: {str: int} dict mapping variable names to values for preconditions
            effects: {str: int} dict mapping variable named to values for effects
            cost: int operator cost
    """
    name = ""
    op_vars = []
    precond_dict = {}
    effect_dict = {}
    cost = 0

    for cur in range(len(lines)):
        line = lines[cur]
        if cur == 0:
            fields = re.split(" |\n", line)
            name = fields[0]
            op_vars = fields[1:len(fields)-1]
        
        elif cur == 1:
            n_preconds = int(line)
        
        elif cur <= 1 + n_preconds:
            fields = re.split(" |\n", line)
            precond_dict[fields[0]] = int(fields[1])
        
        elif cur == 2 + n_preconds:
            n_effects = int(line)
        
        elif cur <= 2 + n_preconds + n_effects:
            fields = re.split(" |\n", line)
            if fields[0] == "0":
                var_name = fields[1]
                var_old_val = int(fields[2])
                var_new_val = int(fields[3])
                
                if var_old_val != -1:
                    precond_dict[var_name] = var_old_val
                
                effect_dict[var_name] = var_new_val
            
            else:
                print("Weird cases..")
            
        elif cur == len(lines) - 1:
            cost = int(line)
        
        else:
            print("You shouldn't be here. Something is wrong in parse_operator")
        
    return Operator(
        name = name,
        op_vars = op_vars,
        preconds = precond_dict,
        effects = effect_dict,
        cost = cost
    )

def parse_axiom(lines):
    cond_dict = {}
    effect_dict = {}
    for cur in range(len(lines)):
        line = lines[cur]
        if cur == 0:
            n_conds = int(lines[cur])
        elif cur <= n_conds:
            fields = re.split(" |\n", line)
            var_name = fields[0]
            var_val = int(fields[1])
            cond_dict[var_name] = var_val
        else:
            fields = re.split(" |\n", line)
            var_name = fields[0]
            var_old_val = int(fields[1])
            var_new_val = int(fields[2])

            if var_old_val != -1:
                cond_dict[var_name] = var_old_val

            effect_dict[var_name] = var_new_val
    
    return Axiom(conditions=cond_dict, effects=effect_dict)

