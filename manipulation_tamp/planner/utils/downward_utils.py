'''
    This is a utility file for interfacing with FastDownward Planner tools
'''
from __future__ import print_function
import os
import re
import sys
import time
import subprocess

try:
    # python 3
    from builtins import open
except ImportError:
    # python 2
    from codecs import open

from collections import namedtuple

# include fastdownward directory
filepath = os.path.abspath(__file__)
directory = os.path.dirname(filepath)
FD_PATH = os.path.join(directory, '../downward/')
# TRANSLATE_PATH = os.path.join(FD_PATH, 'builds/release/bin/translate/')
# sys.path.append(TRANSLATE_PATH)
sys.path.append(FD_PATH)

DOMAIN_INPUT = 'domain.pddl'
PROBLEM_INPUT = 'problem.pddl'
REL_TEMP_DIR = 'temp/'
TEMP_DIR = os.path.join(os.getcwd(), REL_TEMP_DIR)

TRANSLATE_FLAGS = []
original_argv = sys.argv[:]
# set system arguments to stop pddl_parser import errors
sys.argv = sys.argv[:1] + TRANSLATE_FLAGS + [DOMAIN_INPUT, PROBLEM_INPUT]

from downward_constants import Domain, Problem, Action, ExitCodes

# import pddl
# from pddl_parser import lisp_parser, parsing_functions
from driver import arguments, run_components, cleanup, aliases, returncodes


################################################################################

# def parse_lisp(lisp_file_name):
#     """ parse a lisp / pddl file into a nested list
    
#     Args:
#         lisp_file_name: filename (string)
#     Returns:
#         a nested list parsed from the file
#     Raises:
#         SystemExit: an error occured
#     """
#     try:
#         lisp = open(lisp_file_name)
#         return lisp_parser.parse_nested_list(lisp)
#     except IOError as e:
#         raise SystemExit("Error: Could not read file: %s\nReason: %s." %
#                          (e.filename, e))
#     except lisp_parser.ParseError as e:
#         raise SystemExit("Error: Could not parse file. Reason: %s" % e)

# def parse_domain(domain_list):
#     """ parse domain list into named tuple

#     Args:
#         domain_list: output from parse_lisp
#     Returns:
#         domain_pddl: namedtuple representing pddl_domain
#     """

#     domain_pddl = Domain(*parsing_functions.parse_domain_pddl(domain_list))

#     return domain_pddl

# def parse_problem(problem_list, domain_pddl):
#     """ parse problem list into named tuple

#     Args:
#         problem_list: output from parse_lisp
#         domain_pddl: namedtuple representing the domain
#     Returns:
#         problem_pddl: namedtuple representing pddl_domain
#     """
    
#     problem_pddl = Problem(*parsing_functions.parse_task_pddl(
#         problem_list,
#         domain_pddl.type_dict,
#         domain_pddl.predicate_dict
#     ))

#     return problem_pddl

def parse_arguments():
    """ parse arguments for FastDownward
    
    Returns:
        parsed argument in argparse.ArgumentParser object
    """
    return arguments.parse_args()

def parse_arguments_list(args_list):
    """ parse arguments for FastDownward
    
    Args:
        args_list: list of string arguments
    Returns:
        parsed argument in argparse.ArgumentParser object
    """
    sys.argv = args_list
    return parse_arguments()

def run_translate(args):
    """ Wrapper function for driver.run_components.run_translate

    Args:
        args: parsed argument in argparse.ArgumentParser object
    Returns:
        exitcode: int
        continue_execution: boolean
    File:
        output.sas file containing translation
    """
    (exitcode, continue_execution) = run_components.run_translate(args)
    return (exitcode, continue_execution)

def run_search(args):
    """ Wrapper function for driver.run_components.run_search

    Args:
        args: parsed argument in argparse.ArgumentParser object
    Returns:
        exitcode: int
        continue_execution: boolean
    File:
        sas_plan file containing found solution
    """
    (exitcode, continue_execution) = run_components.run_search(args)
    return (exitcode, continue_execution)

def run_validate(args):
    """ Wrapper function for driver.run_components.run_validate

    Args:
        args: parsed argument in argparse.ArgumentParser object
    Returns:
        exitcode: int
        continue_execution: boolean
    """
    (exitcode, continue_execution) = run_components.run_validate(args)
    return (exitcode, continue_execution)

def read_plan(sas_plan='sas_plan'):
    """ read sas_plan file

    Args:
        sas_plan: filename for sas_plan
    Returns:
        actions: list of Action namedtuple
    """
    f = open(sas_plan)
    actions_list = []
    for line in f:
        if line[0]==";":
            continue
        
        action_split = line[1:len(line)-1].split()
        for i in range(len(action_split)):
            edited = False
            word = action_split[i]
            if word[0] == '(':
                word = word[1:]
                edited = True
            elif word[-1] == ')':
                word = word[:-1]
                edited = True
            
            if edited:
                action_split[i] = word
                
        action = Action(action_split[0], action_split[1:])
        actions_list.append(action)
    
    return actions_list

def cleanup_temporary_files(args):
    """ A wrapper function for driver.cleanup.cleanup_temporary_files

    Args:
        args: parsed argument in argparse.ArgumentParser object
    """
    cleanup.cleanup_temporary_files(args)

def search_from_pddl(args, keep_sas_file=True):
    """ run FD_search
    Args:
        args: parsed argument in argparse.ArgumentParser object
        keep_sas_file: boolean, whether to keep intermediate file
    Returns:
        exitcode: int, exitcode, see downward.driver.returncodes
    """

    args.keep_sas_file=keep_sas_file
    exitcode = None
    for component in args.components:
        if component == "translate":
            (exitcode, continue_execution) = run_translate(args)
        elif component == "search":
            (exitcode, continue_execution) = run_search(args)
            if not args.keep_sas_file:
                print("Remove intermediate file {}".format(args.sas_file))
                os.remove(args.sas_file)
        elif component == "validate":
            (exitcode, continue_execution) = run_validate(args)
        else:
            assert False, "Error: unhandled component: {}".format(component)
        print("{component} exit code: {exitcode}".format(**locals()))
        print()
        if not continue_execution:
            print("Driver aborting after {}".format(component))
            break

    return exitcode

def get_exitcode_msg(exitcode):
    """ Get the message for an exitcode

    Args:
        exitcode: int
    Returns:
        msg: string, exitcode message
    """
    exit_msg = ExitCodes[exitcode]
    
    if returncodes.is_unrecoverable(exitcode):
        exit_msg = "UNRECOVERABLE!: " + exit_msg
    
    return exit_msg