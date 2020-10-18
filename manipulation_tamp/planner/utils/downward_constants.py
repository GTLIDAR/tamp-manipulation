from collections import namedtuple

################################################################################
Domain = namedtuple('Domain', ['name', 'requirements', 'types', 'type_dict', 
                               'constants', 'predicates', 'predicate_dict', 
                               'functions', 'actions', 'axioms'])

Problem = namedtuple('Problem', ['task_name', 'task_domain_name', 'task_requirements',
                                 'objects', 'init', 'goal', 'use_metric'])

PDDLProblem = namedtuple('PDDLProblem', ['domain_pddl', 'constant_map',
                                         'stream_pddl', 'stream_map', 'init', 'goal'])
Solution = namedtuple('Solution', ['plan', 'cost', 'facts'])

Assignment =  namedtuple('Assignment', ['args'])
Action = namedtuple('Action', ['name', 'args'])
DurativeAction = namedtuple('DurativeAction', ['name', 'args', 'start', 'duration'])
################################################################################

# misc
PI = 3.1416
EQ = '=' # xnor

DEFAULT_MAX_TIME = 30 
DEFAULT_PLANNER = 'ff-astar'

TEMP_DIR = 'temp/'
TRANSLATE_OUTPUT = 'output.sas'
SEARCH_OUTPUT = 'sas_plan'
SEARCH_COMMAND = 'downward --internal-plan-file {} {} < {}'
INF = float('inf')
INFINITY = 'infinity'
GOAL_NAME = '@goal' # @goal-reachable

SEARCH_OPTIONS = {
    # Optimal
    'dijkstra': '--heuristic "h=blind(transform=adapt_costs(cost_type=NORMAL))" '
                '--search "astar(h,cost_type=NORMAL,max_time=%s,bound=%s)"',
    'max-astar': '--heuristic "h=hmax(transform=adapt_costs(cost_type=NORMAL))"'
                 ' --search "astar(h,cost_type=NORMAL,max_time=%s,bound=%s)"',
    'lmcut-astar': '--heuristic "h=lmcut(transform=adapt_costs(cost_type=NORMAL))"'
                 ' --search "astar(h,cost_type=NORMAL,max_time=%s,bound=%s)"',

    # Suboptimal
    'ff-astar': '--heuristic "h=ff(transform=adapt_costs(cost_type=NORMAL))" '
                '--search "astar(h,cost_type=NORMAL,max_time=%s,bound=%s)"',
    'ff-eager': '--heuristic "h=ff(transform=adapt_costs(cost_type=PLUSONE))" '
                '--search "eager_greedy([h],max_time=%s,bound=%s)"',
    'ff-eager-pref': '--heuristic "h=ff(transform=adapt_costs(cost_type=PLUSONE))" '
                     '--search "eager_greedy([h],preferred=[h],max_time=%s,bound=%s)"',
    'ff-lazy': '--heuristic "h=ff(transform=adapt_costs(cost_type=PLUSONE))" '
               '--search "lazy_greedy([h],preferred=[h],max_time=%s,bound=%s)"',
    'goal-lazy': '--heuristic "h=goalcount(transform=no_transform())" '
                 '--search "lazy_greedy([h],randomize_successors=True,max_time=%s,bound=%s)"',
    'add-random-lazy': '--heuristic "h=add(transform=adapt_costs(cost_type=PLUSONE))" '
                       '--search "lazy_greedy([h],randomize_successors=True,max_time=%s,bound=%s)"',

    'ff-eager-tiebreak': '--heuristic "h=ff(transform=no_transform())" '
                         '--search "eager(tiebreaking([h, g()]),reopen_closed=false,'
                         'cost_type=NORMAL,max_time=%s,bound=%s, f_eval=sum([g(), h]))"', # preferred=[h],
    'ff-lazy-tiebreak': '--heuristic "h=ff(transform=no_transform())" '
                         '--search "lazy(tiebreaking([h, g()]),reopen_closed=false,'
                         'randomize_successors=True,cost_type=NORMAL,max_time=%s,bound=%s)"',  # preferred=[h],

    'ff-ehc': '--heuristic "h=ff(transform=adapt_costs(cost_type=NORMAL))" '
              '--search "ehc(h,preferred=[h],preferred_usage=RANK_PREFERRED_FIRST,'
              'cost_type=NORMAL,max_time=%s,bound=%s)"',
    # The key difference is that ehc resets the open list upon finding an improvement
}

##################################################

# WARNING: overflow on h^add! Costs clamped to 100000000
MAX_FD_COST = 1e8


############################# exit codes #################################
ExitCodes = {
0: "SUCCESS",
1: "SEARCH_PLAN_FOUND_AND_OUT_OF_MEMORY",
2: "SEARCH_PLAN_FOUND_AND_OUT_OF_TIME",
3: "SEARCH_PLAN_FOUND_AND_OUT_OF_MEMORY_AND_TIME",

10: "TRANSLATE_UNSOLVABLE",
11: "SEARCH_UNSOLVABLE",
12: "SEARCH_UNSOLVED_INCOMPLETE",

20: "TRANSLATE_OUT_OF_MEMORY",
21: "TRANSLATE_OUT_OF_TIME",
22: "SEARCH_OUT_OF_MEMORY",
23: "SEARCH_OUT_OF_TIME",
24: "SEARCH_OUT_OF_MEMORY_AND_TIME",

30: "TRANSLATE_CRITICAL_ERROR",
31: "TRANSLATE_INPUT_ERROR",
32: "SEARCH_CRITICAL_ERROR",
33: "SEARCH_INPUT_ERROR",
34: "SEARCH_UNSUPPORTED",
35: "DRIVER_CRITICAL_ERROR",
36: "DRIVER_INPUT_ERROR",
37: "DRIVER_UNSUPPORTED"
}