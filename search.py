# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
import math

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    start = problem.getStartState()
    c = problem.getStartState()
    exploredState = []
    exploredState.append(start)
    states = util.Stack()
    stateTuple = (start, [])
    states.push(stateTuple)
    while not states.isEmpty() and not problem.isGoalState(c):
        state, actions = states.pop()
        exploredState.append(state)
        successor = problem.getSuccessors(state)
        for i in successor:
            coordinates = i[0]
            if not coordinates in exploredState:
                c = i[0]
                direction = i[1]
                states.push((coordinates, actions + [direction]))
    return actions + [direction]
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    start = problem.getStartState()
    queue = util.Queue()
    explored = []
    explored.append(start)
    queue.push((start, []))
    while not queue.isEmpty():
        current, action = queue.pop()
        if problem.isGoalState(current):
            return action
        successors = problem.getSuccessors(current)
        for i in successors:
            coordinates = i[0]
            if not coordinates in explored:
                direction = i[1]
                explored.append(coordinates)
                queue.push((coordinates, action + [direction]))

    return action
    util.raiseNotDefined()


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    start = problem.getStartState()
    exploredState = []
    states = util.PriorityQueue()
    states.push((start, []), 0)
    while not states.isEmpty():
        state, actions = states.pop()
        if problem.isGoalState(state):
            return actions
        if state not in exploredState:
            successors = problem.getSuccessors(state)
            for succ in successors:
                coordinates = succ[0]
                if coordinates not in exploredState:
                    directions = succ[1]
                    newCost = actions + [directions]
                    states.push((coordinates, actions + [directions]), problem.getCostOfActions(newCost))
        exploredState.append(state)
    return actions
    util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    start = problem.getStartState()
    exploredState = []
    states = util.PriorityQueue()
    states.push((start, []), nullHeuristic(start, problem))
    nCost = 0
    while not states.isEmpty():
        state, actions = states.pop()
        if problem.isGoalState(state):
            return actions
        if state not in exploredState:
            successors = problem.getSuccessors(state)
            for succ in successors:
                coordinates = succ[0]
                if coordinates not in exploredState:
                    directions = succ[1]
                    nActions = actions + [directions]
                    nCost = problem.getCostOfActions(nActions) + heuristic(coordinates, problem)
                    states.push((coordinates, actions + [directions]), nCost)
        exploredState.append(state)
    return
    util.raiseNotDefined()

def get_succ_in_direction(direction, successors):
    for succ in successors:
        if succ[1] == direction:
            return succ
    return ((0,0), 'No', 0)

def directional_search(problem):
    start = problem.getStartState()
    explored = []
    action = []
    explored.append(start)
    current = start
    states = util.Stack()
    states.push((start, []))
    while not problem.isGoalState(current):
        successors = problem.getSuccessors(current)
        succ = get_succ_in_direction('West', successors)
        if (not (succ[0] in explored)) and (succ[1] != 'No'):
            action = action + [succ[1]]
            explored.append(current)
            current = succ[0]
            states.push((current, action))
        else:
            succ = get_succ_in_direction('East', successors)
            if (not (succ[0] in explored)) and (succ[1] != 'No'):
                action = action + [succ[1]]
                explored.append(current)
                current = succ[0]
                states.push((current, action))
            else:
                succ = get_succ_in_direction('North', successors)
                if (not (succ[0] in explored)) and (succ[1] != 'No'):
                    action = action + [succ[1]]
                    explored.append(current)
                    current = succ[0]
                    states.push((current, action))
                else:
                    succ = get_succ_in_direction('South', successors)
                    if (not (succ[0] in explored)) and (succ[1] != 'No'):
                        action = action + [succ[1]]
                        explored.append(current)
                        current = succ[0]
                        states.push((current, action))
                    else:
                        current, action = states.pop()
    return action

def reconstruct_path(s):
    total_path = []
    while s['action'] != None:
        total_path.append(s['action'])
        s = s['parent']
    total_path.reverse()
    return total_path

def c(a, b):
    return math.sqrt(((a[0] - b[0]) * (a[0] - b[0])) + ((a[1] - b[1]) * (a[1] - b[1])))

def update_vertex(s, neighbor):
    if s['action'] == neighbor['action']:
        if s['parent']['gScore'] + c(s['parent']['state'], neighbor['state']) < neighbor['gScore'] :
            neighbor['gScore'] = s['parent']['gScore'] + c(s['parent']['state'], neighbor['state'])
            neighbor['parent'] = s
    else :
        if s['gScore'] + c(s['state'], neighbor['state']) < neighbor['gScore'] :
            neighbor['gScore'] = s['gScore'] + c(s['state'], neighbor['state'])
            neighbor['parent'] = s


def thetaStarSearch(problem, heuristic=nullHeuristic):
    start = problem.getStartState()
    node = { 'state' : start, 'action' : None, 'cost' : 0, 'gScore' : 0, 'parent' : start}
    open = util.PriorityQueue()
    open.push((node), (node['gScore'] + heuristic(node['state'])))
    closed = []

    while not open.isEmpty():
        s= open.pop()
        if problem.isGoalState(s['state']):
            return reconstruct_path(s)
        closed.append(s['state'])
        successors = problem.getSuccessors(s['state'])
        for succ in successors:
            neighbor = {'state': succ[0], 'action': succ[1], 'cost': succ[2], 'gScore': 0, 'parent': s}
            if neighbor['state'] not in closed:
                update_vertex(s, neighbor)
                open.update(neighbor, neighbor['gScore'] + heuristic(neighbor))
    return None



def longest_search(problem):
    start = problem.getStartState()
    longest = {'action': [], 'cost': 0}
    node = {'state': start, 'actions': [], 'cost': 0, 'explored': [start]}
    stack = util.Stack()
    stack.push(node)
    while not stack.isEmpty():
        node= stack.pop()
        if problem.isGoalState(node['state']):
            if node['cost'] > longest['cost']:
                longest['action'] = node['actions']
                longest['cost'] = node['cost']
        successors = problem.getSuccessors(node['state'])
        for succ in successors:
            if succ[0] not in node['explored']:
                newNdoe = {'state': succ[0], 'actions': node['actions'] + [succ[1]], 'cost': node['cost'] + succ[2],
                                   'explored': node['explored'] + [succ[0]]}
                stack.push(newNdoe)
    return longest['action']
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
ls = longest_search
ds = directional_search
tstar = thetaStarSearch
