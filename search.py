# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for 
# educational purposes provided that (1) you do not distribute or publish 
# solutions, (2) you retain this notice, and (3) you provide clear 
# attribution to UC Berkeley, including a link to 
# http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html
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
import sys
import copy

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

def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.

    You are not required to implement this, but you may find it useful for Q5.
    """
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def iterativeDeepeningSearch(problem):
    """
    Perform DFS with increasingly larger depth.

    Begin with a depth of 1 and increment depth by 1 at every step.

    i = 1
    while i:
        possible_solution = depthFirstSearch(problem, i)
        if (possible_solution.pop() == 1):
            return possible_solution
        # print("Depth of " + i)
        i += 1
    """
    def recursive(state, problem, limit, solution, history):
        # print("limit = " + str(limit))
        # print(solution)
        if problem.isGoalState(state):
            return solution
        elif limit == 0:
            return []
        else:
            limit_reached = False
            test = problem.getSuccessors(state)
            for each in test:
                child = each[0]
                if child in history:
                    continue
                result = recursive(child, problem, limit - 1, solution + [each[1]], history + [child])
                if result == []:
                    limit_reached = True
                else:
                    return result
            if limit_reached:
                return []
            else:
                return []

    def depthFirstSearch(problem, limit = -1):
        history = [problem.getStartState()]
        return recursive(problem.getStartState(), problem, limit, [], history)


    i = 1
    while i:
        result = depthFirstSearch(problem, i)
        if result != []:
            return result
        i += 1
    return []

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    closed = set()
    fringe = util.PriorityQueue()
    start = problem.getStartState()
    curr = (start,[],0)
    heuristicValue = heuristic(curr[0],problem)
    priorityValue = heuristicValue + curr[2]
    fringe.push(curr,priorityValue)
    while True:
        if fringe.isEmpty():
            return []
        node = fringe.pop()
        if problem.isGoalState(node[0]):
            return node[1]
        if node[0] not in closed:
            closed.add(node[0])
            savedCost = node[2]
            for each in problem.getSuccessors(node[0]):
                cost = savedCost + each[2]
                path = node[1] + [each[1]] 
                newHeuristic = heuristic(each[0],problem)
                priorityValue = newHeuristic + cost
                fringe.push((each[0], path, cost),priorityValue)


# Abbreviations
bfs = breadthFirstSearch
astar = aStarSearch
ids = iterativeDeepeningSearch
