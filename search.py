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

    # create fringe stack and visited list
    fringe = util.Stack()
    visited = []

    # push the start node on the fringe stack
    fringe.push((start, [], 1))

    while not fringe.isEmpty():
      n = fringe.pop()
      actions = n[1]

      visited.append(n[0])

      # check if goal state found
      if problem.isGoalState(n[0]):
        return actions

      for successor in problem.getSuccessors(n[0]):
          if not successor[0] in visited:
              successorActions = list(n[1])
              successorActions.append(successor[1])
              #print "add to fringe: ", (successor[0], testAct, successor[2])
              fringe.push((successor[0], successorActions, successor[2]))

    return actions

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    start = problem.getStartState()

    # create fringe queue and visited list
    fringe = util.Queue()
    visited = []
    succeeded = []

    # push the start node on the fringe queue
    fringe.push((start, [], 1))

    while not fringe.isEmpty():
        n = fringe.pop()
        actions = n[1]

        visited.append(n[0])

        # check if goal state found
        if problem.isGoalState(n[0]):
            return actions

        for successor in problem.getSuccessors(n[0]):
            if not successor[0] in visited:
                if not successor[0] in succeeded:
                    succeeded.append(successor[0])
                    successorActions = list(n[1])
                    successorActions.append(successor[1])
                    #print "add to fringe: ", (successor[0], successorActions, successor[2])
                    fringe.push((successor[0], successorActions, successor[2]))

    return actions

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    start = problem.getStartState()

    # create fringe queue and visited list
    fringe = util.PriorityQueue()
    visited = []
    succeeded = []

    # push the start node on the fringe queue
    fringe.push((start, [], 0), 0)

    while not fringe.isEmpty():
        n = fringe.pop()
        print "node: ", n
        actions = n[1]

        visited.append(n[0])

        # check if goal state found
        if problem.isGoalState(n[0]):
            return actions

        #print "successors: ", problem.getSuccessors(n[0])
        #print "current action on node: ", n[1]

        for successor in problem.getSuccessors(n[0]):
            #print "successors on %s : %s" % (str(n[0]), problem.getSuccessors(n[0]))
            if not successor[0] in visited:
                if (not successor[0] in succeeded) or (problem.isGoalState(successor[0])):
                    succeeded.append(successor[0])
                    successorActions = list(n[1])
                    successorActions.append(successor[1])
                    successorCost = n[2] + successor[2]
                    #print "pushing: ", (successor[0], successorCost)
                    fringe.push((successor[0], successorActions, successorCost), successorCost)
        #print "went through successors"

        # check if goal state found
        if problem.isGoalState(n[0]):
            return actions

    return actions

    """start = problem.getStartState()

    #print "Start: ", start
    #print "Is the start a goal?", problem.isGoalState(start)
    #print "Start's successors: ", problem.getSuccessors(start)

    # create fringe queue and visited list
    fringe = util.PriorityQueue()
    visited = []
    actions = []

    # if current state is goal state then return no actions
    if problem.isGoalState(start):
        return actions

    # push the start node on the fringe queue
    fringe.push((start, [], 1), 1)

    while not fringe.isEmpty():
        n = fringe.pop()
        actions = n[1]
        #print "actions 1: ", actions

        # check if goal path found
        if problem.isGoalState(n[0]):
            #print "actions: ", actions
            return actions

        for successor in problem.getSuccessors(n[0]):
            actions = list(n[1])
            if not (successor[0] in visited):
                visited.append(successor[0])
                #print "successor: ", successor
                actions.append(successor[1])
                fringe.push((successor[0], actions, successor[2]), successor[2])

    print "actions: ", actions
    return actions
    #util.raiseNotDefined()"""

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
