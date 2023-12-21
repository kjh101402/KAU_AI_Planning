# searchAgents.py
# ---------------
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
from game import Directions
import os
import subprocess
import time

"""
This file contains all of the agents that can be selected to control Pacman.  To
select an agent, use the '-p' option when running pacman.py.  Arguments can be
passed to your agent using '-a'.  For example, to load a SearchAgent that uses
depth first search (dfs), run the following command:

> python pacman.py -p SearchAgent -a fn=depthFirstSearch

Commands to invoke other search strategies can be found in the project
description.

Please only change the parts of the file you are asked to.  Look for the lines
that say

"*** YOUR CODE HERE ***"

The parts you fill in start about 3/4 of the way down.  Follow the project
description for details.

Good luck and happy searching!
"""

from game import Directions
from game import Agent
from game import Actions
import util
import time
import search

class GoWestAgent(Agent):
    "An agent that goes West until it can't."

    def getAction(self, state):
        "The agent receives a GameState (defined in pacman.py)."
        if Directions.WEST in state.getLegalPacmanActions():
            return Directions.WEST
        else:
            return Directions.STOP

#######################################################
# This portion is written for you, but will only work #
#       after you fill in parts of search.py          #
#######################################################


class FFSearchAgent(Agent):
    def __init__(self, dpath='./pd.pddl', ppath='./auto.pddl', prob='oneshot', cycle=1):
        print('[SearchAgent] using function FFplanner')
        self.dpath = dpath          # PDDL Domain file path
        self.ppath = ppath          # PDDL Problem file path
        self.visited = []           # 팩맨이 방문한 칸
        self.problem = prob         # oneshot / search / pacman
        self.cycle = int(cycle)     # pacman에서 몇 번에 한번씩 플래너 돌릴지
        self.actionIndex = 0        # 액션 인덱스
    
    
    # Call ffPlanner
    def callFF(self):
        action = Directions.STOP
        south = Directions.SOUTH
        west = Directions.WEST
        east = Directions.EAST
        north = Directions.NORTH
        actions = []
        
        # FF Planner를 통해 solution 파일 생성
        # wait()을 통해 solution 파일 생성될 때까지 기다림
        subprocess.Popen(f"~/FF-v2.3/ff -o {self.dpath} -f {self.ppath}", shell=True).wait()

        
        # solution 파일을 열어서 액션 시퀀스 파싱
        with open(f'{self.ppath}.soln', 'r') as f:
            lines = f.readlines()
            # print(lines)
            for n, line in enumerate(lines):
                line = line.strip().rstrip(')').lstrip('(').split(' ')
                # print(f"{n}: {line}")
                if line[0] != 'MOVE' and line[0] != 'KILL':
                    continue
                now_x, now_y = map(int, line[2].split('_')[1:])
                if line[0] == 'MOVE':
                    next_x, next_y = map(int, line[3].split('_')[1:])
                elif line[0] == 'KILL':
                    next_x, next_y = map(int, line[4].split('_')[1:])
                
                if next_x - now_x == 1:
                    actions.append(east)
                elif next_x - now_x == -1:
                    actions.append(west)
                elif next_y - now_y == 1:
                    actions.append(north)
                elif next_y - now_y == -1:
                    actions.append(south)
        

        # 답이 안나올 때 기존 파일 다시 가져오는 경우가 있어서 속을 비움
        with open(f'{self.ppath}.soln', 'w') as f:
            f.write('')
        
        # 파싱한 액션 시퀀스 반환
        return actions
                    
                
    
    # PDDL의 도메인 파일 생성
    # 문제 유형에 따라 다르게 생성
    def generateDomain(self):
        with open(self.dpath, 'w') as f:
            lines = []
            
            # 도메인 명, requirements, type 부분
            lines.append('(define (domain PACMAN)\n')
            lines.append('\t(:requirements :strips :typing)\n')
            lines.append('\t(:types\n')
            lines.append('\t\tpacman\n')
            if self.problem == 'pacman':
                lines.append('\t\tghost\n')
            if self.problem != 'oneshot':
                lines.append('\t\tfood\n')
            lines.append('\t\tlocation)\n')
            lines.append('\n')
            
            # predicate 부분
            lines.append('\t(:predicates\n')
            lines.append('\t\t(connected ?l1 ?l2 - location)\n')
            lines.append('\t\t(pacman-at ?l - location)\n')
            if self.problem == 'pacman':
                lines.append('\t\t(bomb-at ?l - location)\n')
                lines.append('\t\t(weak ?g - ghost)')
                lines.append('\t\t(food-at ?l - location)\n')
                lines.append('\t\t(ghost-at ?g - ghost ?l - location)\n')
                
                # lines.append('\t\t(warning ?l - location)\n')
                
            elif self.problem == 'oneshot':
                lines.append('\t\t(visited ?p - pacman ?l - location)\n')
                
            elif self.problem == 'search':
                lines.append('\t\t(food-at ?l - location)\n')

            lines.append('\t)\n')
            lines.append('\n')


            # Action 부분
            lines.append('\t(:action move\n')
            lines.append('\t\t:parameters (?p - pacman ?from ?to - location)\n')
            if self.problem == 'oneshot':
                lines.append('\t\t:precondition (and  (connected ?from ?to) (pacman-at ?from) (not (visited ?p ?to)))\n')
                lines.append('\t\t:effect (and (pacman-at ?to) (not (pacman-at ?from)) (visited ?p ?to))\n')
            elif self.problem == 'search':
                lines.append('\t\t:precondition (and (connected ?from ?to) (pacman-at ?from))\n')
                lines.append('\t\t:effect (and (pacman-at ?to) (not (pacman-at ?from)))\n')
            elif self.problem == 'pacman':
                
                # lines.append('\t\t:precondition (and (not (warning ?to)) (connected ?from ?to) (pacman-at ?from) (forall (?g - ghost) (not (ghost-at ?g ?to))))\n')
                
                lines.append('\t\t:precondition (and (connected ?from ?to) (pacman-at ?from) (forall (?g - ghost) (not (ghost-at ?g ?to))))\n')
                lines.append('\t\t:effect (and (pacman-at ?to) (not (pacman-at ?from)))\n')
            lines.append('\t)\n')
            lines.append('\n')
            
            
            
            if self.problem == 'pacman':
                lines.append('\t(:action kill\n')
                lines.append('\t\t:parameters (?p - pacman ?pl - location ?g - ghost ?gl - location)\n')
                lines.append('\t\t:precondition (and (pacman-at ?pl) (connected ?pl ?gl) (ghost-at ?g ?gl) (weak ?g))\n')
                lines.append('\t\t:effect (and (not (pacman-at ?pl)) (pacman-at ?gl) (not (ghost-at ?g ?gl)) (not (weak ?g)))\n')
                lines.append('\t)\n\n')
                
                lines.append('\t(:action power\n')
                lines.append('\t\t:parameters (?p - pacman ?l - location)\n')
                lines.append('\t\t:precondition (and (pacman-at ?l) (bomb-at ?l))\n')
                lines.append('\t\t:effect (and (not (bomb-at ?l)) (forall (?g - ghost) (weak ?g)))\n')
                lines.append('\t)\n\n')
                
            
            if self.problem != 'oneshot':
                lines.append('\t(:action eat\n')
                lines.append('\t\t:parameters (?p - pacman ?f - food ?l - location)\n')
                lines.append('\t\t:precondition (and (pacman-at ?l) (food-at ?l))\n')
                lines.append('\t\t:effect (and (not (food-at ?l)))\n')
                lines.append('\t)\n\n')

            lines.append(')\n')
            
            f.writelines(lines)

    
    # PDDL problem objects
    # 문제에 존재하는 인스턴스들
    def PDDLObjects(self, state):
        objects = '\t\tp1 - pacman\n\t\t'
        walls = state.getWalls()
        # Location (모두 동일)
        for x in range(1, state.data.layout.width-1):
            for y in range(1, state.data.layout.height-1):
                location = f'L_{x}_{y} '
                if walls[x][y]:
                    location = ' ' * len(location)
                    
                objects = objects + location
            
            objects = objects + '- location\n\t\t'
            
        # food (pacman, search)
        if self.problem != 'oneshot':
            foods = state.getFood().asList()
            for i in range(len(foods)):
                food = f'f{i+1} '
                objects = objects + food
            if len(foods) > 0:
                objects = objects + '- food\n'
            
        #bomb (capsule)
        if self.problem == 'pacman':
            bombs = state.getCapsules()
            for i in range(len(bombs)):
                bomb = f'b{i+1} '
                objects = objects + bomb
            if len(bombs) > 0:
                objects = objects + '- bomb\n\t\t'
            
        #ghost
        if self.problem == 'pacman':
            ghosts = state.getGhostStates()
            for i in range(len(ghosts)):
                ghost = f'g{i+1} '
                objects = objects + ghost
            if len(ghosts) > 0:
                objects = objects + '- ghost\n\t\t'
        
        objects = objects + '\n'
        return objects
    
    
    # PDDL problem init (문제의 현재 상태)
    def PDDLInit(self, state):
        init = '\t\t'
        
        # pacman-at ?l - location (공통) (팩맨의 위치)
        x, y = state.getPacmanPosition()
        pacman = f'(pacman-at L_{x}_{y})\n\n\t\t'
        init = init + pacman
        
        
        if self.problem == 'pacman':
            # bomb-at ?l - location (폭탄(캡슐)의 위치)
            bombs = state.getCapsules()
            for x, y in bombs:
                bomb = f'(bomb-at L_{x}_{y})\n\t\t'
                init = init + bomb
            if len(bombs) > 0:
                init = init + '\n\t\t'
        
        
            # ghost-at ?g -ghost ?l - location (유령의 위치)
            # warning ?l - location (유령 근처 위험) - 넣어서 좋을 때도 있고 아닐 때도 있음
            ghosts = state.getGhostStates()
            
            # walls = state.getWalls()
            # warnings = []
            
            for idx, g in enumerate(ghosts):
                ghost_number = f'g{idx+1}'
                weak = '' if g.scaredTimer == 0 else f'(weak {ghost_number})'
                x, y = map(int, g.getPosition())
                
                # spooky = ''
                # for dx, dy in [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]:
                #     if walls[dx][dy]: continue
                #     warn = f'L_{dx}_{dy}'
                #     if warn not in warnings:
                #         warnings.append(warn)
                #         spooky = spooky + f'(warning {warn}) '
                # ghost = f'{weak} (ghost-at {ghost_number} L_{x}_{y}) {spooky}\n\t\t'
                    
                ghost = f'{weak} (ghost-at {ghost_number} L_{x}_{y})\n\t\t'
                init = init + ghost
            if len(ghosts) > 0:
                init = init + '\n\t\t'
        
                
        # food-at ?l - location (음식의 위치)
        if self.problem != 'oneshot':
            foods = state.getFood().asList()
            for x, y in foods:
                food = f'(food-at L_{x}_{y})\n\t\t'
                init = init + food
            if len(foods) > 0:
                init = init + '\n\t\t'
        
        
        if self.problem == 'oneshot':
        # visited ?p - pacman ?l - location (한붓그리기, 방문 이력)
            x, y = state.getPacmanPosition()
            position = f'L_{x}_{y}'
            self.visited.append(position)
        
            for v in self.visited:
                init = init + f'(visited p1 {v})\n\t\t'
            init = init + '\n\t\t'
        
        
        # connected ?l1 ?l2 - location (공통) (칸의 연결 여부)
        walls = state.getWalls()
        for x in range(1, state.data.layout.width):
            for y in range(1, state.data.layout.height):
                if walls[x][y]: continue
                
                now = f'L_{x}_{y}'
                for dx, dy in [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]:
                    if walls[dx][dy]: continue
                    adj = f'L_{dx}_{dy}'
                    init = init + f'(connected {now} {adj})\n\t\t'
        init = init + '\n'
        
        return init
    
    
    # 맨해튼 거리 반환 함수
    def ManhattanDistance(self, food, ghost):
        xy1 = food
        xy2 = ghost
        return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])
    
    
    # PDDL problem goal
    def PDDLGoal(self, state):
        # 한붓 그리기 문제의 목표, 모든 칸 방문하기
        if self.problem == 'oneshot':
            goal = '(forall (?p - pacman ?l - location) (visited ?p ?l))'
        
        # 팩맨의 목표
        # 모든 음식 먹기지만 Sub Goal만 명시
        # 유령한테서 가장 먼 음식을 우선 목표로
        elif self.problem == 'pacman':
            foods = state.getFood().asList()
            ghosts = state.getGhostStates()
            best = 0
            goal = '(forall (?l - location) (not (food-at ?l)))'
            for food in foods:
                MDsum = 0
                for ghost in ghosts:
                    x, y = map(int, ghost.getPosition())
                    MDsum += self.ManhattanDistance(food, (x, y))
                if best < MDsum:
                    best = MDsum
                    goal = f'(not (food-at L_{food[0]}_{food[1]}))'
            # goal = '(forall (?l - location) (not (food-at ?l)))'
        
        # 탐색은 유령이 없다고 가정해서 모든 음식 먹기를 목표로 명시
        elif self.problem == 'search':
            goal = '(forall (?l - location) (not (food-at ?l)))'
        return goal

    
    # PDDL Problem 파일 생성
    def generateProblem(self, state):
        with open(self.ppath, 'w') as f:
            lines = []
            
            lines.append('(define (problem PacmanProblem)\n')
            lines.append('\t(:domain PACMAN)\n')
            lines.append('\t(:objects\n')
            lines.append(self.PDDLObjects(state))
            lines.append('\t)\n\n')
            
            lines.append('\t(:init\n')
            lines.append(self.PDDLInit(state))
            lines.append('\t)\n\n')
            
            lines.append('\t(:goal')
            lines.append(self.PDDLGoal(state))
            lines.append('\t)\n)\n')
            
            f.writelines(lines)


    # 초기 상태
    def registerInitialState(self, state):
        """
        This is the first time that the agent sees the layout of the game
        board. Here, we choose a path to the goal. In this phase, the agent
        should compute the path to the goal and store it in a local variable.
        All of the work is done in this method!

        state: a GameState object (pacman.py)
        """
        # 상태와 문제에 맞게 도메인과 문제 PDDL 파일을 생성하고 
        # FF Planner를 통해 답을 찾아낸다
        self.generateDomain()
        self.generateProblem(state)
        self.actions  = self.callFF() # Find a path


    # 매 턴마다 액션을 가져오는 함수
    def getAction(self, state):
        """
        Returns the next action in the path chosen earlier (in
        registerInitialState).  Return Directions.STOP if there is no further
        action to take.

        state: a GameState object (pacman.py)
        """

        # 상태가 변하지 않는 search, oneshot 문제
        # registerInitialState를 통해 구한 액션 시퀀스를 끝까지 이용
        if self.problem != 'pacman':
            if 'actionIndex' not in dir(self): self.actionIndex = 0
            i = self.actionIndex
            self.actionIndex += 1
            if i < len(self.actions):
                return self.actions[i]
            else:
                return Directions.STOP
        
        # 상태가 수시로 변하는 pacman 문제
        # 사이클마다 플래너를 다시 돌린다
        else:
            # 사이클 도래시 플래너 돌리기
            if self.actionIndex % self.cycle == 0:
                self.generateProblem(state)
                actions = self.callFF() # Find a path
                # 답이 나오면 갖고 있고 인덱스 0으로 초기화
                if len(actions) > 0:
                    self.actions = actions
                    self.actionIndex = 0
            # print(actions)
            # 사이클 아니면 기존 액션 승계
            else:
                actions = self.actions
                
            # 답에 따라서 인덱스 결정
            # 답이 있으면 인덱스를 사이클로 나눈 나머지를 사용
            if len(actions) != 0:
                i = self.actionIndex % self.cycle
                self.actionIndex += 1
            # 답이 없으면 그대로 써서 이전 액션 시퀀스를 사용
            else:
                i = self.actionIndex
                self.actionIndex += 1
                
            # print(f'{self.actionIndex}: {actions[:20]}')
            print(f'{i}: {self.actions[:20]}')
            
            # 액션 시퀀스 길이 내에 있으면 그대로 반환, 아니면 정지
            if i < len(self.actions):
                return self.actions[i]
            else:
                return Directions.STOP
            
        

        
        

class SearchAgent(Agent):
    """
    This very general search agent finds a path using a supplied search
    algorithm for a supplied search problem, then returns actions to follow that
    path.

    As a default, this agent runs DFS on a PositionSearchProblem to find
    location (1,1)

    Options for fn include:
      depthFirstSearch or dfs
      breadthFirstSearch or bfs


    Note: You should NOT change any code in SearchAgent
    """

    def __init__(self, fn='depthFirstSearch', prob='PositionSearchProblem', heuristic='nullHeuristic'):
        # Warning: some advanced Python magic is employed below to find the right functions and problems

        # Get the search function from the name and heuristic
        if fn not in dir(search):
            raise AttributeError(fn + ' is not a search function in search.py.')
        func = getattr(search, fn)
        if 'heuristic' not in func.__code__.co_varnames:
            print('[SearchAgent] using function ' + fn)
            self.searchFunction = func
        else:
            if heuristic in globals().keys():
                heur = globals()[heuristic]
            elif heuristic in dir(search):
                heur = getattr(search, heuristic)
            else:
                raise AttributeError(heuristic + ' is not a function in searchAgents.py or search.py.')
            print('[SearchAgent] using function %s and heuristic %s' % (fn, heuristic))
            # Note: this bit of Python trickery combines the search algorithm and the heuristic
            self.searchFunction = lambda x: func(x, heuristic=heur)

        # Get the search problem type from the name
        if prob not in globals().keys() or not prob.endswith('Problem'):
            raise AttributeError(prob + ' is not a search problem type in SearchAgents.py.')
        self.searchType = globals()[prob]
        print('[SearchAgent] using problem type ' + prob)

    def registerInitialState(self, state):
        """
        This is the first time that the agent sees the layout of the game
        board. Here, we choose a path to the goal. In this phase, the agent
        should compute the path to the goal and store it in a local variable.
        All of the work is done in this method!

        state: a GameState object (pacman.py)
        """
        if self.searchFunction == None: raise Exception("No search function provided for SearchAgent")
        starttime = time.time()
        # print('state==============')
        # print(state.data.layout)
        # # print('food', state.data.food.asList())
        # print('food', state.getFood())
        # print('capsule', state.getCapsules())
        # print('height:' , state.data.layout.height)
        # print('width', state.data.layout.width)
        # print(state.getWalls())
        # for x in range(state.data.layout.width):
        #     for y in range(state.data.layout.height):
        #         print(f"({x}, {y}) : {state.getWalls()[x][y]}")
        # print('state==============')
        problem = self.searchType(state) # Makes a new search problem
        self.actions  = self.searchFunction(problem) # Find a path
        totalCost = problem.getCostOfActions(self.actions)
        print('Path found with total cost of %d in %.1f seconds' % (totalCost, time.time() - starttime))
        if '_expanded' in dir(problem): print('Search nodes expanded: %d' % problem._expanded)

    def getAction(self, state):
        """
        Returns the next action in the path chosen earlier (in
        registerInitialState).  Return Directions.STOP if there is no further
        action to take.

        state: a GameState object (pacman.py)
        """
        # print('state==============')
        # # print(state.data.layout)   # not change
        # print(state.getPacmanPosition())
        # print(state.getPacmanState())
        # print('capsule eaten: ', state.data._capsuleEaten)
        # # print(state.data.agentStates)
        # print(state.getGhostPositions())
        # # print('agent states: ', state.data.agentStates[1].scaredTimer)
        # print('state==============')
        if 'actionIndex' not in dir(self): self.actionIndex = 0
        i = self.actionIndex
        self.actionIndex += 1
        if i < len(self.actions):
            return self.actions[i]
        else:
            return Directions.STOP
        
    

class PositionSearchProblem(search.SearchProblem):
    """
    A search problem defines the state space, start state, goal test, successor
    function and cost function.  This search problem can be used to find paths
    to a particular point on the pacman board.

    The state space consists of (x,y) positions in a pacman game.

    Note: this search problem is fully specified; you should NOT change it.
    """

    def __init__(self, gameState, costFn = lambda x: 1, goal=(1,1), start=None, warn=True, visualize=True):
        """
        Stores the start and goal.

        gameState: A GameState object (pacman.py)
        costFn: A function from a search state (tuple) to a non-negative number
        goal: A position in the gameState
        """
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition()
        if start != None: self.startState = start
        self.goal = goal
        self.costFn = costFn
        self.visualize = visualize
        if warn and (gameState.getNumFood() != 1 or not gameState.hasFood(*goal)):
            print('Warning: this does not look like a regular search maze')

        # For display purposes
        self._visited, self._visitedlist, self._expanded = {}, [], 0 # DO NOT CHANGE

    def getStartState(self):
        return self.startState

    def isGoalState(self, state):
        isGoal = state == self.goal

        # For display purposes only
        if isGoal and self.visualize:
            self._visitedlist.append(state)
            import __main__
            if '_display' in dir(__main__):
                if 'drawExpandedCells' in dir(__main__._display): #@UndefinedVariable
                    __main__._display.drawExpandedCells(self._visitedlist) #@UndefinedVariable

        return isGoal

    def getSuccessors(self, state):
        """
        Returns successor states, the actions they require, and a cost of 1.

         As noted in search.py:
             For a given state, this should return a list of triples,
         (successor, action, stepCost), where 'successor' is a
         successor to the current state, 'action' is the action
         required to get there, and 'stepCost' is the incremental
         cost of expanding to that successor
        """

        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x,y = state
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextState = (nextx, nexty)
                cost = self.costFn(nextState)
                successors.append( ( nextState, action, cost) )

        # Bookkeeping for display purposes
        self._expanded += 1 # DO NOT CHANGE
        if state not in self._visited:
            self._visited[state] = True
            self._visitedlist.append(state)

        return successors

    def getCostOfActions(self, actions):
        """
        Returns the cost of a particular sequence of actions. If those actions
        include an illegal move, return 999999.
        """
        if actions == None: return 999999
        x,y= self.getStartState()
        cost = 0
        for action in actions:
            # Check figure out the next state and see whether its' legal
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]: return 999999
            cost += self.costFn((x,y))
        return cost

class StayEastSearchAgent(SearchAgent):
    """
    An agent for position search with a cost function that penalizes being in
    positions on the West side of the board.

    The cost function for stepping into a position (x,y) is 1/2^x.
    """
    def __init__(self):
        self.searchFunction = search.uniformCostSearch
        costFn = lambda pos: .5 ** pos[0]
        self.searchType = lambda state: PositionSearchProblem(state, costFn, (1, 1), None, False)

class StayWestSearchAgent(SearchAgent):
    """
    An agent for position search with a cost function that penalizes being in
    positions on the East side of the board.

    The cost function for stepping into a position (x,y) is 2^x.
    """
    def __init__(self):
        self.searchFunction = search.uniformCostSearch
        costFn = lambda pos: 2 ** pos[0]
        self.searchType = lambda state: PositionSearchProblem(state, costFn)

def manhattanHeuristic(position, problem, info={}):
    "The Manhattan distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goal
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])

def euclideanHeuristic(position, problem, info={}):
    "The Euclidean distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goal
    return ( (xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2 ) ** 0.5

#####################################################
# This portion is incomplete.  Time to write code!  #
#####################################################

class CornersProblem(search.SearchProblem):
    """
    This search problem finds paths through all four corners of a layout.

    You must select a suitable state space and successor function
    """

    def __init__(self, startingGameState):
        """
        Stores the walls, pacman's starting position and corners.
        """
        self.walls = startingGameState.getWalls()
        self.startingPosition = startingGameState.getPacmanPosition()
        top, right = self.walls.height-2, self.walls.width-2
        self.corners = ((1,1), (1,top), (right, 1), (right, top))
        for corner in self.corners:
            if not startingGameState.hasFood(*corner):
                print('Warning: no food in corner ' + str(corner))
        self._expanded = 0 # DO NOT CHANGE; Number of search nodes expanded
        # Please add any code here which you would like to use
        # in initializing the problem
        "*** YOUR CODE HERE ***"

    def getStartState(self):
        """
        Returns the start state (in your state space, not the full Pacman state
        space)
        """
        "*** YOUR CODE HERE ***"
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
        Returns whether this search state is a goal state of the problem.
        """
        "*** YOUR CODE HERE ***"
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
        Returns successor states, the actions they require, and a cost of 1.

         As noted in search.py:
            For a given state, this should return a list of triples, (successor,
            action, stepCost), where 'successor' is a successor to the current
            state, 'action' is the action required to get there, and 'stepCost'
            is the incremental cost of expanding to that successor
        """

        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            # Add a successor state to the successor list if the action is legal
            # Here's a code snippet for figuring out whether a new position hits a wall:
            #   x,y = currentPosition
            #   dx, dy = Actions.directionToVector(action)
            #   nextx, nexty = int(x + dx), int(y + dy)
            #   hitsWall = self.walls[nextx][nexty]

            "*** YOUR CODE HERE ***"

        self._expanded += 1 # DO NOT CHANGE
        return successors

    def getCostOfActions(self, actions):
        """
        Returns the cost of a particular sequence of actions.  If those actions
        include an illegal move, return 999999.  This is implemented for you.
        """
        if actions == None: return 999999
        x,y= self.startingPosition
        for action in actions:
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]: return 999999
        return len(actions)


def cornersHeuristic(state, problem):
    """
    A heuristic for the CornersProblem that you defined.

      state:   The current search state
               (a data structure you chose in your search problem)

      problem: The CornersProblem instance for this layout.

    This function should always return a number that is a lower bound on the
    shortest path from the state to a goal of the problem; i.e.  it should be
    admissible (as well as consistent).
    """
    corners = problem.corners # These are the corner coordinates
    walls = problem.walls # These are the walls of the maze, as a Grid (game.py)

    "*** YOUR CODE HERE ***"
    return 0 # Default to trivial solution

class AStarCornersAgent(SearchAgent):
    "A SearchAgent for FoodSearchProblem using A* and your foodHeuristic"
    def __init__(self):
        self.searchFunction = lambda prob: search.aStarSearch(prob, cornersHeuristic)
        self.searchType = CornersProblem

class FoodSearchProblem:
    """
    A search problem associated with finding the a path that collects all of the
    food (dots) in a Pacman game.

    A search state in this problem is a tuple ( pacmanPosition, foodGrid ) where
      pacmanPosition: a tuple (x,y) of integers specifying Pacman's position
      foodGrid:       a Grid (see game.py) of either True or False, specifying remaining food
    """
    def __init__(self, startingGameState):
        self.start = (startingGameState.getPacmanPosition(), startingGameState.getFood())
        self.walls = startingGameState.getWalls()
        self.startingGameState = startingGameState
        self._expanded = 0 # DO NOT CHANGE
        self.heuristicInfo = {} # A dictionary for the heuristic to store information

    def getStartState(self):
        return self.start

    def isGoalState(self, state):
        return state[1].count() == 0

    def getSuccessors(self, state):
        "Returns successor states, the actions they require, and a cost of 1."
        successors = []
        self._expanded += 1 # DO NOT CHANGE
        for direction in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x,y = state[0]
            dx, dy = Actions.directionToVector(direction)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextFood = state[1].copy()
                nextFood[nextx][nexty] = False
                successors.append( ( ((nextx, nexty), nextFood), direction, 1) )
        return successors

    def getCostOfActions(self, actions):
        """Returns the cost of a particular sequence of actions.  If those actions
        include an illegal move, return 999999"""
        x,y= self.getStartState()[0]
        cost = 0
        for action in actions:
            # figure out the next state and see whether it's legal
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]:
                return 999999
            cost += 1
        return cost

class AStarFoodSearchAgent(SearchAgent):
    "A SearchAgent for FoodSearchProblem using A* and your foodHeuristic"
    def __init__(self):
        self.searchFunction = lambda prob: search.aStarSearch(prob, foodHeuristic)
        self.searchType = FoodSearchProblem

def foodHeuristic(state, problem):
    """
    Your heuristic for the FoodSearchProblem goes here.

    This heuristic must be consistent to ensure correctness.  First, try to come
    up with an admissible heuristic; almost all admissible heuristics will be
    consistent as well.

    If using A* ever finds a solution that is worse uniform cost search finds,
    your heuristic is *not* consistent, and probably not admissible!  On the
    other hand, inadmissible or inconsistent heuristics may find optimal
    solutions, so be careful.

    The state is a tuple ( pacmanPosition, foodGrid ) where foodGrid is a Grid
    (see game.py) of either True or False. You can call foodGrid.asList() to get
    a list of food coordinates instead.

    If you want access to info like walls, capsules, etc., you can query the
    problem.  For example, problem.walls gives you a Grid of where the walls
    are.

    If you want to *store* information to be reused in other calls to the
    heuristic, there is a dictionary called problem.heuristicInfo that you can
    use. For example, if you only want to count the walls once and store that
    value, try: problem.heuristicInfo['wallCount'] = problem.walls.count()
    Subsequent calls to this heuristic can access
    problem.heuristicInfo['wallCount']
    """
    position, foodGrid = state
    "*** YOUR CODE HERE ***"
    return 0

class ClosestDotSearchAgent(SearchAgent):
    "Search for all food using a sequence of searches"
    def registerInitialState(self, state):
        self.actions = []
        currentState = state
        while(currentState.getFood().count() > 0):
            nextPathSegment = self.findPathToClosestDot(currentState) # The missing piece
            self.actions += nextPathSegment
            for action in nextPathSegment:
                legal = currentState.getLegalActions()
                if action not in legal:
                    t = (str(action), str(currentState))
                    raise Exception('findPathToClosestDot returned an illegal move: %s!\n%s' % t)
                currentState = currentState.generateSuccessor(0, action)
        self.actionIndex = 0
        print('Path found with cost %d.' % len(self.actions))

    def findPathToClosestDot(self, gameState):
        """
        Returns a path (a list of actions) to the closest dot, starting from
        gameState.
        """
        # Here are some useful elements of the startState
        startPosition = gameState.getPacmanPosition()
        food = gameState.getFood()
        walls = gameState.getWalls()
        problem = AnyFoodSearchProblem(gameState)

        "*** YOUR CODE HERE ***"
        util.raiseNotDefined()

class AnyFoodSearchProblem(PositionSearchProblem):
    """
    A search problem for finding a path to any food.

    This search problem is just like the PositionSearchProblem, but has a
    different goal test, which you need to fill in below.  The state space and
    successor function do not need to be changed.

    The class definition above, AnyFoodSearchProblem(PositionSearchProblem),
    inherits the methods of the PositionSearchProblem.

    You can use this search problem to help you fill in the findPathToClosestDot
    method.
    """

    def __init__(self, gameState):
        "Stores information from the gameState.  You don't need to change this."
        # Store the food for later reference
        self.food = gameState.getFood()

        # Store info for the PositionSearchProblem (no need to change this)
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition()
        self.costFn = lambda x: 1
        self._visited, self._visitedlist, self._expanded = {}, [], 0 # DO NOT CHANGE

    def isGoalState(self, state):
        """
        The state is Pacman's position. Fill this in with a goal test that will
        complete the problem definition.
        """
        x,y = state

        "*** YOUR CODE HERE ***"
        util.raiseNotDefined()

def mazeDistance(point1, point2, gameState):
    """
    Returns the maze distance between any two points, using the search functions
    you have already built. The gameState can be any game state -- Pacman's
    position in that state is ignored.

    Example usage: mazeDistance( (2,4), (5,6), gameState)

    This might be a useful helper function for your ApproximateSearchAgent.
    """
    x1, y1 = point1
    x2, y2 = point2
    walls = gameState.getWalls()
    assert not walls[x1][y1], 'point1 is a wall: ' + str(point1)
    assert not walls[x2][y2], 'point2 is a wall: ' + str(point2)
    prob = PositionSearchProblem(gameState, start=point1, goal=point2, warn=False, visualize=False)
    return len(search.bfs(prob))
