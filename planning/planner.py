from __future__ import annotations

from collections.abc import Callable

from planning.pddl import (
    Action,
    ActionSchema,
    Problem,
    State,
    Objects,
    get_all_groundings,
)
from planning.utils import Queue, PriorityQueue
from planning.heuristics import nullHeuristic


# ---------------------------------------------------------------------------
# Reference implementation – read and understand before coding the rest.
# ---------------------------------------------------------------------------


def tinyBaseSearch(problem: Problem) -> list[Action]:
    """
    Hardcoded plan for the tinyBase layout.
    The robot at (1,4) must: pick up supplies at (1,3), set them up at (1,2),
    pick up the patient at (1,1), bring them to (1,2), and execute Rescue.

    Useful to understand the Action object format and plan structure.
    """
    robot = "robot"
    supplies = "supplies_0"
    patient = "patient_0"

    c14 = (1, 4)  # robot start
    c13 = (1, 3)  # supplies
    c12 = (1, 2)  # medical post
    c11 = (1, 1)  # patient

    plan = [
        Action(
            "Move(robot,(1,4),(1,3))",
            [("At", robot, c14), ("Adjacent", c14, c13), ("Free", c13)],
            [],
            [("At", robot, c13), ("Free", c14)],
            [("At", robot, c14), ("Free", c13)],
        ),
        Action(
            "PickUp(robot,supplies_0,(1,3))",
            [
                ("At", robot, c13),
                ("At", supplies, c13),
                ("HandsFree", robot),
                ("Pickable", supplies),
            ],
            [],
            [("Holding", robot, supplies)],
            [("At", supplies, c13), ("HandsFree", robot)],
        ),
        Action(
            "Move(robot,(1,3),(1,2))",
            [("At", robot, c13), ("Adjacent", c13, c12), ("Free", c12)],
            [],
            [("At", robot, c12), ("Free", c13)],
            [("At", robot, c13), ("Free", c12)],
        ),
        Action(
            "SetupSupplies(robot,supplies_0,(1,2))",
            [("At", robot, c12), ("MedicalPost", c12), ("Holding", robot, supplies)],
            [("SuppliesReady", c12)],
            [("SuppliesReady", c12), ("HandsFree", robot)],
            [("Holding", robot, supplies)],
        ),
        Action(
            "Move(robot,(1,2),(1,1))",
            [("At", robot, c12), ("Adjacent", c12, c11), ("Free", c11)],
            [],
            [("At", robot, c11), ("Free", c12)],
            [("At", robot, c12), ("Free", c11)],
        ),
        Action(
            "PickUp(robot,patient_0,(1,1))",
            [
                ("At", robot, c11),
                ("At", patient, c11),
                ("HandsFree", robot),
                ("Pickable", patient),
            ],
            [],
            [("Holding", robot, patient)],
            [("At", patient, c11), ("HandsFree", robot)],
        ),
        Action(
            "Move(robot,(1,1),(1,2))",
            [("At", robot, c11), ("Adjacent", c11, c12), ("Free", c12)],
            [],
            [("At", robot, c12), ("Free", c11)],
            [("At", robot, c11), ("Free", c12)],
        ),
        Action(
            "PutDown(robot,patient_0,(1,2))",
            [("At", robot, c12), ("Holding", robot, patient)],
            [],
            [("At", patient, c12), ("HandsFree", robot)],
            [("Holding", robot, patient)],
        ),
        Action(
            "Rescue(robot,patient_0,(1,2))",
            [
                ("At", robot, c12),
                ("At", patient, c12),
                ("MedicalPost", c12),
                ("SuppliesReady", c12),
            ],
            [],
            [("Rescued", patient)],
            [("At", patient, c12)],
        ),
    ]
    return plan


# ---------------------------------------------------------------------------
# Punto 2 – Forward Planning
# ---------------------------------------------------------------------------


def forwardBFS(problem: Problem) -> list[Action]:
    """
    Forward BFS in state space.

    Explore states reachable from the initial state by applying actions,
    in breadth-first order, until a goal state is found.

    Returns a list of Action objects forming a valid plan, or [] if no plan exists.

    Tip: The state is a frozenset of fluents. Use problem.getSuccessors(state)
         to get (next_state, action, cost) triples. Track visited states to
         avoid revisiting the same state twice (graph search, not tree search).
    """
    #Version refactorizada
    #Prompt empleado: refactoriza este codigo para que sea más compacto y limpio. usa unpacking donde sea posible, y 
    #reemplaza loops simples por expresiones equivalentes.Mantén exactamente la misma lógica y tipos de retorno.
    frontier = Queue()
    frontier.push((problem.getStartState(), []))
    visited = {problem.getStartState()}
    while not frontier.isEmpty():
        state, actions = frontier.pop()
        if problem.isGoalState(state):
            return actions
        for next_state, action, _ in problem.getSuccessors(state):
            if next_state not in visited:
                visited.add(next_state)
                frontier.push((next_state, actions + [action]))
    
    return []

    # Version inicial
    # start = problem.getStartState()
    # frontier = Queue()
    # frontier.push((start, []))
    # visited = set()
    # visited.add(start)
    # while not frontier.isEmpty():
    #     state, actions = frontier.pop()
        
    #     if problem.isGoalState(state):
    #         return actions
        
    #     for next_state, action, cost in problem.getSuccessors(state):
    #         if next_state not in visited:
    #             visited.add(next_state)
    #             frontier.push((next_state, actions + [action]))
    
    # return []


# ---------------------------------------------------------------------------
# Punto 3 – Backward Planning
# ---------------------------------------------------------------------------


def regress(goal_set: State, action: Action) -> State | None:
    """
    Compute the regression of goal_set through action.

    Given a goal description (set of fluents that must be true) and an action,
    return the new goal description that, if satisfied, guarantees the original
    goal is satisfied after executing action.

    REGRESS(g, a) = (g − ADD(a)) ∪ PRECOND_pos(a)
        IF:  ADD(a) ∩ g ≠ ∅   (action is relevant: contributes to the goal)
        AND: DEL(a) ∩ g = ∅   (action does not undo any goal fluent)
    Returns None if the action is not relevant or creates a contradiction.

    Tip: Use frozenset operations: intersection (&), difference (-), union (|).
         Check relevance first, then check for contradictions, then compute.
    """
    #Version iterada con ayuda de la ia
    if not action.add_list & goal_set:
        return None
    if action.del_list & goal_set:
        return None
    return (goal_set - action.add_list) | action.precond_pos

    #Version inicial
    # if len(action.add_list & goal_set) == 0:
    #     return None
    # if len(action.del_list & goal_set) != 0:
    #     return None
    # new_goal = (goal_set - action.add_list) | action.precond_pos
    # return new_goal


def backwardSearch(problem: Problem) -> list[Action]:
    """
    Backward search (regression search) from the goal.

    Start from the goal description and apply action regressions until
    the resulting goal is satisfied by the initial state.

    Returns a list of Action objects forming a valid plan (in forward order),
    or [] if no plan exists.

    Tip: The "state" in backward search is a frozenset of fluents that must
         be true (a partial goal description). The initial state is reached
         when all fluents in the current goal are satisfied by problem.initial_state.
         Only consider actions whose add_list has at least one unsatisfied goal fluent
         (relevant actions). Use regress() to compute the new subgoal.
         Skip subgoals that contain static predicates (MedicalPost, Adjacent,
         Pickable) that are false in the initial state — these are dead ends.
    """
    #Version iterada con ayuda de la ia
    #Prompt usado: ¿Puedes simplificarlo para que quede más limpio y corto sin cambiar lo que hace?
    frontier = Queue()
    frontier.push((problem.goal, []))
    visited = {problem.goal}
    
    while not frontier.isEmpty():
        goal, actions = frontier.pop()
        if goal.issubset(problem.initial_state):
            return actions
        for action in get_all_groundings(problem.domain, problem.objects):
            new_goal = regress(goal, action)
            if new_goal is not None and new_goal not in visited:
                visited.add(new_goal)
                frontier.push((new_goal, [action] + actions))
    
    return []

    #Version inicial
    # start_goal = problem.goal
    # frontier = Queue()
    # frontier.push((start_goal, []))
    # visited = set()
    # visited.add(start_goal)
    
    # while not frontier.isEmpty():
    #     goal, actions = frontier.pop()
        
    #     if goal.issubset(problem.initial_state):
    #         return actions
        
    #     for action in get_all_groundings(problem.domain, problem.objects):
    #         new_goal = regress(goal, action)
    #         if new_goal is None:
    #             continue
    #         if new_goal in visited:
    #             continue
    #         visited.add(new_goal)
    #         frontier.push((new_goal, [action] + actions))
    
    # return []


# ---------------------------------------------------------------------------
# Punto 4 – A* Planner
# ---------------------------------------------------------------------------

# Heuristic signature:  heuristic(state, goal, domain, objects) -> float
Heuristic = Callable[[State, State, list[ActionSchema], Objects], float]


def aStarPlanner(
    problem: Problem,
    heuristic: Heuristic = nullHeuristic,
) -> list[Action]:
    """
    Forward A* search guided by a heuristic.

    Combines the real accumulated cost g(n) with the heuristic estimate h(n)
    to prioritize which state to expand next: f(n) = g(n) + h(n).

    Returns a list of Action objects forming a valid plan, or [] if no plan exists.

    Tip: The heuristic signature is heuristic(state, goal, domain, objects) → float.
         Use PriorityQueue with priority = g + h(next_state).
         Track the best g-cost seen for each state to avoid stale expansions.
    """
    # Version refactorizada con ayuda de la IA
    # Prompt usado: "Refactoriza este A* para que use PriorityQueue de utils.py,
    # cachee la heurística por estado, y descarte entradas obsoletas comparando
    # g acumulada contra el mejor g visto. Mantén exactamente la misma lógica."
    start = problem.getStartState()
    h_start = heuristic(start, problem.goal, problem.domain, problem.objects)

    frontier = PriorityQueue()
    frontier.push((start, []), h_start)

    best_g: dict = {start: 0}
    h_cache: dict = {start: h_start}

    while not frontier.isEmpty():
        state, actions = frontier.pop()
        g = len(actions)

        # Entrada obsoleta: ya encontramos un camino más corto a este estado.
        if g > best_g.get(state, float("inf")):
            continue

        if problem.isGoalState(state):
            return actions

        for next_state, action, cost in problem.getSuccessors(state):
            new_g = g + cost
            if new_g < best_g.get(next_state, float("inf")):
                best_g[next_state] = new_g
                if next_state not in h_cache:
                    h_cache[next_state] = heuristic(
                        next_state, problem.goal, problem.domain, problem.objects
                    )
                f = new_g + h_cache[next_state]
                frontier.push((next_state, actions + [action]), f)

    return []

    # Version inicial
    # start = problem.getStartState()
    # frontier = PriorityQueue()
    # frontier.push((start, []), heuristic(start, problem.goal, problem.domain, problem.objects))
    # best_g = {}
    # best_g[start] = 0
    # while not frontier.isEmpty():
    #     state, actions = frontier.pop()
    #     g = len(actions)
    #     if g > best_g.get(state, 10**9):
    #         continue
    #     if problem.isGoalState(state):
    #         return actions
    #     for next_state, action, cost in problem.getSuccessors(state):
    #         new_g = g + cost
    #         if new_g < best_g.get(next_state, 10**9):
    #             best_g[next_state] = new_g
    #             h = heuristic(next_state, problem.goal, problem.domain, problem.objects)
    #             frontier.push((next_state, actions + [action]), new_g + h)
    # return []


# Aliases used by the command-line argument parser
tinyBaseSearch = tinyBaseSearch
forwardBFS = forwardBFS
backwardSearch = backwardSearch
aStarPlanner = aStarPlanner
