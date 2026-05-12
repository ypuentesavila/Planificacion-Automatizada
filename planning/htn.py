from __future__ import annotations

from itertools import permutations

from planning.pddl import Action, Problem, apply_action, is_applicable
from planning.utils import Queue


# ---------------------------------------------------------------------------
# HTN Infrastructure
# ---------------------------------------------------------------------------


class HLA:
    """
    A High-Level Action (HLA) in HTN planning.

    An HLA is an abstract task that can be refined into sequences of
    more primitive actions (or other HLAs). Each refinement is a list
    of HLA or Action objects.

    name:        Human-readable name for display
    refinements: List of possible refinements, each a list of HLA/Action objects
    """

    def __init__(self, name: str, refinements: list[list] | None = None) -> None:
        self.name = name
        self.refinements = refinements or []

    def __repr__(self) -> str:
        return f"HLA({self.name})"


def is_primitive(action: Action | HLA) -> bool:
    """Return True if action is a primitive (grounded Action), False if it is an HLA."""
    return isinstance(action, Action)


def is_plan_primitive(plan: list[Action | HLA]) -> bool:
    """Return True if every step in the plan is a primitive action."""
    return all(is_primitive(step) for step in plan)


# ---------------------------------------------------------------------------
# Punto 5a – hierarchicalSearch
# ---------------------------------------------------------------------------


def hierarchicalSearch(problem: Problem, hlas: list[HLA]) -> list[Action]:
    """
    HTN planning via BFS over hierarchical plan refinements.

    Start with an initial plan containing a single top-level HLA.
    At each step, find the first non-primitive step in the plan and
    replace it with one of its refinements. Continue until the plan
    is fully primitive and achieves the goal when executed from the
    initial state.

    Returns a list of primitive Action objects, or [] if no plan found.
    """
    # Version refactorizada con ayuda de la IA
    # Prompt usado: "Implementa BFS sobre planes jerárquicos: cada nodo es una
    # lista mixta de HLA/Action. Expande la primera HLA encontrada generando un
    # plan por cada refinamiento. Detén cuando el plan sea completamente
    # primitivo y la simulación desde initial_state alcance el goal."
    if not hlas:
        return []

    frontier = Queue()
    frontier.push(list(hlas))

    while not frontier.isEmpty():
        plan = frontier.pop()
        problem._expanded += 1

        if is_plan_primitive(plan):
            state = problem.initial_state
            valid = True
            for action in plan:
                if not is_applicable(state, action):
                    valid = False
                    break
                state = apply_action(state, action)
            if valid and problem.isGoalState(state):
                return plan
            continue

        # Encontrar la primera HLA y generar un nuevo plan por cada refinamiento
        for i, step in enumerate(plan):
            if not is_primitive(step):
                for refinement in step.refinements:
                    new_plan = plan[:i] + list(refinement) + plan[i + 1 :]
                    frontier.push(new_plan)
                break

    return []

    # Version inicial
    # if len(hlas) == 0:
    #     return []
    # frontier = Queue()
    # frontier.push(list(hlas))
    # while not frontier.isEmpty():
    #     plan = frontier.pop()
    #     primitive = True
    #     for step in plan:
    #         if not is_primitive(step):
    #             primitive = False
    #             break
    #     if primitive:
    #         state = problem.initial_state
    #         ok = True
    #         for action in plan:
    #             if not is_applicable(state, action):
    #                 ok = False
    #                 break
    #             state = apply_action(state, action)
    #         if ok and problem.isGoalState(state):
    #             return plan
    #         continue
    #     idx = -1
    #     for i in range(len(plan)):
    #         if not is_primitive(plan[i]):
    #             idx = i
    #             break
    #     hla = plan[idx]
    #     for refinement in hla.refinements:
    #         new_plan = plan[:idx] + list(refinement) + plan[idx+1:]
    #         frontier.push(new_plan)
    # return []


# ---------------------------------------------------------------------------
# Punto 5a / 5b – HLA Definitions
# ---------------------------------------------------------------------------

# Cap de permutaciones de pacientes en MultiRescue (N! crece muy rápido).
# Para N <= MAX_PERMUTATIONS_N usamos todas las permutaciones; para N mayor
# usamos un único orden (por id de paciente). En este dominio el orden no
# afecta la longitud del plan cuando hay un solo puesto médico, pero
# habilitamos varias permutaciones por completitud del enfoque HTN.
_MAX_PERMUTATIONS_N = 4


def build_htn_hierarchy(problem: Problem) -> list[HLA]:
    """
    Build HTN HLAs for the rescue domain.

    The hierarchy defines four HLA types:
      - Navigate(from, to):       Move the robot step by step from one cell to another
      - PrepareSupplies(s, m):    Collect supplies and set them up at the medical post
      - ExtractPatient(p, m):     Pick up the patient and bring them to the medical post
      - FullRescueMission(s,p,m): Complete one rescue: prepare supplies + extract + rescue

    Refinements are built from the ground state to generate concrete Action objects.
    """
    # Version refactorizada con ayuda de la IA
    # Prompt usado: "Construye una jerarquía HTN para el dominio de rescate.
    # Cada HLA debe tener refinamientos concretos (lista de HLA/Action). Para
    # Navigate, calcula el camino BFS más corto entre celdas usando la
    # adyacencia derivada del estado inicial. Para PrepareSupplies y
    # ExtractPatient, construye secuencias que sigan la lógica de tinyBaseSearch
    # (PickUp/SetupSupplies/PutDown/Rescue grounded). Para múltiples pacientes
    # genera refinamientos con permutaciones del orden (cap a N <= 4)."
    initial_state = problem.initial_state
    objects = problem.objects

    patients = objects["patients"]
    supplies = objects["supplies"]
    medical_posts = objects["medical_posts"]

    if not patients or not medical_posts or not supplies:
        return []

    # Posición inicial del robot
    robot_start = next(
        f[2] for f in initial_state if f[0] == "At" and f[1] == "robot"
    )

    # Posición de cada paciente y suministro
    obj_position: dict[str, tuple] = {}
    for f in initial_state:
        if f[0] == "At" and f[1] != "robot":
            obj_position[f[1]] = f[2]

    # Grafo de adyacencia (no dirigido) derivado del estado inicial
    adjacency: dict[tuple, list[tuple]] = {}
    for f in initial_state:
        if f[0] == "Adjacent":
            adjacency.setdefault(f[1], []).append(f[2])

    def bfs_path(start: tuple, end: tuple) -> list[tuple] | None:
        if start == end:
            return [start]
        visited = {start}
        parent: dict[tuple, tuple] = {}
        frontier = [start]
        while frontier:
            current = frontier.pop(0)
            for neighbor in adjacency.get(current, []):
                if neighbor in visited:
                    continue
                visited.add(neighbor)
                parent[neighbor] = current
                if neighbor == end:
                    path = [end]
                    while path[-1] != start:
                        path.append(parent[path[-1]])
                    return list(reversed(path))
                frontier.append(neighbor)
        return None

    # ---------------- Factories de acciones primitivas grounded ----------------
    # Replican el formato de tinyBaseSearch (planner.py) y de los ActionSchema
    # de domain.py — necesario porque hierarchicalSearch valida primitividad
    # con isinstance(Action) y simula con is_applicable / apply_action.

    def make_move(a: tuple, b: tuple) -> Action:
        return Action(
            f"Move(robot,{a},{b})",
            [("At", "robot", a), ("Adjacent", a, b), ("Free", b)],
            [],
            [("At", "robot", b), ("Free", a)],
            [("At", "robot", a), ("Free", b)],
        )

    def make_pickup(obj: str, loc: tuple) -> Action:
        return Action(
            f"PickUp(robot,{obj},{loc})",
            [
                ("At", "robot", loc),
                ("At", obj, loc),
                ("HandsFree", "robot"),
                ("Pickable", obj),
            ],
            [],
            [("Holding", "robot", obj)],
            [("At", obj, loc), ("HandsFree", "robot")],
        )

    def make_putdown(obj: str, loc: tuple) -> Action:
        return Action(
            f"PutDown(robot,{obj},{loc})",
            [("At", "robot", loc), ("Holding", "robot", obj)],
            [],
            [("At", obj, loc), ("HandsFree", "robot")],
            [("Holding", "robot", obj)],
        )

    def make_setup(s: str, loc: tuple) -> Action:
        return Action(
            f"SetupSupplies(robot,{s},{loc})",
            [("At", "robot", loc), ("MedicalPost", loc), ("Holding", "robot", s)],
            [],
            [("SuppliesReady", loc), ("At", s, loc), ("HandsFree", "robot")],
            [("Holding", "robot", s)],
        )

    def make_rescue(p: str, loc: tuple) -> Action:
        return Action(
            f"Rescue(robot,{p},{loc})",
            [
                ("At", "robot", loc),
                ("At", p, loc),
                ("MedicalPost", loc),
                ("SuppliesReady", loc),
            ],
            [],
            [("Rescued", p)],
            [("At", p, loc)],
        )

    # ---------------- Factories de HLAs ----------------

    def make_navigate(from_cell: tuple, to_cell: tuple) -> HLA:
        """HLA Navigate con un único refinamiento: la secuencia de Move
        primitivos del camino BFS más corto entre from_cell y to_cell."""
        if from_cell == to_cell:
            return HLA(f"Navigate({from_cell}->{to_cell})", [[]])
        path = bfs_path(from_cell, to_cell)
        if path is None:
            return HLA(f"Navigate({from_cell}->{to_cell})", [])
        moves = [make_move(path[i], path[i + 1]) for i in range(len(path) - 1)]
        return HLA(f"Navigate({from_cell}->{to_cell})", [moves])

    def make_prepare_supplies(start: tuple, s: str, m: tuple) -> HLA:
        s_loc = obj_position[s]
        refinement = [
            make_navigate(start, s_loc),
            make_pickup(s, s_loc),
            make_navigate(s_loc, m),
            make_setup(s, m),
        ]
        return HLA(f"PrepareSupplies({s}@{m})", [refinement])

    def make_extract_patient(start: tuple, p: str, m: tuple) -> HLA:
        p_loc = obj_position[p]
        refinement = [
            make_navigate(start, p_loc),
            make_pickup(p, p_loc),
            make_navigate(p_loc, m),
            make_putdown(p, m),
            make_rescue(p, m),
        ]
        return HLA(f"ExtractPatient({p}@{m})", [refinement])

    # ---------------- Construcción del HLA raíz ----------------

    m = medical_posts[0]
    s0 = supplies[0]

    if len(patients) == 1:
        p0 = patients[0]
        prepare = make_prepare_supplies(robot_start, s0, m)
        extract = make_extract_patient(m, p0, m)
        full = HLA(
            f"FullRescueMission({s0},{p0},{m})", [[prepare, extract]]
        )
        return [full]

    # Múltiples pacientes: una sola preparación de suministros y un
    # ExtractPatient por paciente. Permutamos el orden para que el HTN
    # tenga refinamientos alternativos (5b: el orden puede impactar el plan).
    prepare = make_prepare_supplies(robot_start, s0, m)

    if len(patients) <= _MAX_PERMUTATIONS_N:
        orderings = list(permutations(patients))
    else:
        orderings = [tuple(patients)]

    refinements = []
    for ordering in orderings:
        sequence = [prepare]
        for p in ordering:
            sequence.append(make_extract_patient(m, p, m))
        refinements.append(sequence)

    multi = HLA("MultiRescueMission", refinements)
    return [multi]

    # Version inicial (sin permutaciones, sin caché de posiciones)
    # initial_state = problem.initial_state
    # objects = problem.objects
    # patients = objects["patients"]
    # supplies = objects["supplies"]
    # medical_posts = objects["medical_posts"]
    # if len(patients) == 0:
    #     return []
    # if len(medical_posts) == 0:
    #     return []
    # robot_start = None
    # for f in initial_state:
    #     if f[0] == "At" and f[1] == "robot":
    #         robot_start = f[2]
    #         break
    # ... (construcción manual de cada HLA sin helpers)
    # return [HLA("...")]
