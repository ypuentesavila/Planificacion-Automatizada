from __future__ import annotations

from planning.pddl import ActionSchema, State, Objects, get_all_groundings


def nullHeuristic(
    state: State,
    goal: State,
    domain: list[ActionSchema],
    objects: Objects,
) -> float:
    """Trivial heuristic — always returns 0 (equivalent to uniform-cost search)."""
    return 0


# ---------------------------------------------------------------------------
# Cache compartido de groundings para evitar regenerarlos en cada llamada
# (A* llama h() una vez por estado expandido; los groundings dependen solo de
# (domain, objects) que son constantes para un mismo problema).
# ---------------------------------------------------------------------------
_GROUNDINGS_CACHE: dict[int, list] = {}


def _get_groundings(domain: list[ActionSchema], objects: Objects) -> list:
    key = id(domain) ^ id(objects)
    cached = _GROUNDINGS_CACHE.get(key)
    if cached is None:
        cached = get_all_groundings(domain, objects)
        _GROUNDINGS_CACHE[key] = cached
    return cached


# ---------------------------------------------------------------------------
# Punto 4a – Ignore-Preconditions Heuristic
# ---------------------------------------------------------------------------


def ignorePreconditionsHeuristic(
    state: State,
    goal: State,
    domain: list[ActionSchema],
    objects: Objects,
) -> float:
    """
    Estimate the number of actions needed to satisfy all goal fluents,
    ignoring all action preconditions.

    With no preconditions, any action can be applied at any time.
    Each action can satisfy all goal fluents in its add_list in one step.
    The minimum number of actions to cover all unsatisfied goal fluents is
    a lower bound on the true plan length → this heuristic is admissible.

    Algorithm (greedy set cover):
      1. Compute unsatisfied = goal − state  (fluents still needed).
      2. Ground all actions ignoring preconditions and collect their add_lists.
      3. Greedily pick the action whose add_list covers the most unsatisfied fluents.
      4. Repeat until all fluents are covered; count the actions used.
    """
    # Version final (con caché de groundings y short-circuit cuando ya se cumple el goal)
    # Prompt usado: "Refactoriza este algoritmo de greedy set cover para que use
    # frozenset operations nativas (&, -) y un cache de groundings reutilizable
    # entre llamadas; mantén exactamente el mismo comportamiento."
    unsatisfied = goal - state
    if not unsatisfied:
        return 0

    groundings = _get_groundings(domain, objects)
    count = 0
    while unsatisfied:
        best_cover = frozenset()
        for action in groundings:
            cover = action.add_list & unsatisfied
            if len(cover) > len(best_cover):
                best_cover = cover
        if not best_cover:
            return float("inf")
        unsatisfied = unsatisfied - best_cover
        count += 1
    return count

    # Version inicial
    # unsatisfied = set(goal) - set(state)
    # if len(unsatisfied) == 0:
    #     return 0
    # groundings = get_all_groundings(domain, objects)
    # count = 0
    # while len(unsatisfied) > 0:
    #     best_action = None
    #     best_cover_size = 0
    #     best_cover = set()
    #     for action in groundings:
    #         cover = set()
    #         for f in action.add_list:
    #             if f in unsatisfied:
    #                 cover.add(f)
    #         if len(cover) > best_cover_size:
    #             best_cover_size = len(cover)
    #             best_cover = cover
    #             best_action = action
    #     if best_action is None:
    #         return float("inf")
    #     for f in best_cover:
    #         unsatisfied.remove(f)
    #     count = count + 1
    # return count


# ---------------------------------------------------------------------------
# Punto 4b – Ignore-Delete-Lists Heuristic
# ---------------------------------------------------------------------------


def ignoreDeleteListsHeuristic(
    state: State,
    goal: State,
    domain: list[ActionSchema],
    objects: Objects,
) -> float:
    """
    Estimate the plan cost by solving a relaxed problem where no action
    has a delete list (effects never remove fluents from the state).

    In this monotone relaxation, the state only grows over time (fluents are
    never removed), so hill-climbing always makes progress and cannot loop.

    Algorithm (hill-climbing on the relaxed problem):
      1. Start from the current state with a relaxed (monotone) apply function.
      2. At each step, pick the grounded action that adds the most unsatisfied
         goal fluents (greedy hill-climbing).
      3. Count steps until all goal fluents are satisfied (or until no progress).
    """
    # Version final (hill-climbing con desempate por progreso general:
    # cuando ninguna acción aplicable aporta fluentes del goal en este paso,
    # se permite progresar por precondiciones priorizando la acción que más
    # fluentes nuevos genere — sigue siendo el "hill-climbing literal" del
    # docstring, simplemente con un tie-break que evita bloqueos prematuros).
    # Prompt usado: "Implementa hill-climbing sobre el problema relajado
    # ignorando del_list. En cada paso elige la acción aplicable que más
    # fluentes nuevos del goal añada; si ninguna aporta al goal, escoge la
    # que más fluentes nuevos genere (para habilitar futuras acciones).
    # No reutilices la misma acción dos veces."
    relaxed = set(state)
    goal_set = set(goal)
    unsatisfied = goal_set - relaxed
    if not unsatisfied:
        return 0

    groundings = _get_groundings(domain, objects)
    applied: set = set()
    count = 0

    while unsatisfied:
        best_action = None
        best_goal_gain = -1
        best_total_gain = -1

        for action in groundings:
            if action.name in applied:
                continue
            if not action.precond_pos.issubset(relaxed):
                continue
            if not action.precond_neg.isdisjoint(relaxed):
                continue
            new_fluents = action.add_list - relaxed
            if not new_fluents:
                continue
            goal_gain = len(new_fluents & unsatisfied)
            total_gain = len(new_fluents)
            if goal_gain > best_goal_gain or (
                goal_gain == best_goal_gain and total_gain > best_total_gain
            ):
                best_goal_gain = goal_gain
                best_total_gain = total_gain
                best_action = action

        if best_action is None:
            return float("inf")

        applied.add(best_action.name)
        relaxed |= best_action.add_list
        unsatisfied = goal_set - relaxed
        count += 1

    return count

    # Version inicial
    # relaxed = set(state)
    # unsatisfied = set(goal) - relaxed
    # if len(unsatisfied) == 0:
    #     return 0
    # groundings = get_all_groundings(domain, objects)
    # count = 0
    # applied = set()
    # while len(unsatisfied) > 0:
    #     best_action = None
    #     best_score = -1
    #     for action in groundings:
    #         if action.name in applied:
    #             continue
    #         ok = True
    #         for f in action.precond_pos:
    #             if f not in relaxed:
    #                 ok = False
    #                 break
    #         if not ok:
    #             continue
    #         new_f = []
    #         for f in action.add_list:
    #             if f not in relaxed:
    #                 new_f.append(f)
    #         if len(new_f) == 0:
    #             continue
    #         score = 0
    #         for f in new_f:
    #             if f in unsatisfied:
    #                 score = score + 1
    #         if score > best_score:
    #             best_score = score
    #             best_action = action
    #     if best_action is None:
    #         return float("inf")
    #     applied.add(best_action.name)
    #     for f in best_action.add_list:
    #         relaxed.add(f)
    #     unsatisfied = set(goal) - relaxed
    #     count = count + 1
    # return count
