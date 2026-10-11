"""Deterministic, geometry-based right-of-way for two warehouse robots."""

from collections import deque
import math


def shortest_path(start, goal, blocked, cols, rows):
    if goal in blocked or not (0 <= goal[0] < cols and 0 <= goal[1] < rows):
        return []
    queue = deque([start])
    previous = {start: None}
    while queue:
        cell = queue.popleft()
        if cell == goal:
            path = []
            while cell is not None:
                path.append(cell)
                cell = previous[cell]
            return path[::-1]
        c, r = cell
        for neighbor in ((c + 1, r), (c - 1, r), (c, r + 1), (c, r - 1)):
            if (0 <= neighbor[0] < cols and 0 <= neighbor[1] < rows
                    and neighbor not in blocked and neighbor not in previous):
                previous[neighbor] = cell
                queue.append(neighbor)
    return []


def refuge_bid(start, goal, peer, peer_goal, blocked, cols, rows):
    """Return (extra grid steps if yielding, refuge, route to refuge).

    A refuge is at least two cells from the peer and outside the peer's
    immediate route. The first move must increase separation; this prevents
    both robots from driving into one another to reach a nominal refuge.
    """
    peer_route = shortest_path(peer, peer_goal, blocked, cols, rows)
    # Ustepienie o jedna komorke w tyl nie pomaga, jesli zwyciezca ma
    # przejechac ta sama aleja: wymuszaloby kolejne licytacje i cofanie.
    peer_corridor = set(peer_route) | {peer, peer_goal}
    direct = shortest_path(start, goal, blocked, cols, rows)
    direct_steps = len(direct) - 1 if direct else abs(start[0]-goal[0]) + abs(start[1]-goal[1])
    peer_distance = abs(start[0]-peer[0]) + abs(start[1]-peer[1])
    options = []
    for c in range(cols):
        for r in range(rows):
            refuge = (c, r)
            if refuge == start or refuge in blocked or refuge in peer_corridor:
                continue
            if abs(c-peer[0]) + abs(r-peer[1]) < 2:
                continue
            route = shortest_path(start, refuge, blocked | {peer}, cols, rows)
            if not route or len(route) > 6:
                continue
            if len(route) > 1 and (abs(route[1][0]-peer[0]) + abs(route[1][1]-peer[1])
                                   <= peer_distance):
                continue
            # Po przejezdzie zwyciezcy jego obecna komorka bedzie wolna.
            # Traktowanie jej jako stalej blokady zawyzalo koszt krotkiej
            # mijanki i wybieralo wielokomorkowe cofanie w tej samej alei.
            onward = shortest_path(refuge, goal, blocked, cols, rows)
            if not onward:
                continue
            detour = len(route) + len(onward) - 2 - direct_steps
            options.append((max(0, detour), len(route), refuge, route))
    if not options:
        return math.inf, None, []
    cost, _, refuge, route = min(options)
    return cost, refuge, route


def winner(our_id, our_bid, other_id, other_bid):
    """The robot for which yielding costs more keeps right-of-way."""
    if our_bid != other_bid:
        return our_id if our_bid > other_bid else other_id
    return min(our_id, other_id)
