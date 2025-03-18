from blockworld import BlockWorld
import heapq
import numpy as np
# Před importem blockworld "patchneme" alias np.int, aby odkazoval na np.int64.
np.int = np.int64

def canonical_state(state):
    """
    Převádí stav do kanonické, hashovatelné reprezentace.
    """
    return tuple(tuple(tower) for tower in state)

class BlockWorldHeuristic(BlockWorld):
    def __init__(self, num_blocks=5, state=None):
        super().__init__(num_blocks, state)

    def heuristic(self, goal):
        """
        Heuristika, která porovnává každou věž v aktuálním stavu se
        všemi věžemi v cíli a hledá maximální počet shodných spodních bloků.
        Následně spočte, kolik bloků zbývá „opravit“.
        """
        # Pomocná funkce pro zjištění, kolik bloků odspodu se shoduje.
        def matched_bottom_length(towerA, towerB):
            """
            Porovná věž towerA a towerB odspodu. Jakmile narazí na rozdíl,
            skončí a vrátí počet shodných bloků.
            """
            i = 0
            while i < len(towerA) and i < len(towerB):
                # Porovnáváme i-tý blok odspodu (index od konce)
                if towerA[-1 - i] != towerB[-1 - i]:
                    break
                i += 1
            return i

        # 1) Spočítám celkový počet bloků
        total_blocks = sum(len(t) for t in self.state)

        # 2) Pro každou věž v current_state hledám "nejlepší" shodu
        #    s libovolnou věží v goal_state
        sum_of_best_matches = 0
        for towerA in self.state:
            best_match_for_this_tower = 0
            for towerB in goal.state:
                # kolik spodních bloků se přesně shoduje?
                match_len = matched_bottom_length(towerA, towerB)
                if match_len > best_match_for_this_tower:
                    best_match_for_this_tower = match_len

            sum_of_best_matches += best_match_for_this_tower

        # 3) Heuristika = kolik bloků je ještě "mimo", tj. total - už dobře položené
        misplaced = total_blocks - sum_of_best_matches
        return misplaced


class AStar():
    def search(self, start, goal):
        """
        Prohledává stavový prostor pomocí A* algoritmu.
        start, goal: instance BlockWorldHeuristic (nebo BlockWorld), kde goal má definovaný svůj stav.
        """
        import heapq
        from collections import defaultdict

        def canonical_state(conf):
            # Pomocná funkce na vytvoření hashovatelné reprezentace stavu
            # (pokud už nějakou máš, používej ji)
            return tuple(tuple(t) for t in conf)

        open_set = []
        start_state_key = canonical_state(start.get_state())

        # g_score: vzdálenost od startu
        g_score = defaultdict(lambda: float('inf'))
        g_score[start_state_key] = 0

        # f_score = g_score + heuristika
        start_f = start.heuristic(goal)
        heapq.heappush(open_set, (start_f, start, []))  # (priorita, stav, cesta_akcí)

        closed_set = set()

        while open_set:
            f_current, current, path_so_far = heapq.heappop(open_set)
            current_key = canonical_state(current.get_state())

            if current_key in closed_set:
                continue
            closed_set.add(current_key)

            # Ověříme, zda jsme v cíli
            if current.get_state() == goal.get_state():
                return path_so_far

            # Projdeme všechny sousedy
            for action, neighbor in current.get_neighbors():
                neighbor_key = canonical_state(neighbor.get_state())
                tentative_g = g_score[current_key] + 1

                if tentative_g < g_score[neighbor_key]:
                    g_score[neighbor_key] = tentative_g
                    f_neighbor = tentative_g + neighbor.heuristic(goal)
                    heapq.heappush(open_set, (f_neighbor, neighbor, path_so_far + [action]))

        # Pokud se open_set vyprázdní a my nenašli cíl, vrátíme None.
        return None

    
if __name__ == '__main__':
	# Here you can test your algorithm. You can try different N values, e.g. 6, 7.
	N = 5

	start = BlockWorldHeuristic(N)
	goal = BlockWorldHeuristic(N)

	# start = BlockWorldHeuristic(N, state='[[2], [1], [4], [3, 5]]')
	# goal  = BlockWorldHeuristic(N, state='[[5, 3, 1], [2], [4]]')

	print("Searching for a path:")
	# Použití v hlavním kódu
	print(convert_npint64_string(str(start)))
	print(convert_npint64_string(str(goal)))

	#print(f"{start} -> {goal}")
	print()

	astar = AStar()
	path = astar.search(start, goal)

	if path is not None:
		print("Found a path:")
		print(path)

		print("\nHere's how it goes:")

		s = start.clone()
		print(s)

		for a in path:
			s.apply(a)
			print(s)

	else:
		print("No path exists.")

	print("Total expanded nodes:", BlockWorld.expanded)