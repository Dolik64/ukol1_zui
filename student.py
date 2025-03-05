from blockworld import BlockWorld
import heapq
import numpy as np
# Před importem blockworld "patchneme" alias np.int, aby odkazoval na np.int64.
np.int = np.int64

def convert_npint64_string(s):
    """
    Converts a string containing np.int64() values into a string with normal integers.
    """
    return s.replace("np.int64(", "").replace(")", "")

def convert_npint64_string(s):
    """
    Converts a string containing np.int64() values into a string with normal integers.
    """
    return s.replace("np.int64(", "").replace(")", "")


class BlockWorldHeuristic(BlockWorld):
	def __init__(self, num_blocks=5, state=None):
		BlockWorld.__init__(self, num_blocks, state)

	def heuristic(self, goal):
		self_state = self.get_state()
		goal_state = goal.get_state()

		# ToDo. Implement the heuristic here.

		return 0.

class AStar():
    def search(self, start, goal):
        """
        Prohledává stavový prostor pomocí A* algoritmu.
        Vrací seznam akcí, které transformují start do goal.
        Pokud cesta neexistuje, vrací None.
        """
        # Priority queue: každý prvek je tuple (f, current_state, path, g)
        # f = g + h, kde g je náklad od startu a h je odhad (heuristika)
        open_set = []
        start_f = 0 + start.heuristic(goal)
        heapq.heappush(open_set, (start_f, start, [], 0))
        
        # Uzavřená množina pro zamezení opětovného zpracování stejných stavů.
        closed_set = set()
        
        while open_set:
            f, current, path, g = heapq.heappop(open_set)
            
            # Použijeme kanonickou reprezentaci stavu pro kontrolu, zda jsme ho již zpracovali.
            state_key = str(current.get_state())
            if state_key in closed_set:
                continue
            closed_set.add(state_key)
            
            # Pokud jsme dosáhli cílového stavu, vrátíme cestu (seznam akcí)
            if current.get_state() == goal.get_state():
                return path
            
            # Prozkoumej všechny možné tahy z aktuálního stavu.
            for action, neighbor in current.get_neighbors():
                # Předpokládáme, že každý tah má náklad 1.
                new_g = g + 1
                new_f = new_g + neighbor.heuristic(goal)
                new_path = path + [action]
                neighbor_key = str(neighbor.get_state())
                if neighbor_key in closed_set:
                    continue
                heapq.heappush(open_set, (new_f, neighbor, new_path, new_g))
        
        # Pokud cesta nebyla nalezena, vrátíme None.
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