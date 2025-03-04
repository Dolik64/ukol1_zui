from blockworld import BlockWorld



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
		# ToDo. Return a list of optimal actions that takes start to goal.
		
		# You can access all actions and neighbors like this:
		# for action, neighbor in state.get_neighbors():
		# 	...

		return None

if __name__ == '__main__':
	# Here you can test your algorithm. You can try different N values, e.g. 6, 7.
	N = 5

	# start = BlockWorldHeuristic(N)
	# goal = BlockWorldHeuristic(N)

	start = BlockWorldHeuristic(N, state='[[2], [1], [4], [3, 5]]')
	goal  = BlockWorldHeuristic(N, state='[[5, 3, 1], [2], [4]]')

	print("Searching for a path:")
	print(f"{start} -> {goal}")
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