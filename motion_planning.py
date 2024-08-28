import heapq

class Move:                                             
    def __init__(self, state, depth, heuristic, path):
        self.state = state
        self.depth = depth
        self.heuristic = heuristic
        self.totalCost = heuristic+depth
        self.path=path
        
    def __lt__(self, other):                            
        if(self.totalCost < other.totalCost):
            return True
        elif (self.totalCost == other.totalCost):
            if(self.depth < other.depth):
                return True
            else:
                return self.heuristic < other.heuristic 
        else:
            return False
            
def neighbors(current):
    # define the list of 4 neighbors
    neighbors = [(0,1),(0,-1),(1,0),(-1,0)]
    return [ (current[0]+nbr[0], current[1]+nbr[1]) for nbr in neighbors ]

def heuristic_distance(candidate, goal):
    return abs(candidate[0]-goal[0]) + abs(candidate[1]-goal[1])

def expand(state, obstacles):
    nextFrontier=[]
    nghs=neighbors(state)
    for ngh in nghs:
        if ngh not in obstacles:
            nextFrontier.append(ngh)
    return nextFrontier
        
def get_path_from_A_star(start, goal, obstacles):
    # input  start: integer 2-tuple of the current grid, e.g., (0, 0)
    #        goal: integer 2-tuple  of the goal grid, e.g., (5, 1)
    #        obstacles: a list of grids marked as obstacles, e.g., [(2, -1), (2, 0), ...]
    # output path: a list of grids connecting start to goal, e.g., [(1, 0), (1, 1), ...]
    #   note that the path should contain the goal but not the start
    #   e.g., the path from (0, 0) to (2, 2) should be [(1, 0), (1, 1), (2, 1), (2, 2)] 
    queue=[]
    heapq.heapify(queue)
    path=[]
    heapq.heappush(queue,Move(start,0,heuristic_distance(start,goal), path))
    repeated=set()
    while(len(queue)>0):
        node=heapq.heappop(queue)
        if(str(node.state) in repeated):
            continue
        repeated.add(str(node.state))
        if node.state==goal:
            return node.path
        nextFrontier = expand(node.state, obstacles)  
        
        for state in nextFrontier:
            if (str(state) in repeated):
                continue
            newPath=node.path+[state]
            heapq.heappush(queue,Move(state, node.depth + 1,  heuristic_distance(state, goal),newPath)) 
    return -1


if __name__ == '__main__':
    start = (0, 0) # this is a tuple data structure in Python initialized with 2 integers
    goal = (-5, -2)
    obstacles = [(-2, 1), (-2, 0), (-2, -1), (-2, -2), (-4, -2), (-4, -3)]
    path = get_path_from_A_star(start, goal, obstacles)
    print(path)