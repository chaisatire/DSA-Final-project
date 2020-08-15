import math

def shortest_path(M, start, goal):
    """
    This function aims to find the shortest distance between start and goad using A* algorithm.
    For this function, I took refernce from from leetcode's explanation to A* search:
    https://leetcode.com/problems/shortest-path-in-binary-matrix/discuss/313347/a-search-in-python
    """
    priorityQ = PriorityQueue()
    priorityQ.add(start)

    # Keep track of distances travelled for the intersections
    node_dist = float()
    path_dist = float()

    # keep track of visited nodes and previous cities in the path
    visited = set()
    prev = dict()

    # Initializing the city distance from the start
    distance = {key: math.inf for key in M.intersections.keys()}
    distance[start] = 0

    # Checking all nodes in the queue
    while priorityQ:
        current = priorityQ.pop()
        if current in visited:
            continue

        # Terminate condition when goad find the current node.
        if goal == current:
            path = [current]
            while start != current:
                current = prev[current]
                path.append(current)
            path.reverse()
            return list(path)

        visited.add(current)

        # Going through all the cities to find the estimated distance from start to goal via adjoining city
        for city in M.roads[current]:
            node_dist = distance[current] + calculate_distance(M, current, city)
            path_dist = node_dist + calculate_distance(M, city, goal)
            if distance[city] > node_dist:
                distance[city] = node_dist
                prev[city] = current
                priorityQ.add(city, priority = path_dist)

    # Nothing Found so returning None
    return


## Helpers

def calculate_distance(M, start, goal):
    """ Helper function:
    We are getting the cartesian co-ordinates first and then finding the Euclidean distance.
    """
    coord_1 = M.intersections[start]
    coord_2 = M.intersections[goal]
    return math.sqrt((coord_1[0] - coord_2[0]) ** 2 + (coord_1[1] - coord_2[1]) ** 2)


from heapq import heappush, heappop


class PriorityQueue:
    """
    This is a support class which creates a basic priority Queue to be used. Reference to this code:
    https://leetcode.com/problems/shortest-path-in-binary-matrix/discuss/313347/a-search-in-python
    """

    def __init__(self, iterable=[]):
        self.heap = []
        for value in iterable:
            heappush(self.heap, (0, value))

    def add(self, value, priority=0):
        heappush(self.heap, (priority, value))

    def pop(self):
        priority, value = heappop(self.heap)
        return value

    def __len__(self):
        return len(self.heap)
