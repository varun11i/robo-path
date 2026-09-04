# import heapq so we can efficiently select the node with the lowest cost
import heapq

# import time so we can measure algorithm execution time
import time


class AStarPlanner:

    # initialize the planner with the environment it will navigate
    def __init__(self, environment):
        self.environment = environment

    # calculate the estimated distance from the current position to the goal
    def heuristic(self, current, goal):

        # use manhattan distance because the robot moves
        # only up, down, left, and right
        return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

    # return all valid neighboring cells around the current position
    def get_neighbors(self, position):

        # unpack the row and column
        row, col = position

        # define the four possible movement directions
        directions = [
            (-1, 0),  # move up
            (1, 0),   # move down
            (0, -1),  # move left
            (0, 1)    # move right
        ]

        # create an empty list for valid neighbors
        neighbors = []

        # check every possible movement direction
        for row_change, col_change in directions:

            # calculate the neighboring position
            new_row = row + row_change
            new_col = col + col_change

            # only include cells that are valid and not obstacles
            if self.environment.is_free(new_row, new_col):
                neighbors.append((new_row, new_col))

        # return all valid neighboring positions
        return neighbors

    # reconstruct the final path from the goal back to the start
    def reconstruct_path(self, came_from, current):

        # begin with the goal position
        path = [current]

        # continue moving backward until the start is reached
        while current in came_from:

            # retrieve the previous position
            current = came_from[current]

            # add it to the path
            path.append(current)

        # reverse the path so it runs from start to goal
        path.reverse()

        # return the completed path
        return path

    # run the a* path-planning algorithm
    def find_path(self, start, goal):

        # record the time when the search begins
        start_time = time.perf_counter()

        # create the priority queue
        open_set = []

        # add the starting position
        # its priority is its estimated distance to the goal
        heapq.heappush(
            open_set,
            (self.heuristic(start, goal), start)
        )

        # store how each node was reached
        came_from = {}

        # store the cost from the start to each node
        g_score = {start: 0}

        # store nodes that have already been fully explored
        closed_set = set()

        # count how many nodes the algorithm explores
        nodes_explored = 0

        # continue while there are nodes available to explore
        while open_set:

            # retrieve the node with the smallest f-score
            current_priority, current = heapq.heappop(open_set)

            # skip this node if it was already explored
            if current in closed_set:
                continue

            # mark the current node as explored
            closed_set.add(current)

            # increase the explored-node counter
            nodes_explored += 1

            # check whether the goal has been reached
            if current == goal:

                # reconstruct the final path
                path = self.reconstruct_path(came_from, current)

                # record the ending time
                end_time = time.perf_counter()

                # calculate execution time in milliseconds
                execution_time_ms = (end_time - start_time) * 1000

                # create a dictionary containing performance metrics
                metrics = {
                    "path_length": len(path) - 1,
                    "nodes_explored": nodes_explored,
                    "execution_time_ms": execution_time_ms
                }

                # return both the path and performance metrics
                return path, metrics

            # examine every valid neighbor
            for neighbor in self.get_neighbors(current):

                # skip neighbors that have already been explored
                if neighbor in closed_set:
                    continue

                # calculate the cost of reaching this neighbor
                tentative_g_score = g_score[current] + 1

                # check whether this route is better
                if (
                    neighbor not in g_score
                    or tentative_g_score < g_score[neighbor]
                ):

                    # record the previous position
                    came_from[neighbor] = current

                    # update the cost from the start
                    g_score[neighbor] = tentative_g_score

                    # calculate the heuristic distance
                    h_score = self.heuristic(neighbor, goal)

                    # calculate the total estimated cost
                    f_score = tentative_g_score + h_score

                    # add the neighbor to the priority queue
                    heapq.heappush(
                        open_set,
                        (f_score, neighbor)
                    )

        # calculate execution time even if no path exists
        end_time = time.perf_counter()

        execution_time_ms = (end_time - start_time) * 1000

        # create metrics for the failed search
        metrics = {
            "path_length": None,
            "nodes_explored": nodes_explored,
            "execution_time_ms": execution_time_ms
        }

        # return no path along with the search metrics
        return None, metrics
    
    
    # define the dijkstra path-planning algorithm
class DijkstraPlanner:

    # initialize the planner with the environment
    def __init__(self, environment):
        self.environment = environment

    # return all valid neighboring cells
    def get_neighbors(self, position):

        # unpack the current row and column
        row, col = position

        # define the four possible movement directions
        directions = [
            (-1, 0),  # move up
            (1, 0),   # move down
            (0, -1),  # move left
            (0, 1)    # move right
        ]

        # create a list to store valid neighbors
        neighbors = []

        # examine every possible movement direction
        for row_change, col_change in directions:

            # calculate the neighboring position
            new_row = row + row_change
            new_col = col + col_change

            # only include cells that are valid and not obstacles
            if self.environment.is_free(new_row, new_col):
                neighbors.append((new_row, new_col))

        # return the valid neighboring positions
        return neighbors

    # reconstruct the final path from goal back to start
    def reconstruct_path(self, came_from, current):

        # begin with the goal
        path = [current]

        # trace backward until the starting position is reached
        while current in came_from:

            # retrieve the previous position
            current = came_from[current]

            # add it to the path
            path.append(current)

        # reverse the path so it runs from start to goal
        path.reverse()

        # return the completed path
        return path

    # execute dijkstra's shortest-path algorithm
    def find_path(self, start, goal):

        # record the starting time
        start_time = time.perf_counter()

        # create the priority queue
        open_set = []

        # add the start position with a cost of 0
        heapq.heappush(open_set, (0, start))

        # store how each position was reached
        came_from = {}

        # store the shortest known distance from the start
        distance = {start: 0}

        # store positions that have already been explored
        closed_set = set()

        # count how many unique nodes are explored
        nodes_explored = 0

        # continue while there are positions available to explore
        while open_set:

            # retrieve the position with the lowest distance
            current_distance, current = heapq.heappop(open_set)

            # skip positions that have already been processed
            if current in closed_set:
                continue

            # mark the position as explored
            closed_set.add(current)

            # increase the explored-node counter
            nodes_explored += 1

            # check whether the destination has been reached
            if current == goal:

                # reconstruct the shortest path
                path = self.reconstruct_path(came_from, current)

                # record the ending time
                end_time = time.perf_counter()

                # calculate execution time in milliseconds
                execution_time_ms = (end_time - start_time) * 1000

                # store performance metrics
                metrics = {
                    "path_length": len(path) - 1,
                    "nodes_explored": nodes_explored,
                    "execution_time_ms": execution_time_ms
                }

                # return the path and metrics
                return path, metrics

            # examine every valid neighboring position
            for neighbor in self.get_neighbors(current):

                # ignore neighbors that were already processed
                if neighbor in closed_set:
                    continue

                # each grid movement has a cost of 1
                new_distance = distance[current] + 1

                # check whether this is the best route found so far
                if (
                    neighbor not in distance
                    or new_distance < distance[neighbor]
                ):

                    # update the shortest known distance
                    distance[neighbor] = new_distance

                    # remember where this neighbor came from
                    came_from[neighbor] = current

                    # add the neighbor to the priority queue
                    heapq.heappush(
                        open_set,
                        (new_distance, neighbor)
                    )

        # record the ending time if no path was found
        end_time = time.perf_counter()

        # calculate total search time
        execution_time_ms = (end_time - start_time) * 1000

        # store metrics for the unsuccessful search
        metrics = {
            "path_length": None,
            "nodes_explored": nodes_explored,
            "execution_time_ms": execution_time_ms
        }

        # return no path along with the metrics
        return None, metrics