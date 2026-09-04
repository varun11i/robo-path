# import random so obstacles can appear unpredictably while the robot moves
import random

# import the existing environment
from environment import GridEnvironment

# import the existing a* planner
from planner import AStarPlanner


def main():

    # create a new 20 by 20 environment
    environment = GridEnvironment(
        rows=20,
        cols=20
    )

    # define the robot's starting position
    start = (1, 1)

    # define the destination
    goal = (18, 18)

    # generate the initial random obstacle configuration
    environment.generate_random_obstacles(
        obstacle_probability=0.15,
        start=start,
        goal=goal
    )

    # create the a* planner
    planner = AStarPlanner(environment)

    # calculate the initial route
    path, metrics = planner.find_path(
        start,
        goal
    )

    # stop if no initial route exists
    if path is None:

        print("No initial path found.")

        environment.visualize(
            start=start,
            goal=goal,
            path=None,
            algorithm_name="Dynamic A*"
        )

        return

    print("Initial path found.")

    print(
        "Initial path length:",
        metrics["path_length"]
    )

    # current position of the robot
    current = start

    # store every position actually visited by the robot
    trajectory = [current]

    # count how many times the robot replans
    replans = 0

    # accumulate total nodes explored across all planning attempts
    total_nodes_explored = metrics[
        "nodes_explored"
    ]

    # accumulate total planning time
    total_planning_time = metrics[
        "execution_time_ms"
    ]

    # count robot movements
    steps = 0

    # continue until the robot reaches the goal
    while current != goal:

        # make sure a usable path currently exists
        if path is None or len(path) < 2:

            print(
                "Robot cannot continue."
            )

            break

        # give a dynamic obstacle a 25 percent chance
        # of appearing before each movement
        obstacle_event = (
            random.random() < 0.25
        )

        if obstacle_event:

            # look at several positions ahead
            # on the robot's current route
            possible_obstacles = path[
                1:min(6, len(path))
            ]

            # never place an obstacle on the goal
            possible_obstacles = [
                position
                for position in possible_obstacles
                if position != goal
            ]

            # only continue if there is a valid
            # future path position to block
            if possible_obstacles:

                # choose one upcoming path position
                blocked_position = random.choice(
                    possible_obstacles
                )

                # add the new obstacle
                environment.add_obstacle(
                    blocked_position[0],
                    blocked_position[1]
                )

                print(
                    "\nDynamic obstacle appeared at:",
                    blocked_position
                )

                print(
                    "Replanning from:",
                    current
                )

                # calculate a completely new path
                # from the robot's current position
                path, replan_metrics = (
                    planner.find_path(
                        current,
                        goal
                    )
                )

                # increase the replanning counter
                replans += 1

                # accumulate planning statistics
                total_nodes_explored += (
                    replan_metrics[
                        "nodes_explored"
                    ]
                )

                total_planning_time += (
                    replan_metrics[
                        "execution_time_ms"
                    ]
                )

                # check whether the new obstacle
                # made the destination unreachable
                if path is None:

                    print(
                        "No alternative path exists."
                    )

                    break

                print(
                    "New path length:",
                    replan_metrics[
                        "path_length"
                    ]
                )

        # move the robot one position
        # along the current valid path
        next_position = path[1]

        # update the robot's position
        current = next_position

        # record the movement
        trajectory.append(current)

        # increase the movement counter
        steps += 1

        print(
            f"Step {steps}: robot moved to {current}"
        )

        # remove the position that was just reached
        # so path[1] remains the next movement
        path = path[1:]

    # check whether the robot successfully arrived
    if current == goal:

        print(
            "\nGoal reached successfully."
        )

    else:

        print(
            "\nNavigation ended before reaching the goal."
        )

    # display final navigation statistics
    print(
        "\nDynamic Navigation Results"
    )

    print(
        "--------------------------"
    )

    print(
        "Robot movements:",
        steps
    )

    print(
        "Replanning events:",
        replans
    )

    print(
        "Total nodes explored:",
        total_nodes_explored
    )

    print(
        "Total planning time:",
        f"{total_planning_time:.4f} ms"
    )

    # visualize the route the robot
    # actually traveled
    environment.visualize(
        start=start,
        goal=goal,
        path=trajectory,
        algorithm_name="Dynamic A*"
    )


if __name__ == "__main__":
    main()