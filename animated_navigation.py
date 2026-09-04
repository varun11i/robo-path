# import random so obstacles can appear while the robot is moving
import random

# import time so the final animation can remain visible briefly
import time

# import matplotlib for live visualization
import matplotlib.pyplot as plt

# import the existing environment
from environment import GridEnvironment

# import the existing a* planner
from planner import AStarPlanner


# draw the current state of the robot navigation system
def draw_navigation_state(
    environment,
    current,
    goal,
    path,
    trajectory,
    step,
    replans
):

    # clear the previous frame
    plt.clf()

    # display the obstacle grid
    plt.imshow(
        environment.grid,
        cmap="binary",
        origin="upper"
    )

    # draw the currently planned path
    if path is not None:

        path_rows = [
            position[0]
            for position in path
        ]

        path_cols = [
            position[1]
            for position in path
        ]

        plt.plot(
            path_cols,
            path_rows,
            linewidth=2,
            label="Current A* Path"
        )

    # draw the path the robot has actually traveled
    if trajectory:

        trajectory_rows = [
            position[0]
            for position in trajectory
        ]

        trajectory_cols = [
            position[1]
            for position in trajectory
        ]

        plt.plot(
            trajectory_cols,
            trajectory_rows,
            linewidth=3,
            label="Robot Trajectory"
        )

    # draw the robot's current position
    plt.scatter(
        current[1],
        current[0],
        marker="o",
        s=180,
        label="Robot"
    )

    # draw the goal position
    plt.scatter(
        goal[1],
        goal[0],
        marker="*",
        s=220,
        label="Goal"
    )

    # display grid coordinates
    plt.xticks(
        range(environment.cols)
    )

    plt.yticks(
        range(environment.rows)
    )

    # draw the cell grid
    plt.grid(True)

    # display current navigation information
    plt.title(
        f"Dynamic A* Navigation | "
        f"Step: {step} | "
        f"Replans: {replans}"
    )

    # display the legend
    plt.legend(
        loc="upper right"
    )

    # immediately update the figure
    plt.draw()

    # pause briefly so movement is visible
    plt.pause(0.25)


def main():

    # create a 20 by 20 environment
    environment = GridEnvironment(
        rows=20,
        cols=20
    )

    # define start and goal positions
    start = (1, 1)
    goal = (18, 18)

    # generate the initial random obstacles
    environment.generate_random_obstacles(
        obstacle_probability=0.15,
        start=start,
        goal=goal
    )

    # create the a* planner
    planner = AStarPlanner(
        environment
    )

    # calculate the initial path
    path, metrics = planner.find_path(
        start,
        goal
    )

    # stop if the initial map cannot be solved
    if path is None:

        print(
            "No initial path found."
        )

        return

    # store the robot's current position
    current = start

    # record the robot's actual movements
    trajectory = [
        current
    ]

    # count robot movements
    steps = 0

    # count replanning events
    replans = 0

    # track total nodes explored
    total_nodes_explored = (
        metrics["nodes_explored"]
    )

    # track total planning time
    total_planning_time = (
        metrics["execution_time_ms"]
    )

    print(
        "Initial path found."
    )

    print(
        "Initial path length:",
        metrics["path_length"]
    )

    # turn on matplotlib interactive mode
    plt.ion()

    # create the visualization window
    plt.figure(
        figsize=(8, 8)
    )

    # display the initial state
    draw_navigation_state(
        environment,
        current,
        goal,
        path,
        trajectory,
        steps,
        replans
    )

    # continue until the goal is reached
    while current != goal:

        # stop if no usable path remains
        if path is None or len(path) < 2:

            print(
                "Robot cannot continue."
            )

            break

        # randomly decide whether a new obstacle appears
        obstacle_event = (
            random.random() < 0.25
        )

        if obstacle_event:

            # select cells several steps ahead
            # on the current path
            possible_obstacles = path[
                1:min(6, len(path))
            ]

            # do not place an obstacle directly on the goal
            possible_obstacles = [
                position
                for position in possible_obstacles
                if position != goal
            ]

            if possible_obstacles:

                # randomly choose an upcoming path cell
                blocked_position = (
                    random.choice(
                        possible_obstacles
                    )
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

                # briefly redraw the map
                # so the new obstacle becomes visible
                draw_navigation_state(
                    environment,
                    current,
                    goal,
                    path,
                    trajectory,
                    steps,
                    replans
                )

                # calculate a new path
                path, replan_metrics = (
                    planner.find_path(
                        current,
                        goal
                    )
                )

                # record the replanning event
                replans += 1

                # accumulate search statistics
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

                # show the newly calculated route
                draw_navigation_state(
                    environment,
                    current,
                    goal,
                    path,
                    trajectory,
                    steps,
                    replans
                )

        # select the next position
        next_position = path[1]

        # move the robot
        current = next_position

        # record the new position
        trajectory.append(
            current
        )

        # increase movement count
        steps += 1

        print(
            f"Step {steps}: "
            f"robot moved to {current}"
        )

        # remove the completed movement
        # from the current path
        path = path[1:]

        # redraw the animation
        draw_navigation_state(
            environment,
            current,
            goal,
            path,
            trajectory,
            steps,
            replans
        )

    # check whether navigation succeeded
    if current == goal:

        print(
            "\nGoal reached successfully."
        )

    else:

        print(
            "\nNavigation ended before "
            "reaching the goal."
        )

    # display final performance statistics
    print(
        "\nAnimated Navigation Results"
    )

    print(
        "----------------------------"
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

    # show the final robot position
    draw_navigation_state(
        environment,
        current,
        goal,
        path,
        trajectory,
        steps,
        replans
    )

    # turn interactive plotting off
    plt.ioff()

    # keep the final frame visible
    plt.show()


if __name__ == "__main__":
    main()