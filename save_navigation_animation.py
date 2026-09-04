# import random so dynamic obstacles can appear during navigation
import random

# import os for creating the animation output folder
import os

# import copy so each frame stores its own version of the grid
import copy

# import matplotlib for visualization
import matplotlib.pyplot as plt

# import animation tools for creating the gif
from matplotlib.animation import FuncAnimation, PillowWriter

# import the robot environment
from environment import GridEnvironment

# import the a* planner
from planner import AStarPlanner


def run_navigation():

    # create the environment
    environment = GridEnvironment(
        rows=20,
        cols=20
    )

    # define start and goal positions
    start = (1, 1)
    goal = (18, 18)

    # generate the initial randomized environment
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

    # stop if no initial route exists
    if path is None:
        print("No initial path found.")
        return None

    # set the robot's current position
    current = start

    # store the actual trajectory
    trajectory = [current]

    # count robot movements
    steps = 0

    # count replanning events
    replans = 0

    # store every animation frame
    frames = []

    # store the initial frame
    frames.append(
        {
            "grid": copy.deepcopy(
                environment.grid
            ),
            "current": current,
            "path": path.copy(),
            "trajectory": trajectory.copy(),
            "step": steps,
            "replans": replans,
            "event": "Initial Path"
        }
    )

    # continue until the robot reaches the goal
    while current != goal:

        # stop if no path remains
        if path is None or len(path) < 2:
            break

        # randomly decide whether a new obstacle appears
        obstacle_event = (
            random.random() < 0.25
        )

        if obstacle_event:

            # select several future positions
            # along the current planned path
            possible_obstacles = path[
                1:min(6, len(path))
            ]

            # prevent an obstacle from appearing
            # directly on the goal
            possible_obstacles = [
                position
                for position in possible_obstacles
                if position != goal
            ]

            if possible_obstacles:

                # choose one future cell
                blocked_position = (
                    random.choice(
                        possible_obstacles
                    )
                )

                # add the obstacle
                environment.add_obstacle(
                    blocked_position[0],
                    blocked_position[1]
                )

                print(
                    "Dynamic obstacle:",
                    blocked_position
                )

                # save a frame showing the new obstacle
                frames.append(
                    {
                        "grid": copy.deepcopy(
                            environment.grid
                        ),
                        "current": current,
                        "path": path.copy(),
                        "trajectory": trajectory.copy(),
                        "step": steps,
                        "replans": replans,
                        "event": (
                            f"Obstacle at "
                            f"{blocked_position}"
                        )
                    }
                )

                # replan from the robot's current location
                path, replan_metrics = (
                    planner.find_path(
                        current,
                        goal
                    )
                )

                # increase the replanning counter
                replans += 1

                # stop if the new obstacle
                # makes the goal unreachable
                if path is None:

                    print(
                        "No alternative path exists."
                    )

                    break

                print(
                    "Replanned path length:",
                    replan_metrics[
                        "path_length"
                    ]
                )

                # save another frame showing
                # the newly calculated path
                frames.append(
                    {
                        "grid": copy.deepcopy(
                            environment.grid
                        ),
                        "current": current,
                        "path": path.copy(),
                        "trajectory": trajectory.copy(),
                        "step": steps,
                        "replans": replans,
                        "event": "Path Replanned"
                    }
                )

        # move to the next position
        current = path[1]

        # record the movement
        trajectory.append(
            current
        )

        # increase step counter
        steps += 1

        # remove the completed path position
        path = path[1:]

        # save the new robot position as a frame
        frames.append(
            {
                "grid": copy.deepcopy(
                    environment.grid
                ),
                "current": current,
                "path": (
                    path.copy()
                    if path is not None
                    else None
                ),
                "trajectory": trajectory.copy(),
                "step": steps,
                "replans": replans,
                "event": "Robot Moving"
            }
        )

    # determine whether navigation succeeded
    if current == goal:
        print("\nGoal reached successfully.")
    else:
        print(
            "\nNavigation ended before "
            "reaching the goal."
        )

    print(
        "Robot movements:",
        steps
    )

    print(
        "Replanning events:",
        replans
    )

    # return everything needed
    # to build the animation
    return {
        "frames": frames,
        "start": start,
        "goal": goal,
        "rows": environment.rows,
        "cols": environment.cols
    }


def create_animation(simulation):

    # extract recorded simulation information
    frames = simulation["frames"]
    start = simulation["start"]
    goal = simulation["goal"]
    rows = simulation["rows"]
    cols = simulation["cols"]

    # create the matplotlib figure
    fig, ax = plt.subplots(
        figsize=(8, 8)
    )

    # draw one animation frame
    def update(frame_number):

        # clear the previous frame
        ax.clear()

        # retrieve the current recorded state
        state = frames[frame_number]

        grid = state["grid"]
        current = state["current"]
        path = state["path"]
        trajectory = state["trajectory"]
        step = state["step"]
        replans = state["replans"]
        event = state["event"]

        # display the obstacle environment
        ax.imshow(
            grid,
            cmap="binary",
            origin="upper"
        )

        # draw the currently planned route
        if path is not None:

            path_rows = [
                position[0]
                for position in path
            ]

            path_cols = [
                position[1]
                for position in path
            ]

            ax.plot(
                path_cols,
                path_rows,
                linewidth=2,
                label="Planned A* Path"
            )

        # draw the robot's actual trajectory
        if trajectory:

            trajectory_rows = [
                position[0]
                for position in trajectory
            ]

            trajectory_cols = [
                position[1]
                for position in trajectory
            ]

            ax.plot(
                trajectory_cols,
                trajectory_rows,
                linewidth=3,
                label="Robot Trajectory"
            )

        # draw the starting position
        ax.scatter(
            start[1],
            start[0],
            marker="s",
            s=120,
            label="Start"
        )

        # draw the robot
        ax.scatter(
            current[1],
            current[0],
            marker="o",
            s=180,
            label="Robot"
        )

        # draw the goal
        ax.scatter(
            goal[1],
            goal[0],
            marker="*",
            s=220,
            label="Goal"
        )

        # configure the grid
        ax.set_xticks(
            range(cols)
        )

        ax.set_yticks(
            range(rows)
        )

        ax.grid(True)

        # display current navigation state
        ax.set_title(
            f"Dynamic A* Navigation\n"
            f"Step: {step} | "
            f"Replans: {replans} | "
            f"{event}"
        )

        ax.legend(
            loc="upper right"
        )

    # create the animation
    animation = FuncAnimation(
        fig,
        update,
        frames=len(frames),
        interval=300,
        repeat=False
    )

    # create the output folder
    os.makedirs(
        "outputs/animations",
        exist_ok=True
    )

    # define the gif filename
    filename = (
        "outputs/animations/"
        "dynamic_astar_navigation.gif"
    )

    # create the gif writer
    writer = PillowWriter(
        fps=4
    )

    # save the animation
    animation.save(
        filename,
        writer=writer,
        dpi=120
    )

    # close the matplotlib window
    plt.close(fig)

    print(
        "\nAnimation saved to:",
        filename
    )


def main():

    # run the navigation simulation
    simulation = run_navigation()

    # only create the gif if
    # an initial path was available
    if simulation is not None:

        create_animation(
            simulation
        )


if __name__ == "__main__":
    main()