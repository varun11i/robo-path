# import the gridenvironment class from environment.py
from environment import GridEnvironment

# import the astarplanner class from planner.py
from planner import AStarPlanner, DijkstraPlanner


def main():

    # create the environment
    environment = GridEnvironment(rows=20, cols=20)

    # define the starting position
    start = (1, 1)

    # define the destination
    goal = (18, 18)

    # randomly generate obstacles
    environment.generate_random_obstacles(
        obstacle_probability=0.20,
        start=start,
        goal=goal
    )

    # create both path planners
    astar_planner = AStarPlanner(environment)
    dijkstra_planner = DijkstraPlanner(environment)

    # run a* on the randomized environment
    astar_path, astar_metrics = astar_planner.find_path(
        start,
        goal
    )

    # run dijkstra on the exact same environment
    dijkstra_path, dijkstra_metrics = dijkstra_planner.find_path(
        start,
        goal
    )

    # display the a* results
    print("\nA* Results")
    print("----------------------")

    if astar_path is not None:
        print("Path found: Yes")
        print("Path length:", astar_metrics["path_length"])
    else:
        print("Path found: No")

    print(
        "Nodes explored:",
        astar_metrics["nodes_explored"]
    )

    print(
        "Execution time:",
        f"{astar_metrics['execution_time_ms']:.4f} ms"
    )

    # display the dijkstra results
    print("\nDijkstra Results")
    print("----------------------")

    if dijkstra_path is not None:
        print("Path found: Yes")
        print(
            "Path length:",
            dijkstra_metrics["path_length"]
        )
    else:
        print("Path found: No")

    print(
        "Nodes explored:",
        dijkstra_metrics["nodes_explored"]
    )

    print(
        "Execution time:",
        f"{dijkstra_metrics['execution_time_ms']:.4f} ms"
    )

    # display a direct comparison if both algorithms found a path
    if astar_path is not None and dijkstra_path is not None:

        print("\nAlgorithm Comparison")
        print("----------------------")

        print(
            "A* nodes explored:",
            astar_metrics["nodes_explored"]
        )

        print(
            "Dijkstra nodes explored:",
            dijkstra_metrics["nodes_explored"]
        )

        # calculate the difference in explored nodes
        node_difference = (
            dijkstra_metrics["nodes_explored"]
            - astar_metrics["nodes_explored"]
        )

        print(
            "Difference in nodes explored:",
            node_difference
        )

        # calculate the percentage reduction in explored nodes
        if dijkstra_metrics["nodes_explored"] > 0:

            node_reduction = (
                node_difference
                / dijkstra_metrics["nodes_explored"]
            ) * 100

            print(
                "A* node reduction:",
                f"{node_reduction:.2f}%"
            )

    # visualize the a* route
    environment.visualize(
        start=start,
        goal=goal,
        path=astar_path
    )


if __name__ == "__main__":
    main()