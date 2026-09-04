# import csv so benchmark results can be saved for later analysis
import csv

# import os for creating output folders
import os

# import statistics for calculating average values
import statistics

# import datetime so every benchmark file gets a unique name
from datetime import datetime

# import the environment and path-planning algorithms
from environment import GridEnvironment
from planner import AStarPlanner, DijkstraPlanner


def run_benchmark(
    number_of_runs=100,
    rows=20,
    cols=20,
    obstacle_probability=0.20
):

    # store the results from every randomized environment
    results = []

    # count how many environments each algorithm solves
    astar_successes = 0
    dijkstra_successes = 0

    # count cases where both algorithms find paths but path lengths differ
    path_mismatches = 0

    # repeat the experiment for the requested number of runs
    for run_number in range(1, number_of_runs + 1):

        # create a completely new environment for this experiment
        environment = GridEnvironment(
            rows=rows,
            cols=cols
        )

        # define the start and goal positions
        start = (1, 1)
        goal = (rows - 2, cols - 2)

        # create a new random obstacle configuration
        environment.generate_random_obstacles(
            obstacle_probability=obstacle_probability,
            start=start,
            goal=goal
        )

        # create both planners using the exact same environment
        astar_planner = AStarPlanner(environment)
        dijkstra_planner = DijkstraPlanner(environment)

        # run a*
        astar_path, astar_metrics = astar_planner.find_path(
            start,
            goal
        )

        # run dijkstra
        dijkstra_path, dijkstra_metrics = (
            dijkstra_planner.find_path(
                start,
                goal
            )
        )

        # determine whether each algorithm reached the goal
        astar_success = astar_path is not None
        dijkstra_success = dijkstra_path is not None

        # update the success counters
        if astar_success:
            astar_successes += 1

        if dijkstra_success:
            dijkstra_successes += 1

        # verify that both algorithms return the same optimal path length
        if astar_success and dijkstra_success:

            if (
                astar_metrics["path_length"]
                != dijkstra_metrics["path_length"]
            ):
                path_mismatches += 1

        # calculate node reduction when dijkstra explored at least one node
        if dijkstra_metrics["nodes_explored"] > 0:

            node_reduction_percent = (
                (
                    dijkstra_metrics["nodes_explored"]
                    - astar_metrics["nodes_explored"]
                )
                / dijkstra_metrics["nodes_explored"]
            ) * 100

        else:
            node_reduction_percent = 0

        # save the results for this individual environment
        results.append(
            {
                "run": run_number,
                "astar_success": astar_success,
                "dijkstra_success": dijkstra_success,
                "astar_path_length":
                    astar_metrics["path_length"],
                "dijkstra_path_length":
                    dijkstra_metrics["path_length"],
                "astar_nodes_explored":
                    astar_metrics["nodes_explored"],
                "dijkstra_nodes_explored":
                    dijkstra_metrics["nodes_explored"],
                "astar_execution_time_ms":
                    astar_metrics["execution_time_ms"],
                "dijkstra_execution_time_ms":
                    dijkstra_metrics["execution_time_ms"],
                "astar_node_reduction_percent":
                    node_reduction_percent
            }
        )

        # display progress in the terminal
        print(
            f"completed run "
            f"{run_number}/{number_of_runs}"
        )

    # calculate overall success rates
    astar_success_rate = (
        astar_successes
        / number_of_runs
    ) * 100

    dijkstra_success_rate = (
        dijkstra_successes
        / number_of_runs
    ) * 100

    # calculate average nodes explored across all environments
    average_astar_nodes = statistics.mean(
        result["astar_nodes_explored"]
        for result in results
    )

    average_dijkstra_nodes = statistics.mean(
        result["dijkstra_nodes_explored"]
        for result in results
    )

    # calculate average execution times
    average_astar_time = statistics.mean(
        result["astar_execution_time_ms"]
        for result in results
    )

    average_dijkstra_time = statistics.mean(
        result["dijkstra_execution_time_ms"]
        for result in results
    )

    # calculate the average reduction in explored nodes
    average_node_reduction = statistics.mean(
        result["astar_node_reduction_percent"]
        for result in results
    )

    # keep only environments where both algorithms found a path
    successful_results = [
        result
        for result in results
        if (
            result["astar_success"]
            and result["dijkstra_success"]
        )
    ]

    # calculate average path length only for successful environments
    if successful_results:

        average_path_length = statistics.mean(
            result["astar_path_length"]
            for result in successful_results
        )

    else:
        average_path_length = None

    # display the final benchmark summary
    print("\nBenchmark Summary")
    print("==============================")

    print("Total environments:", number_of_runs)

    print(
        "A* success rate:",
        f"{astar_success_rate:.2f}%"
    )

    print(
        "Dijkstra success rate:",
        f"{dijkstra_success_rate:.2f}%"
    )

    if average_path_length is not None:
        print(
            "Average successful path length:",
            f"{average_path_length:.2f}"
        )

    print(
        "Average A* nodes explored:",
        f"{average_astar_nodes:.2f}"
    )

    print(
        "Average Dijkstra nodes explored:",
        f"{average_dijkstra_nodes:.2f}"
    )

    print(
        "Average A* node reduction:",
        f"{average_node_reduction:.2f}%"
    )

    print(
        "Average A* execution time:",
        f"{average_astar_time:.4f} ms"
    )

    print(
        "Average Dijkstra execution time:",
        f"{average_dijkstra_time:.4f} ms"
    )

    print(
        "Optimal-path mismatches:",
        path_mismatches
    )

    # create a folder for benchmark results
    os.makedirs(
        "outputs/benchmarks",
        exist_ok=True
    )

    # generate a unique filename
    timestamp = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )

    filename = (
        f"outputs/benchmarks/"
        f"benchmark_{timestamp}.csv"
    )

    # define the columns stored in the csv file
    fieldnames = [
        "run",
        "astar_success",
        "dijkstra_success",
        "astar_path_length",
        "dijkstra_path_length",
        "astar_nodes_explored",
        "dijkstra_nodes_explored",
        "astar_execution_time_ms",
        "dijkstra_execution_time_ms",
        "astar_node_reduction_percent"
    ]

    # save all benchmark results to csv
    with open(
        filename,
        "w",
        newline=""
    ) as csv_file:

        writer = csv.DictWriter(
            csv_file,
            fieldnames=fieldnames
        )

        # write the column names
        writer.writeheader()

        # write every experiment
        writer.writerows(results)

    # display where the benchmark results were saved
    print(
        "\nBenchmark results saved to:",
        filename
    )


if __name__ == "__main__":

    run_benchmark(
        number_of_runs=100,
        rows=20,
        cols=20,
        obstacle_probability=0.20
    )