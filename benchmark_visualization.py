# import pandas for reading and analyzing benchmark results
import pandas as pd

# import matplotlib for creating benchmark visualizations
import matplotlib.pyplot as plt

# import os for creating output directories
import os

# import glob for locating the most recent benchmark csv file
import glob


def load_latest_benchmark():

    # find all benchmark csv files
    benchmark_files = glob.glob(
        "outputs/benchmarks/benchmark_*.csv"
    )

    # check whether any benchmark files exist
    if not benchmark_files:
        raise FileNotFoundError(
            "No benchmark CSV files were found."
        )

    # find the most recently created benchmark file
    latest_file = max(
        benchmark_files,
        key=os.path.getctime
    )

    # display which file is being analyzed
    print(
        "Loading benchmark file:",
        latest_file
    )

    # load the csv into a pandas dataframe
    data = pd.read_csv(latest_file)

    return data


def plot_nodes_explored(data):

    # calculate average nodes explored by each algorithm
    astar_average = data[
        "astar_nodes_explored"
    ].mean()

    dijkstra_average = data[
        "dijkstra_nodes_explored"
    ].mean()

    # create the bar chart
    plt.figure(figsize=(7, 5))

    plt.bar(
        ["A*", "Dijkstra"],
        [astar_average, dijkstra_average],
        color=["royalblue", "darkorange"]
    )

    # add axis labels and title
    plt.ylabel("Average Nodes Explored")

    plt.title(
        "A* vs Dijkstra: Search Efficiency"
    )

    # add the actual values above each bar
    plt.text(
        0,
        astar_average,
        f"{astar_average:.2f}",
        ha="center",
        va="bottom"
    )

    plt.text(
        1,
        dijkstra_average,
        f"{dijkstra_average:.2f}",
        ha="center",
        va="bottom"
    )

    # save the figure
    plt.savefig(
        "outputs/benchmarks/nodes_explored_comparison.png",
        dpi=300,
        bbox_inches="tight"
    )

    plt.show()


def plot_execution_time(data):

    # calculate average execution time
    astar_average = data[
        "astar_execution_time_ms"
    ].mean()

    dijkstra_average = data[
        "dijkstra_execution_time_ms"
    ].mean()

    # create the bar chart
    plt.figure(figsize=(7, 5))

    plt.bar(
        ["A*", "Dijkstra"],
        [astar_average, dijkstra_average],
        color=["royalblue", "darkorange"]
    )

    plt.ylabel(
        "Average Execution Time (ms)"
    )

    plt.title(
        "A* vs Dijkstra: Planning Time"
    )

    # display values above the bars
    plt.text(
        0,
        astar_average,
        f"{astar_average:.4f}",
        ha="center",
        va="bottom"
    )

    plt.text(
        1,
        dijkstra_average,
        f"{dijkstra_average:.4f}",
        ha="center",
        va="bottom"
    )

    # save the figure
    plt.savefig(
        "outputs/benchmarks/execution_time_comparison.png",
        dpi=300,
        bbox_inches="tight"
    )

    plt.show()


def plot_success_rate(data):

    # convert boolean success values to percentages
    astar_success = (
        data["astar_success"].mean()
        * 100
    )

    dijkstra_success = (
        data["dijkstra_success"].mean()
        * 100
    )

    # create the bar chart
    plt.figure(figsize=(7, 5))

    plt.bar(
        ["A*", "Dijkstra"],
        [astar_success, dijkstra_success],
        color=["royalblue", "darkorange"]
    )

    plt.ylabel("Success Rate (%)")

    plt.ylim(0, 100)

    plt.title(
        "A* vs Dijkstra: Pathfinding Success Rate"
    )

    # display values above the bars
    plt.text(
        0,
        astar_success,
        f"{astar_success:.2f}%",
        ha="center",
        va="bottom"
    )

    plt.text(
        1,
        dijkstra_success,
        f"{dijkstra_success:.2f}%",
        ha="center",
        va="bottom"
    )

    # save the figure
    plt.savefig(
        "outputs/benchmarks/success_rate_comparison.png",
        dpi=300,
        bbox_inches="tight"
    )

    plt.show()


def plot_node_reduction(data):

    # create the figure
    plt.figure(figsize=(9, 5))

    # plot the percentage reduction for every randomized run
    plt.plot(
        data["run"],
        data["astar_node_reduction_percent"],
        color="royalblue"
    )

    # calculate the overall average reduction
    average_reduction = data[
        "astar_node_reduction_percent"
    ].mean()

    # draw a horizontal line representing the average
    plt.axhline(
        average_reduction,
        color="darkorange",
        linestyle="--",
        label=(
            f"Average Reduction: "
            f"{average_reduction:.2f}%"
        )
    )

    plt.xlabel("Benchmark Run")

    plt.ylabel(
        "A* Node Reduction (%)"
    )

    plt.title(
        "A* Search-Space Reduction Across Randomized Environments"
    )

    plt.legend()

    plt.grid(True)

    # save the figure
    plt.savefig(
        "outputs/benchmarks/node_reduction_by_run.png",
        dpi=300,
        bbox_inches="tight"
    )

    plt.show()


def main():

    # make sure the benchmark output directory exists
    os.makedirs(
        "outputs/benchmarks",
        exist_ok=True
    )

    # load the most recent benchmark data
    data = load_latest_benchmark()

    # create all benchmark figures
    plot_nodes_explored(data)

    plot_execution_time(data)

    plot_success_rate(data)

    plot_node_reduction(data)

    # confirm that the figures were created
    print(
        "\nAll benchmark visualizations saved successfully."
    )


if __name__ == "__main__":
    main()