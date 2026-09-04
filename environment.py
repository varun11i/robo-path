# import numpy for creating and working with the 2d grid
import numpy as np

# import matplotlib for visualizing the robot environment
import matplotlib.pyplot as plt
import os
from datetime import datetime


# define a class that represents the robot's 2d environment
class GridEnvironment:

    # initialize the environment with a default size of 20 rows and 20 columns
    def __init__(self, rows=20, cols=20):

        # store the number of rows in the grid
        self.rows = rows

        # store the number of columns in the grid
        self.cols = cols

        # create a 2d numpy array filled with zeros
        # 0 represents free space where the robot can move
        # 1 will represent an obstacle
        self.grid = np.zeros((rows, cols), dtype=int)

    # add a single obstacle at a specific row and column
    def add_obstacle(self, row, col):

        # check that the requested position exists inside the grid
        if self.is_valid_position(row, col):

            # change the cell value from 0 to 1 to mark it as an obstacle
            self.grid[row, col] = 1

    # add a rectangular block of obstacles to the environment
    def add_obstacle_rectangle(self, start_row, start_col, end_row, end_col):

        # select all cells inside the specified rectangle
        # setting them equal to 1 marks them as obstacles
        self.grid[start_row:end_row + 1, start_col:end_col + 1] = 1
        
    # randomly generate obstacles throughout the environment
    def generate_random_obstacles(
        self,
        obstacle_probability=0.20,
        start=None,
        goal=None
    ):

        # loop through every cell in the grid
        for row in range(self.rows):
            for col in range(self.cols):

                # never place an obstacle on the start position
                if start is not None and (row, col) == start:
                    continue

                # never place an obstacle on the goal position
                if goal is not None and (row, col) == goal:
                    continue

                # generate a random number between 0 and 1
                random_value = np.random.random()

                # turn the cell into an obstacle based on the probability
                if random_value < obstacle_probability:
                    self.grid[row, col] = 1

    # check whether a row and column are inside the grid boundaries
    def is_valid_position(self, row, col):

        # the row must be between 0 and rows - 1
        # the column must be between 0 and cols - 1
        return 0 <= row < self.rows and 0 <= col < self.cols

    # check whether the robot is allowed to move into a cell
    def is_free(self, row, col):

        # first confirm the position is inside the grid
        # then confirm that the cell contains 0 instead of an obstacle
        return self.is_valid_position(row, col) and self.grid[row, col] == 0


    # display the environment using matplotlib
    # display the environment using matplotlib
    def visualize(self, start=None, goal=None, path=None, algorithm_name="Robot Navigation"):

        # create a new figure
        plt.figure(figsize=(8, 8))

        # display the grid
        plt.imshow(self.grid, cmap="binary", origin="upper")

        # check whether a path was provided
        if path is not None:

            # separate row and column values from the path
            path_rows = [position[0] for position in path]
            path_cols = [position[1] for position in path]

            # draw the planned route
            plt.plot(
                path_cols,
                path_rows,
                linewidth=3,
                label=f"{algorithm_name} Path"
            )

            # draw individual path positions
            plt.scatter(
                path_cols,
                path_rows,
                s=25
            )

        # display the start position
        if start is not None:
            plt.scatter(
                start[1],
                start[0],
                marker="o",
                s=150,
                label="Start"
            )

        # display the goal position
        if goal is not None:
            plt.scatter(
                goal[1],
                goal[0],
                marker="*",
                s=200,
                label="Goal"
            )

        # display axis labels
        plt.xticks(range(self.cols))
        plt.yticks(range(self.rows))

        # display grid lines
        plt.grid(True)

        # display legend
        plt.legend()

        # add title
        plt.title(f"{algorithm_name} Robot Navigation")

        # create an outputs folder if it does not already exist
        os.makedirs("outputs", exist_ok=True)

        # generate a unique timestamp for the filename
        timestamp = datetime.now().strftime(
            "%Y%m%d_%H%M%S_%f"
        )

        # create the complete output filename
        safe_name = (
            algorithm_name
            .lower()
            .replace("*", "star")
            .replace(" ", "_")
        )

        filename = (
            f"outputs/"
            f"{safe_name}_path_{timestamp}.png"
)

        # save the figure before displaying it
        plt.savefig(
            filename,
            dpi=300,
            bbox_inches="tight"
        )

        # print the saved file location
        print(f"Figure saved to: {filename}")

        # display the figure
        plt.show()