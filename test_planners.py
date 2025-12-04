"""
Test functions for path planning algorithms.
ROBO 201/202 Joint Assignment - Fall 2025

This module contains the polymorphic test function and hyperparameter tuning utilities.
"""

import csv
import random

from motion_planning.path_planner_interface import PathPlanner, Configuration
from motion_planning.obstacles import generate_random_obstacles
from motion_planning.rrt import RRTPlanner
from motion_planning.prm import PRMPlanner


def test_planner(
    planner: PathPlanner,
    scenario_name: str,
    visualize: bool = False,
) -> dict:
    """
    Polymorphic test function for path planning algorithms.

    This function should work with ANY PathPlanner subclass (RRT, PRM, etc.)

    Args:
        planner: An instance of a PathPlanner subclass
        scenario_name: Name of the test scenario
        visualize: Whether to visualize the result

    Returns:
        Dictionary containing:
            - 'success': bool - whether a path was found
            - 'path_length': float - total length of the path
            - 'planning_time': float - time taken to plan (seconds)
            - 'num_nodes': int - number of nodes explored (if applicable)
            - 'scenario': str - name of the test scenario
    """
    # 1. Execute the planning algorithm
    success = planner.plan()

    # 2. Get the path
    path = planner.get_path()

    # 3. Calculate path length
    path_length = 0.0
    if success and len(path) > 1:
        for i in range(len(path) - 1):
            path_length += path[i].distance_to(path[i + 1])

    # 4. Get planning time and number of nodes
    planning_time = planner.get_planning_time()
    num_nodes = planner.get_num_nodes()

    # 6. Optional visualization (left as a placeholder)
    if visualize:
        # You can implement your own visualize.plot_planner(planner) here
        print(f"[{scenario_name}] Visualization not implemented yet.")

    result = {
        "success": success,
        "path_length": path_length,
        "planning_time": planning_time,
        "num_nodes": num_nodes,
        "scenario": scenario_name,
    }

    return result


def main():
    """
    Main function to run path planning experiments.
    Students should modify this to test different scenarios and parameters.
    """
    print("=" * 70)
    print("Path Planning Assignment - ROBO 201/202")
    print("=" * 70)

    # Define workspace
    start = Configuration(1.0, 1.0)
    goal = Configuration(9.0, 9.0)
    bounds = ((0.0, 10.0), (0.0, 10.0))

    num_obstacles_list = [0, 10, 100]

    # Hyperparameter grids
    rrt_param_grid = [
        (0.5, 1000),
        (0.5, 5000),
        (1.0, 1000),
        (1.0, 5000),
        (2.0, 1000),
        (2.0, 5000),
        (2.0, 10000),
    ]

    prm_param_grid = [
        (1000, 5),
        (1000, 10),
        (5000, 5),
        (5000, 10),
        (10000, 5),
        (10000, 10),
        (10000, 15),
    ]

    num_trials_per_setting = 10  # adjust if runtime is high

    # Write all results to CSV (for your plots and tables)
    with open("results.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "algorithm",
                "scenario",
                "step_size",
                "max_iterations",
                "num_samples",
                "k_neighbors",
                "num_obstacles",
                "trial",
                "success",
                "path_length",
                "planning_time",
                "num_nodes",
            ]
        )

        for num_obs in num_obstacles_list:
            for trial in range(num_trials_per_setting):
                scenario_name = f"obs_{num_obs}_trial_{trial}"
                obstacles = generate_random_obstacles(num_obs, bounds)

                # RRT tests
                for step_size, max_iter in rrt_param_grid:
                    rrt = RRTPlanner(
                        start=start,
                        goal=goal,
                        bounds=bounds,
                        step_size=step_size,
                        max_iterations=max_iter,
                    )
                    rrt.set_obstacles(obstacles)
                    res = test_planner(rrt, scenario_name, visualize=False)
                    writer.writerow(
                        [
                            "RRT",
                            scenario_name,
                            step_size,
                            max_iter,
                            "",
                            "",
                            num_obs,
                            trial,
                            res["success"],
                            res["path_length"],
                            res["planning_time"],
                            res["num_nodes"],
                        ]
                    )

                # PRM tests
                for num_samples, k_neighbors in prm_param_grid:
                    prm = PRMPlanner(
                        start=start,
                        goal=goal,
                        bounds=bounds,
                        num_samples=num_samples,
                        k_neighbors=k_neighbors,
                    )
                    prm.set_obstacles(obstacles)
                    res = test_planner(prm, scenario_name, visualize=False)
                    writer.writerow(
                        [
                            "PRM",
                            scenario_name,
                            "",
                            "",
                            num_samples,
                            k_neighbors,
                            num_obs,
                            trial,
                            res["success"],
                            res["path_length"],
                            res["planning_time"],
                            res["num_nodes"],
                        ]
                    )

    print("Finished experiments. Results saved to results.csv")
    print("=" * 70)


if __name__ == "__main__":
    main()
