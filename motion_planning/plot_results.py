import csv
from collections import defaultdict
import matplotlib.pyplot as plt


def load_results(path="results.csv"):
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def get_best_params(rows):
    """
    Find:
      - best step_size for RRT (min average time over all iterations)
      - best k_neighbors for PRM (min average time over all samples)
    """
    rrt_time_by_step = defaultdict(list)
    prm_time_by_k = defaultdict(list)

    for r in rows:
        if r["success"] != "True":
            continue
        algo = r["algorithm"]
        t = float(r["planning_time"])
        if algo == "RRT":
            step = float(r["step_size"])
            rrt_time_by_step[step].append(t)
        elif algo == "PRM":
            k = int(r["k_neighbors"]) if r["k_neighbors"] else None
            if k is not None:
                prm_time_by_k[k].append(t)

    best_step = min(rrt_time_by_step, key=lambda s: sum(rrt_time_by_step[s]) / len(rrt_time_by_step[s]))
    best_k = min(prm_time_by_k, key=lambda k: sum(prm_time_by_k[k]) / len(prm_time_by_k[k]))

    return best_step, best_k


def plot_bar_time(rows, best_step, best_k):
    """
    Bar Plot 1:
      X-axis: max_iterations (RRT) / num_samples (PRM)
      Y-axis: average planning time (s)
      Use best_step for RRT and best_k for PRM.
    """
    agg = defaultdict(list)

    for r in rows:
        if r["success"] != "True":
            continue
        algo = r["algorithm"]
        t = float(r["planning_time"])

        if algo == "RRT":
            step = float(r["step_size"])
            if step != best_step:
                continue
            iters = int(r["max_iterations"])
            key = (algo, iters)
        else:  # PRM
            k = int(r["k_neighbors"]) if r["k_neighbors"] else None
            if k != best_k:
                continue
            samples = int(r["num_samples"])
            key = (algo, samples)

        agg[key].append(t)

    labels = []
    values = []
    for (algo, param), times in sorted(agg.items(), key=lambda t: t[0]):
        labels.append(f"{algo} {param}")
        values.append(sum(times) / len(times))

    plt.figure()
    plt.bar(labels, values)
    plt.xticks(rotation=45)
    plt.ylabel("Average planning time (s)")
    plt.title(f"RRT (step={best_step}) vs PRM (k={best_k})")
    plt.tight_layout()
    plt.savefig("bar_time.png")
    plt.close()


def plot_bar_path_length(rows, best_step, best_k):
    """
    Bar Plot 2:
      X-axis: max_iterations / num_samples
      Y-axis: average path length
      Same best parameters as bar_time.
    """
    agg = defaultdict(list)

    for r in rows:
        if r["success"] != "True":
            continue
        algo = r["algorithm"]
        L = float(r["path_length"]) if r["path_length"] else None
        if L is None:
            continue

        if algo == "RRT":
            step = float(r["step_size"])
            if step != best_step:
                continue
            iters = int(r["max_iterations"])
            key = (algo, iters)
        else:  # PRM
            k = int(r["k_neighbors"]) if r["k_neighbors"] else None
            if k != best_k:
                continue
            samples = int(r["num_samples"])
            key = (algo, samples)

        agg[key].append(L)

    labels = []
    values = []
    for (algo, param), lengths in sorted(agg.items(), key=lambda t: t[0]):
        labels.append(f"{algo} {param}")
        values.append(sum(lengths) / len(lengths))

    plt.figure()
    plt.bar(labels, values)
    plt.xticks(rotation=45)
    plt.ylabel("Average path length")
    plt.title(f"RRT (step={best_step}) vs PRM (k={best_k})")
    plt.tight_layout()
    plt.savefig("bar_path_length.png")
    plt.close()


if __name__ == "__main__":
    rows = load_results("results.csv")
    best_step, best_k = get_best_params(rows)
    print("Best RRT step_size:", best_step)
    print("Best PRM k_neighbors:", best_k)

    plot_bar_time(rows, best_step, best_k)
    plot_bar_path_length(rows, best_step, best_k)
    print("Saved bar_time.png and bar_path_length.png")
