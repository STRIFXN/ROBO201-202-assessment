Motion Planning Assignment — ROBO 201 / ROBO 202 (Fall 2025)

This repository contains Python implementations of the RRT and PRM motion planning algorithms.
Both planners inherit from the provided PathPlanner abstract base class and follow the required interface.
The project includes:

Full RRTPlanner and PRMPlanner implementations

Obstacle handling (circles & rectangles)

A polymorphic test_planner() function

Automated hyperparameter experiments

Performance analysis plots

Setup Instructions
1. Create and activate virtual environment
python3 -m venv venv
source venv/bin/activate

2. Install required packages
pip install matplotlib

3. Run the automated planner experiments
python test_planners.py

This generates:

results.csv: all experiment data

Optional visualizations if enabled

4. Generate the required plots
python motion_planning/plot_results.py

This produces:

bar_time.png

bar_path_length.png

Place these in the ROBO 201 section below.

=========================================================
ROBO 201 REPORT
=========================================================
1. Introduction
Rapidly-Exploring Random Tree (RRT)

RRT is a sampling-based tree expansion algorithm designed for rapid exploration of high-dimensional or cluttered spaces. It incrementally grows a tree outward from the start configuration toward random samples. RRT is efficient for single-query planning problems because it avoids precomputing a global map.

Probabilistic Roadmap Method (PRM)

PRM is a sampling-based roadmap construction method intended for multi-query environments. It generates many collision-free samples and connects neighbors to build a reusable graph. Future planning queries are solved efficiently using graph search.

Differences at a Glance
Feature	RRT	PRM
Query type	Single query	Multi-query
Preprocessing	None	High
Path quality	Lower	Higher
Scalability	Excellent	Slower with many samples
2. Summary of Experimental Results

Experiments varied:

RRT: step_size ∈ {0.5, 1.0, 2.0}, max_iterations ∈ {1000, 5000, 10000}

PRM: num_samples ∈ {1000, 5000, 10000}, k_neighbors ∈ {5, 10, 15}

Environment obstacle count: {0, 10, 100}

Results are logged in results.csv.

Example Extracted Results
Algorithm	Params	Obstacles	Success	Path Length	Time (s)	Nodes
RRT	step=1.0, iter=5000	0	True	15.63	0.00025	124
RRT	step=2.0, iter=5000	0	True	14.28	0.00021	145
PRM	samples=1000, k=10	0	True	11.68	0.36	1002
PRM	samples=5000, k=10	0	True	10.91	1.37	5002

A full dataset of hundreds of trials is available in results.csv.

3. Required Plots

These plots were generated using plot_results.py.

Bar Plot 1 — Average Planning Time

Best parameters from experiments:

RRT step_size = 2.0

PRM k_neighbors = 10

Bar Plot 2 — Average Path Length

Using the same best parameters:

4. Performance Analysis
4.1 Planning Speed

RRT is consistently faster because it only grows a single tree.

PRM becomes slower as the number of samples increases because it must:

Sample many points

Compute many collision checks

Build and search a graph

4.2 Path Quality

PRM’s roadmap creates shorter and smoother paths on average.

RRT often produces longer, jagged paths due to random exploration.

4.3 Success Rates

With zero or few obstacles, both planners succeed almost always.

With 100 obstacles:

PRM succeeds more often if sample count is high

RRT success depends heavily on step_size and max_iterations

4.4 Scalability

PRM scales poorly with large sample counts → computation grows quadratically.

RRT scales well → number of nodes is limited by max_iterations.

4.5 When to Use Each Algorithm
Scenario	Best Planner	Explanation
Single planning query	RRT	Fast, simple, minimal preprocessing
Repeated queries in same map	PRM	Reusable roadmap
Path smoothness important	PRM	Dense sampling yields better paths
Large, high-dimensional space	RRT	Avoids combinatorial explosion
=========================================================
ROBO 202 REPORT
=========================================================
1. Introduction to Polymorphism and Abstract Classes

Python’s abc module enables abstract classes, ensuring that all planners follow the same interface.
PathPlanner defines abstract methods (plan, get_planning_time, get_num_nodes) that every subclass must implement.

This enforces consistency across RRT and PRM implementations.

2. UML Class Diagram
                 ┌───────────────────────────┐
                 │     PathPlanner (ABC)     │
                 │  + plan()*                │
                 │  + is_collision_free()     │
                 │  + get_path()              │
                 │  + get_planning_time()*    │
                 │  + get_num_nodes()*        │
                 └───────────────▲────────────┘
                                 │
        ┌────────────────────────┴────────────────────────┐
        │                                                 │
┌───────────────────┐                           ┌────────────────────┐
│    RRTPlanner      │                           │    PRMPlanner      │
│  + plan()          │                           │  + plan()          │
│  + get_time()      │                           │  + get_time()      │
│  + get_num_nodes() │                           │  + get_num_nodes() │
└───────────────────┘                           └────────────────────┘

3. Polymorphism: Why It Matters
Benefits in this project:
A single test_planner() function works for both RRT and PRM

result = test_planner(rrt, "scenario_1")
result = test_planner(prm, "scenario_1")

New planners can be added with zero changes to testing code

(e.g., RRT*, PRM*, LazyPRM)

Clean, reusable, extensible architecture

4. Code Snippets Showing Polymorphism
Abstract Method Definition

class PathPlanner(ABC):
    @abstractmethod
    def plan(self) -> bool:
        pass

Subclass Implementation

class RRTPlanner(PathPlanner):
    def plan(self) -> bool:
        ...

Polymorphic Usage

def test_planner(planner: PathPlanner, name: str):
    success = planner.plan()
    path = planner.get_path()
    time = planner.get_planning_time()


Authorship Certificate

This is to certify that the document titled “Motion Planning Assignment — ROBO 201 / ROBO 202 (Fall 2025)” is an original work prepared by the undersigned. The document includes Python implementations and analysis of the RRT and PRM motion planning algorithms, detailed experimental results, and project documentation. All content, including code, experiments, and reports, is the intellectual property of the author named below unless otherwise cited.

Certificate Details:

* Document Title: Motion Planning Assignment — ROBO 201 / ROBO 202 (Fall 2025)
* Description: Contains original implementations and analysis of motion planning algorithms (RRT and PRM), experimental results, and project reports for ROBO 201 and ROBO 202 courses.
* Date of Issue: December 4, 2025
* Author: Ahmed Alteneiji

This certificate affirms the authenticity and authorship of the above document.

Signature: Ahmed
Date: 4/Dec