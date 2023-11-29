## DARP Optimal Initial Positions

Welcome to our project, a comprehensive Multi-Robot Coverage Path Planning module (mCPP) that leverages the DARP algorithm to efficiently cover an area of interest, taking into account predefined NoFly zones/obstacles within a designated grid. This implementation is designed to optimize initial positions for a team of mobile robots, specifically tailored for multi-UAV coverage missions.

### Key Features:

The foundation of our methodology already encompasses a suite of features aimed at maximizing the efficiency of individual paths. However, this project introduces an optimization procedure to further enhance performance:

1. **Shape Complexity Reduction:**
The optimization procedure minimizes the complexity of exclusive sub-regions, thereby reducing the number of turns required in the multi-robot solution.

2. **Energy and Time Efficiency:**
The algorithm not only streamlines path shapes but also improves energy and time efficiency for the entire multi-robot solution.

3. **Launch Points Optimization:**
Precise control of UAV launch points is implemented, suggesting ideal locations for setting up launching/docking stations for long-term coverage operations.

### Optimization Algorithm: Tree-Structured Parzen Estimator (TPE)

In the quest for optimal UAV launch points, the choice of an optimization algorithm is critical. After thorough consideration, we have selected the Tree-Structured Parzen Estimator (TPE) as our primary tool for this task.
Why TPE?

- **Efficiency Through Configuration:**
TPE dedicates more time to configuring and evaluating the most promising sets of input parameters, guided by insights gained from previous evaluations.

- **Versatility in Handling Parameters:**
TPE excels in handling various parameter types, including discrete parameters, a crucial aspect in our specific scenario.

- **Surrogate Function Utilization:**
TPE employs a surrogate function, a probabilistic representation of the objective function, constructed using information from previous evaluations. This aids in making informed decisions for optimal launch points, minimizing the number of turns in the mCPP mission.

This strategic use of TPE ensures a time-efficient and cost-effective convergence towards optimal launch points, making the mCPP objective more attainable with fewer iterations.

Thank you for exploring our project! If you have any questions or suggestions, feel free to reach out.

### **Requirements**
This project was created using:
    - Python version: 3.10.12
    - numpy: 1.26.2
    - matplotlib: 3.8.2
    - optuna: 2.0.0
    - opencv-python: 4.8.1.78
    - pygame: 2.5.2
    - scikit-learn: 1.3.2
    - numba: 0.58.1

### Installation and Running
#### To install the application, use:
```
git clone https://github.com/alice-st/DARP_Optimal_Initial_Positions.git --recurse-submodules
cd DARP_Optimal_Initial_Positions
./requirements.sh darp-venv 
. darp-venv/bin/activate
```

#### To run the application, use:

```
python3 darp_x_optuna.py
```

### Usage
By default, without defining any parameters, the darp_x_optuna is going to run for the following setup:

**Grid Dimensions:** 10 10
**Robot Number:** 3
**Portions for each Robot:** [0.3, 0.3, 0.3] 

The expected results inlude the optimal initial positions for the three robots. The final paths can be found under the directory DARP_Optimal_Initial_Positions/results/


## Optimization Results
### Grid Dimensions: 10x10, Number of drones: 3, Without Obstacles

<p align="left">
  <img width="250" height="250" src="1500_trials/Paths/best_paths/x = 10, y = 10, num_drones = 3, obstacles = No_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/medium_paths/x = 10, y = 10, num_drones = 3, obstacles = No_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/worst_paths/x = 10, y = 10, num_drones = 3, obstacles = No_obstacles.png">
</p>

### Grid Dimensions: 15x20, Number of drones: 3, Without Obstacles

<p align="left">
  <img width="250" height="187.5" src="1500_trials/Paths/best_paths/x = 15, y = 20, num_drones = 3, obstacles = No_obstacles.png">
  <img width="250" height="187.5" src="1500_trials/Paths/medium_paths/x = 15, y = 20, num_drones = 3, obstacles = No_obstacles.png">
  <img width="250" height="187.5" src="1500_trials/Paths/worst_paths/x = 15, y = 20, num_drones = 3, obstacles = No_obstacles.png">
</p>

### Grid Dimensions: 20x20, Number of drones: 3, Without Obstacles

<p align="left">
  <img width="250" height="250" src="1500_trials/Paths/best_paths/x = 20, y = 20, num_drones = 3, obstacles = No_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/medium_paths/x = 20, y = 20, num_drones = 3, obstacles = No_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/worst_paths/x = 20, y = 20, num_drones = 3, obstacles = No_obstacles.png">
</p>

### Grid Dimensions: 10x10, Number of drones: 3, With Obstacles

<p align="left">
  <img width="250" height="250" src="1500_trials/Paths/best_paths/x = 10, y = 10, num_drones = 3, obstacles = With_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/medium_paths/x = 10, y = 10, num_drones = 3, obstacles = With_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/worst_paths/x = 10, y = 10, num_drones = 3, obstacles = With_obstacles.png">
</p>

### Grid Dimensions: 15x20, Number of drones: 3, With Obstacles

<p align="left">
  <img width="250" height="187.5" src="1500_trials/Paths/best_paths/x = 15, y = 20, num_drones = 3, obstacles = With_obstacles.png">
  <img width="250" height="187.5" src="1500_trials/Paths/medium_paths/x = 15, y = 20, num_drones = 3, obstacles = With_obstacles.png">
  <img width="250" height="187.5" src="1500_trials/Paths/worst_paths/x = 15, y = 20, num_drones = 3, obstacles = With_obstacles.png">
</p>

### Grid Dimensions: 20x20, Number of drones: 3, With Obstacles

<p align="left">
  <img width="250" height="250" src="1500_trials/Paths/best_paths/x = 20, y = 20, num_drones = 3, obstacles = With_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/medium_paths/x = 20, y = 20, num_drones = 3, obstacles = With_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/worst_paths/x = 20, y = 20, num_drones = 3, obstacles = With_obstacles.png">
</p>

### Grid Dimensions: 10x10, Number of drones: 6, Without Obstacles

<p align="left">
  <img width="250" height="250" src="1500_trials/Paths/best_paths/x = 10, y = 10, num_drones = 6, obstacles = No_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/medium_paths/x = 10, y = 10, num_drones = 6, obstacles = No_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/worst_paths/x = 10, y = 10, num_drones = 6, obstacles = No_obstacles.png">
</p>

### Grid Dimensions: 15x20, Number of drones: 6, Without Obstacles

<p align="left">
  <img width="250" height="187.5" src="1500_trials/Paths/best_paths/x = 15, y = 20, num_drones = 6, obstacles = No_obstacles.png">
  <img width="250" height="187.5" src="1500_trials/Paths/medium_paths/x = 15, y = 20, num_drones = 6, obstacles = No_obstacles.png">
  <img width="250" height="187.5" src="1500_trials/Paths/worst_paths/x = 15, y = 20, num_drones = 6, obstacles = No_obstacles.png">
</p>

### Grid Dimensions: 20x20, Number of drones: 6, Without Obstacles

<p align="left">
  <img width="250" height="250" src="1500_trials/Paths/best_paths/x = 20, y = 20, num_drones = 6, obstacles = No_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/medium_paths/x = 20, y = 20, num_drones = 6, obstacles = No_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/worst_paths/x = 20, y = 20, num_drones = 6, obstacles = No_obstacles.png">
</p>

### Grid Dimensions: 10x10, Number of drones: 6, With Obstacles

<p align="left">
  <img width="250" height="250" src="1500_trials/Paths/best_paths/x = 10, y = 10, num_drones = 6, obstacles = With_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/medium_paths/x = 10, y = 10, num_drones = 6, obstacles = With_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/worst_paths/x = 10, y = 10, num_drones = 6, obstacles = With_obstacles.png">
</p>

### Grid Dimensions: 15x20, Number of drones: 6, With Obstacles

<p align="left">
  <img width="250" height="187.5" src="1500_trials/Paths/best_paths/x = 15, y = 20, num_drones = 6, obstacles = With_obstacles.png">
  <img width="250" height="187.5" src="1500_trials/Paths/medium_paths/x = 15, y = 20, num_drones = 6, obstacles = With_obstacles.png">
  <img width="250" height="187.5" src="1500_trials/Paths/worst_paths/x = 15, y = 20, num_drones = 6, obstacles = With_obstacles.png">
</p>

### Grid Dimensions: 20x20, Number of drones: 6, With Obstacles

<p align="left">
  <img width="250" height="250" src="1500_trials/Paths/best_paths/x = 20, y = 20, num_drones = 6, obstacles = With_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/medium_paths/x = 20, y = 20, num_drones = 6, obstacles = With_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/worst_paths/x = 20, y = 20, num_drones = 6, obstacles = With_obstacles.png">
</p>


### Grid Dimensions: 10x10, Number of drones: 9, Without Obstacles

<p align="left">
  <img width="250" height="250" src="1500_trials/Paths/best_paths/x = 10, y = 10, num_drones = 9, obstacles = No_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/medium_paths/x = 10, y = 10, num_drones = 9, obstacles = No_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/worst_paths/x = 10, y = 10, num_drones = 9, obstacles = No_obstacles.png">
</p>

### Grid Dimensions: 15x20, Number of drones: 9, Without Obstacles

<p align="left">
  <img width="250" height="187.5" src="1500_trials/Paths/best_paths/x = 15, y = 20, num_drones = 9, obstacles = No_obstacles.png">
  <img width="250" height="187.5" src="1500_trials/Paths/medium_paths/x = 15, y = 20, num_drones = 9, obstacles = No_obstacles.png">
  <img width="250" height="187.5" src="1500_trials/Paths/worst_paths/x = 15, y = 20, num_drones = 9, obstacles = No_obstacles.png">
</p>

### Grid Dimensions: 20x20, Number of drones: 9, Without Obstacles

<p align="left">
  <img width="250" height="250" src="1500_trials/Paths/best_paths/x = 20, y = 20, num_drones = 9, obstacles = No_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/medium_paths/x = 20, y = 20, num_drones = 9, obstacles = No_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/worst_paths/x = 20, y = 20, num_drones = 9, obstacles = No_obstacles.png">
</p>

### Grid Dimensions: 10x10, Number of drones: 9, With Obstacles

<p align="left">
  <img width="250" height="250" src="1500_trials/Paths/best_paths/x = 10, y = 10, num_drones = 9, obstacles = With_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/medium_paths/x = 10, y = 10, num_drones = 9, obstacles = With_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/worst_paths/x = 10, y = 10, num_drones = 9, obstacles = With_obstacles.png">
</p>

### Grid Dimensions: 15x20, Number of drones: 9, With Obstacles

<p align="left">
  <img width="250" height="187.5" src="1500_trials/Paths/best_paths/x = 15, y = 20, num_drones = 9, obstacles = With_obstacles.png">
  <img width="250" height="187.5" src="1500_trials/Paths/medium_paths/x = 15, y = 20, num_drones = 9, obstacles = With_obstacles.png">
  <img width="250" height="187.5" src="1500_trials/Paths/worst_paths/x = 15, y = 20, num_drones = 9, obstacles = With_obstacles.png">
</p>

### Grid Dimensions: 20x20, Number of drones: 9, With Obstacles

<p align="left">
  <img width="250" height="250" src="1500_trials/Paths/best_paths/x = 20, y = 20, num_drones = 9, obstacles = With_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/medium_paths/x = 20, y = 20, num_drones = 9, obstacles = With_obstacles.png">
  <img width="250" height="250" src="1500_trials/Paths/worst_paths/x = 20, y = 20, num_drones = 9, obstacles = With_obstacles.png">
</p>