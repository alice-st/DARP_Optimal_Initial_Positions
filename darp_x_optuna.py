import argparse
import numpy as np
import os
import matplotlib as plt
import optuna
from optuna.samplers import TPESampler
import sys
sys.path.append('DARP')
from multiRobotPathPlanner import MultiRobotPathPlanner
from optuna.structs import TrialPruned
from optuna.trial import TrialState

class optimize():
    def __init__(self, rows, cols, number_of_trials, nep, portions, obstacles_positions, num_drones):
        # DARP Parameters
        self.number_of_drones = num_drones
        self.cols = cols
        self.rows = rows
        self.nep = nep
        self.portions = portions
        self.obstacles_positions = obstacles_positions
        self.obstacles_coords = obstacles_positions
        
        # Optimization Parameters
        self.sampler = optuna.samplers.TPESampler()
        self.number_of_trials = number_of_trials
        self.results = []
        self.best_avg = sys.maxsize
        self.all_instances = []

    def optimize(self):        
        study = optuna.create_study(
            study_name="study",
            direction="minimize",
            sampler=self.sampler,
            pruner=optuna.pruners.MedianPruner()
        )

        study.optimize(self.objective, n_trials=self.number_of_trials)
        self.study = study
        
        #In case more than one trials have the same minimum number of turns, pick the trial with the smaller average number of turns
        self.best_trial_number = self.study.best_trial.number
        self.min_number_of_turns = self.study.best_trial.value
        self.best_trial = self.all_instances[self.best_trial_number]
        self.best_avg = self.best_trial.best_case.avg
        
        for trial in study.trials:
            if trial.state == TrialState.PRUNED:
                continue

            if trial.value <= self.min_number_of_turns and (self.all_instances[trial.number].best_case.avg < self.best_avg):
                self.best_trial_number = trial.number
                self.min_number_of_turns = trial.value
                self.best_trial = self.all_instances[self.best_trial_number]
                self.best_avg = self.best_trial.best_case.avg
        
        print("=========== Optimization Results ===========")        
        print("Number of finished trials: ", len(self.study.trials))
        print(f"Best trial: {self.best_trial_number}")
        print(f'Best value: {self.min_number_of_turns}')
        print(f'Best Initial positions: {self.study.best_trial.params}')
        print(f'Best Initial positions in cartesian coordinates: {self.best_trial.darp_instance.initial_positions}')
        print("============================================")        

    def objective(self, trial):
        positions = []
        for i in range(self.number_of_drones):
            positions.append(trial.suggest_int(f"p{i}", 0, self.rows*self.cols-1))

        if len(positions) != len(set(positions)):
            self.all_instances.append("Pruned")
            raise TrialPruned()
        
        for obstacle in self.obstacles_positions:
            for p in positions:
                if p == obstacle:
                    self.all_instances.append("Pruned")
                    raise TrialPruned()

        multiRobotPathPlanner_instance = MultiRobotPathPlanner(self.rows, self.cols, self.nep, positions, self.portions, self.obstacles_coords, False)
        self.all_instances.append(multiRobotPathPlanner_instance)

        if not multiRobotPathPlanner_instance.DARP_success:
            raise TrialPruned()
        else:
            return max(multiRobotPathPlanner_instance.best_case.turns)

    def export_results(self):
        if not self.obstacles_positions:
            obs_flag = "No_obstacles"
        else:
            obs_flag = "With_obstacles"
        
        if not os.path.exists("Results"):
            os.mkdir("Results")

        if not os.path.exists(f'Results/x={self.rows}_y={self.cols}_num_drones={self.number_of_drones}_{obs_flag}'):
            os.mkdir(f'Results/x={self.rows}_y={self.cols}_num_drones={self.number_of_drones}_{obs_flag}')
    
        with open(f'Results/x={self.rows}_y={self.cols}_num_drones={self.number_of_drones}_{obs_flag}/optimization_results.txt', "a") as txt_file:
            txt_file.write(f"Best trial: {self.best_trial_number}\n")
            txt_file.write(f"Best value: {self.min_number_of_turns}\n")
            txt_file.write(f"Best Initial positions: {self.study.best_trial.params}\n")
            txt_file.write(f"Best Initial positions in cartesian coordinates: {self.best_trial.darp_instance.initial_positions}\n")
            txt_file.write(f"Average number of turns: {self.best_avg}\n")
            txt_file.close()

        all_results = dict()
        for i in range(len(self.study.trials)):
            temp_all_results = list()
            for index, value in self.study.trials[i].params.items():
                temp_all_results.append(value)
            
            temp_all_results = tuple(temp_all_results)
            
            if self.study.trials[i].value is not None:
                all_results[temp_all_results] = self.study.trials[i].value
            else:
                all_results[temp_all_results] = None

        with open(f'Results/x={self.rows}_y={self.cols}_num_drones={self.number_of_drones}_{obs_flag}/all_results.txt', "a") as txt_file:            
            for positions, results in all_results.items():
                txt_file.write(f'Initial positions: {positions}, Number of turns: {results}\n')
            txt_file.close()

        fig = optuna.visualization.plot_contour(self.study)
        fig.write_image(f'Results/x={self.rows}_y={self.cols}_num_drones={self.number_of_drones}_{obs_flag}/Results_contour.png')

    def run_DARP_with_best_results(self, visualization):
        initial_positions = []
        
        for robot, position in self.study.best_trial.params.items():
            initial_positions.append(position)

        multiRobotPathPlanner_instance = MultiRobotPathPlanner(self.rows, self.cols, self.nep, initial_positions, 
                                                               self.portions, self.obstacles_coords, visualization)


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '-grid',
        default=(10, 10),
        type=int,
        nargs=2,
        help='Dimensions of the Grid (default: (10, 10))')
    argparser.add_argument(
        '-obs_pos',
        default=[],
        nargs='*',
        type=int,
        help='Dimensions of the Grid (default: (10, 10))')
    argparser.add_argument(
        '-nep',
        action='store_true',
        help='Not Equal Portions shared between the Robots in the Grid (default: False)')
    argparser.add_argument(
        '-portions',
        default=[0.2, 0.3, 0.5],
        nargs='*',
        type=float,
        help='Portion for each Robot in the Grid (default: (0.2, 0.7, 0.1))')
    argparser.add_argument(
        '-vis',
        action='store_true',
        help='Visualize results (default: False)')
    argparser.add_argument(
        '-num_drones',
        default=[3],
        type=int,
        nargs=1,
        help='Insert desired number of drones')
    argparser.add_argument(
        '-number_of_trials',
        default=[200],
        type=int,
        nargs=1,
        help='Insert desired number of trials')
    
    args = argparser.parse_args()
    rows, cols = args.grid
    num_drones = args.num_drones[0]

    # Sanity checks:
    for obstacle in args.obs_pos:
        if obstacle < 0 or obstacle >= rows*cols:
            print("Obstacles should be inside the Grid.")
            sys.exit(3)

    portions = []
    if args.nep:
        portions.extend(args.portions)
    else:
        for drone in range(num_drones):
            portions.append(1/num_drones)

    if num_drones != len(portions):
        print("Portions should be defined for each drone")
        sys.exit(4)

    s = sum(portions)
    if abs(s-1) >= 0.0001:
        print("Sum of portions should be equal to 1.")
        sys.exit(1)

    print("\nInitial Conditions Defined:")
    print("Grid Dimensions:", rows, cols)
    print("Robot Number:", num_drones)
    print("Portions for each Robot:", portions, "\n")

    optimization = optimize(rows, cols, args.number_of_trials[0], args.nep, portions, args.obs_pos, num_drones)
    optimization.optimize()
    optimization.export_results()
   
    if args.vis:
        optimization.run_DARP_with_best_results(args.vis)