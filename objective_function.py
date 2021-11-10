from botorch.settings import suppress_botorch_warnings
from botorch.settings import validate_input_scaling
import optuna
from optuna.samplers import TPESampler
import sys
sys.path.append('DARP')
from darpinPoly import DARPinPoly


class Opt(object):
    def __init__(self, rows, cols, MaxIter, CCvariation, randomLevel, dcells, importance, nep, portions, obstacles_positions, vis, num_drones):
        self.number_of_drones = num_drones
        self.cols = cols
        self.rows = rows
        self.MaxIter = MaxIter
        self.CCvariation = CCvariation
        self.randomLevel = randomLevel
        self.dcells = dcells
        self.importance = importance
        self.nep = nep
        self.portions = portions
        self.obstacles_positions = obstacles_positions
        self.obstacles_coords = []
        for obstacle in self.obstacles_positions:
            self.obstacles_coords.append((obstacle // self.cols, obstacle % self.cols))

        self.n_startup_trials = 10

    def objective(self, trial):
        positions = []
        for i in range(self.number_of_drones):
            positions.append(trial.suggest_int(f"p{i}", 0, self.rows*self.cols-1))

        if all(element == positions[0] for element in positions):
            return sys.maxsize

        for obstacle in self.obstacles_positions:
            for p in positions:
                if p == obstacle:
                    return sys.maxsize

        init_coordinates = []
        for p in positions:
            init_coordinates.append((p // self.cols, p % self.cols))

        turns = DARPinPoly(self.rows, self.cols, self.MaxIter, self.CCvariation, self.randomLevel, self.dcells, self.importance, self.nep, init_coordinates, self.portions, self.obstacles_coords, False)
        minimun_avg = sys.maxsize
        if not turns.success:
            return minimun_avg
        else:
            minimum_mode = 1
            for mode, val in turns.mode_to_drone_turns.items():
                if val.avg < minimun_avg:
                    minimun_avg = val.avg
                    minimum_mode = mode

        return minimun_avg