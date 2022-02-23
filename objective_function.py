from botorch.settings import suppress_botorch_warnings
from botorch.settings import validate_input_scaling
import optuna
from optuna.samplers import TPESampler
import sys
sys.path.append('DARP')
from multiRobotPathPlanner import MultiRobotPathPlanner
from optuna.structs import TrialPruned

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
        self.obstacles_coords = obstacles_positions
    
    def objective(self, trial):
        positions = []
        for i in range(self.number_of_drones):
            positions.append(trial.suggest_int(f"p{i}", 0, self.rows*self.cols-1))

        if len(positions) != len(set(positions)):
            raise TrialPruned()

        for obstacle in self.obstacles_positions:
            for p in positions:
                if p == obstacle:
                    raise TrialPruned()
        
        turns = MultiRobotPathPlanner(self.rows, self.cols, self.nep, positions, self.portions, self.obstacles_coords, False)
        
        minimun_avg = sys.maxsize
        
        if not turns.DARP_success:
            raise TrialPruned()
        else:
            return turns.avg
