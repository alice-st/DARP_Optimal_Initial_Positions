import itertools
import multiprocessing
import sys

sys.path.append('DARP')
from mpl_toolkits import mplot3d
from ortools.sat.python import cp_model
import matplotlib.pyplot as plt
import numpy as np

from DARP.darpinPoly import DARPinPoly

results_list = []
cols = 10
rows = 10
MaxIter = 80000
CCvariation = 0.01
randomLevel = 0.0001
dcells = 2
importance = False
nep = False
portions = []
obstacles_positions = [2, 3, 4]
obstacles_coords = []
for obstacle in obstacles_positions:
    obstacles_coords.append((obstacle // cols, obstacle % cols))


def main():
    num_drones = 2
    possible_possitions = rows*cols

    positions = range(possible_possitions)
    # remove obstacles
    positions = [position for position in positions if position not in obstacles_positions]

    pool = multiprocessing.Pool()
    async_results_list = []
    for init_positions in itertools.combinations(positions, num_drones):
        print(init_positions)
        async_results_list.append(pool.apply_async(calculate_avgs, (init_positions,), callback=append_result))

    for async_result in async_results_list:
        async_result.wait()

    for result in results_list:
        print(result.init_coordinates, result.mode, result.avg)
    return


def calculate_avgs(init_positions: tuple):
    init_coordinates = []
    for init_position in init_positions:
        init_coordinates.append((init_position // cols, init_position % cols))
    turns = DARPinPoly(rows, cols, MaxIter, CCvariation, randomLevel, dcells, importance, nep, init_coordinates, portions, obstacles_coords, False)

    minimun_avg = sys.maxsize
    minimum_mode = 1
    for mode, val in turns.mode_to_drone_turns.items():
        if val.avg < minimun_avg:
            minimun_avg = val.avg
            minimum_mode = mode

    return results(init_positions, minimum_mode, minimun_avg)


def append_result(res):
    results_list.append(res)


class results():
    def __init__(self, init_coordinates, mode, avg):
        self.init_coordinates = init_coordinates
        self.mode = mode
        self.avg = avg


if __name__ == '__main__':
    main()
