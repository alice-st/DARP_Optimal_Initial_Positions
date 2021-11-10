from objective_function import *
import argparse

NUM_TRIALS = 200


def optimize(rows, cols, MaxIter, CCvariation, randomLevel, dcells, importance, nep, portions, obstacles_positions, vis, num_drones):

    samplers = [optuna.integration.BoTorchSampler(),
                optuna.samplers.TPESampler(),
                optuna.samplers.CmaEsSampler()]
    results = []
    opt = Opt(rows, cols, MaxIter, CCvariation, randomLevel, dcells, importance, nep, portions, obstacles_positions, vis, num_drones)

    for sampler in samplers:
        study = optuna.create_study(
            study_name="study",
            directions=["minimize"],
            sampler=sampler,
        )
        study.optimize(opt.objective, n_trials=NUM_TRIALS)
        results.append(study)
    counter = 1
    for result in results:
        print(f"Study{counter}")
        print("Number of finished trials: ", len(result.trials))
        for t in result.best_trials:
            print(f"Best trial: {t.number}")
            print(f'Best value: {t.values[0]}')
            print(f'Best param: {t.params}')

        params = []
        for param in result.trials[0].params.items():
            params.append(param[0])
        fig = optuna.visualization.plot_contour(result, params=params)
        fig.show()
        counter += 1

    fig = optuna.visualization.plot_edf(results)
    fig.show()


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
        default=2,
        type=int,
        nargs=1,
        help='Insert desired number of drones')
    args = argparser.parse_args()

    rows, cols = args.grid
    num_drones = args.num_drones[0]

    for obstacle in args.obs_pos:
        if obstacle < 0 or obstacle >= rows*cols:
            print("Obstacles should be inside the Grid.")
            sys.exit(3)

    portions = []

    if args.nep:
        for portion in args.portions:
            portions.append(portion)
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

    MaxIter = 80000
    CCvariation = 0.01
    randomLevel = 0.0001
    dcells = 2
    importance = False

    print("\nInitial Conditions Defined:")
    print("Grid Dimensions:", rows, cols)
    print("Robot Number:", num_drones)
    print("Portions for each Robot:", portions, "\n")

    optimize(rows, cols, MaxIter, CCvariation, randomLevel, dcells, importance, args.nep, portions, args.obs_pos, args.vis, num_drones)

