from darp_x_optuna import optimize

def main():
    MaxIter = 20000
    CCvariation = 0.01
    randomLevel = 0.0001
    dcells = 2
    importance = False

    optimization_instance = optimize(10, 10, MaxIter, CCvariation, randomLevel, dcells, importance, False, [0.2, 0.3, 0.5], [], False, 3)
                            optimize(rows, cols, number_of_trials, args.nep, portions, args.obs_pos, args.vis, num_drones)

    optimization_instance.optimize()
    # optimization_instance.export_results()

    import pdb
    pdb.set_trace()
if __name__ == '__main__':
    main()