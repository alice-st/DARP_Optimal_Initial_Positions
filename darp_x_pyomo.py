from __future__ import division

import sys
sys.path.append('DARP')
from darpinPoly import DARPinPoly
import pyomo.environ as pyo
from pyomo.environ import *
from pyomo.opt import SolverFactory

import glpk
#import ipopt
from pyomo.opt import SolverFactory
import localsolver

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
    with localsolver.LocalSolver() as ls:
        model = ls.model
        x = model.float(-5, 10)
        y = model.float(0, 15)
        f = model.double_blackbox_function(branin_eval)
        call = model.call()
        call.add_operand(f)
        call.add_operand(x)
        call.add_operand(y)
        model.minimize(call)
        model.close()

        f.blackbox_context.set_evaluation_limit(20)
        ls.solve()

        sol = ls.solution
        print("x = {}".format(sol.get_value(x)))
        print("y = {}".format(sol.get_value(y)))
        print("obj = {}".format(sol.get_value(call)))
    # model = ConcreteModel()
    # # model.x = Var(bounds=(1.0,10.0),initialize=5.0)
    # # model.y = Var(within=Binary)

    # # model.c1 = Constraint(expr=(model.x-4.0)**2 - model.x <= 50.0*(1-model.y))
    # # model.c2 = Constraint(expr=model.x*log(model.x)+5.0 <= 50.0*(model.y))

    # # model.objective = Objective(expr=model.x, sense=minimize)

    # # SolverFactory('mindtpy').solve(model, mip_solver='glpk', nlp_solver='ipopt')
    # # model.objective.display()
    # # model.display()
    # # model.pprint()

    # # import pdb
    # # pdb.set_trace()

    # # model.x = pyo.RangeSet(0, (rows*cols)-1)
    # # model.y = pyo.RangeSet(0, (rows*cols)-1)
    # # model.KV = pyo.Set(dimen=2, initialize=kv_init)
    # model.d1_x = Var(within=Integers, bounds=(0, rows), initialize=0)
    # model.d1_y = Var(within=Integers, bounds=(0, cols), initialize=0)
    # model.d2_x = Var(within=Integers, bounds=(0, rows), initialize=0)
    # model.d2_y = Var(within=Integers, bounds=(0, cols), initialize=1)

    # model.objective = Objective(rule=return_result, sense=minimize)
    # solver = SolverFactory('ipopt')
    # solver.solve(model, tee=True)
    # # model.objective.display()
    # # model.display()
    # # model.pprint()


def kv_init(m):
    return ((k, v) for k in m.x for v in m.y)


def calc_avg(model):
    init_coordinates = []

    init_coordinates.append((model.d1_x, model.d1_y))
    init_coordinates.append((model.d2_x, model.d2_y))
    import pdb
    pdb.set_trace()
    turns = DARPinPoly(rows, cols, MaxIter, CCvariation, randomLevel, dcells, importance, nep, init_coordinates, portions, obstacles_coords, False)
    minimun_avg = sys.maxsize
    minimum_mode = 1
    for mode, val in turns.mode_to_drone_turns.items():
        if val.avg < minimun_avg:
            minimun_avg = val.avg
            minimum_mode = mode
    return minimun_avg


def return_result(model):
    return (calc_avg(model) for n in model.d1_x)#for m in model.d1_y for k in model.d2_x for s in model.d2_y)


if __name__ == '__main__':
    main()
