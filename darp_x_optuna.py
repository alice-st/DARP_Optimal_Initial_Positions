"""
                                        Single-Multi UAV Coverage Mission Planning
"""

import argparse
import numpy as np
import os
import math
import time
import matplotlib as plt
import optuna
import sys
from optuna.samplers import TPESampler
from DARP.multiRobotPathPlanner import MultiRobotPathPlanner
from optuna.structs import TrialPruned
from RealWorld.handleGeo.ConvCoords import ConvCoords
from RealWorld.handleGeo.NodesInPoly import NodesInPoly
from RealWorld.nodesPlacementOptimization.SimulatedAnnealing import SimulatedAnnealing
from RealWorld.nodesPlacementOptimization.Rotate import Rotate
from RealWorld.ConnectComponent import ConnectComponent
from RealWorld.visualizeNEDPaths.visualizeNEDPaths import PlotNEDPaths
from RealWorld.handleGeo.GridPathsToNED import GridPathsToNED
from RealWorld.handleGeo import Dist
from DARPparameters import *

sys.path.append('DARP')


class optimize():
    def __init__(self, rows, cols, MaxIter, CCvariation, randomLevel, dcells, importance, nep, portions,
                 obstacles_positions, vis, num_drones, DarpGrid):

        self.DarpGrid = DarpGrid
        # DARP Parameters
        self.number_of_drones = num_drones
        self.cols = cols
        self.rows = rows
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
        self.vis = vis
        # Optimization Parameters
        self.samplers = [optuna.samplers.TPESampler()]
        # optuna.integration.BoTorchSampler(),
        # optuna.samplers.CmaEsSampler()]
        self.number_of_trials = 100
        self.results = []
        self.all_instances = []

    def optimize(self):

        for sampler in self.samplers:
            study = optuna.create_study(
                study_name="study",
                directions=["minimize"],
                sampler=sampler,
                pruner=optuna.pruners.MedianPruner()
            )
            study.optimize(self.objective, n_trials=self.number_of_trials)
            self.results.append(study)

        for result in self.results:
            best_avg = sys.maxsize
            for t in result.best_trials:
                if self.all_instances[t.number].best_case.avg < best_avg:
                    best_avg = self.all_instances[t.number].best_case.avg
                    self.best_trial = self.all_instances[t.number]

    def objective(self, trial):
        positions = []

        for i in range(self.number_of_drones):
            positions.append(trial.suggest_int(f"p{i}", 0, self.rows * self.cols - 1))

        if len(positions) != len(set(positions)):
            self.all_instances.append("Pruned")
            raise TrialPruned()

        for obstacle in self.obstacles_positions:
            for p in positions:
                if p == obstacle:
                    self.all_instances.append("Pruned")
                    raise TrialPruned()

        multiRobotPathPlanner_instance = MultiRobotPathPlanner(self.rows, self.cols, self.nep, positions, self.portions,
                                                               self.obstacles_coords, self.vis, self.DarpGrid)
        self.all_instances.append(multiRobotPathPlanner_instance)

        if not multiRobotPathPlanner_instance.DARP_success:
            raise TrialPruned()
        else:
            return max(multiRobotPathPlanner_instance.best_case.turns)

    def export_results(self):
        counter = 1
        for result in self.results:
            print(f"Study{counter}")
            print("Number of finished trials: ", len(result.trials))

            if self.obstacles_positions == []:
                obs_flag = "No_obstacles"
            else:
                obs_flag = "With_obstacles"

            if not os.path.exists(
                    f'x = {self.rows}, y = {self.cols}, num_drones = {self.number_of_drones}, obstacles = {obs_flag}'):
                os.mkdir(
                    f'x = {self.rows}, y = {self.cols}, num_drones = {self.number_of_drones}, obstacles = {obs_flag}')

            best_avg = sys.maxsize
            for t in result.best_trials:
                print(f"Best trial: {t.number}")
                print(f'Best value: {t.values[0]}')
                print(f"Best Average: {self.all_instances[t.number].best_case.avg}")
                print(f'Best param: {t.params}')

                with open(
                        f'x = {self.rows}, y = {self.cols}, num_drones = {self.number_of_drones}, obstacles = {obs_flag}/{result.sampler}.txt',
                        "a") as f1:
                    f1.write(f"Best trial: %s \nBest value: %s \nBest param: %s \nBest_avg: %s\n" % (
                        t.number, t.values[0], t.params, self.all_instances[t.number].best_case.avg))
                    f1.close()

            params = []
            for param in result.trials[0].params.items():
                params.append(param[0])

            optuna.visualization.plot_contour(result, params=params)

            all_results = dict()
            for i in range(len(result.trials)):
                temp_all_results = list()
                for index, value in result.trials[i].params.items():
                    temp_all_results.append(value)

                temp_all_results = tuple(temp_all_results)

                if result.trials[i].values != None:
                    all_results[temp_all_results] = result.trials[i].values[0]
                else:
                    all_results[temp_all_results] = None

            with open(
                    f'x = {self.rows}, y = {self.cols}, num_drones = {self.number_of_drones}, obstacles = {obs_flag}/all_results.txt',
                    "a") as f1:
                f1.write(f'Sampler:{result.sampler}\n')

                for pos, res in all_results.items():
                    f1.write(f'{pos}, {res}\n')

                f1.close()

            fig = optuna.visualization.plot_contour(result, params=params)
            fig.write_image(
                f'x = {self.rows}, y = {self.cols}, num_drones = {self.number_of_drones}, obstacles = {obs_flag}/{result.sampler}_contour.png')

            counter += 1


if __name__ == '__main__':

    if real_world:
        """ DEFINE THE DRONE'S SPECS """

        # # Phantom 4 pro Image Sensor specs
        HFOV = 73.4
        hRes = 5472
        ImageWidth = 5472
        ImageHeight = 3648
        SensorWidth = 13.2
        SensorHeight = 8
        FocalLength = 8.8


        def GSD(altitude):
            return ((2 * altitude * getTanFromDegrees(HFOV / 2)) / hRes) * 100


        def getTanFromDegrees(degrees):
            return math.tan(degrees * math.pi / 180)


        def covered(altitude):
            return 2 * altitude * getTanFromDegrees(HFOV / 2)


        def GSDh(altitude):
            return ((altitude * 100) * (SensorHeight / 10)) / ((FocalLength / 10) * ImageHeight)


        def GSDw(altitude):
            return ((altitude * 100) * (SensorWidth / 10)) / ((FocalLength / 10) * ImageWidth)


        """ ---------------------------------- Real World parameter parser ------------------------------------------"""
        geoCoords = []
        geoObstacles = []

        import json

        file = open('inputVariables_test.json')
        data = json.load(file)

        # file = open('D:\AirSim\PythonClient\\adaptive_path_planning\mCPP\inputVariables_Cofly003.json')
        # data = json.load(file)
        #
        # QGIS = True
        # qgis_path = 'D:/AirSim/PythonClient/adaptive_path_planning/qgis/000/13/Polygon000.geojson'
        # # Read polygon data from QGIS:
        # if QGIS:
        #     polygon_file = open(qgis_path)
        #     polygon_data = json.load(polygon_file)
        #     field_name = polygon_data['name']
        #
        #     # Polygon
        #     # for p in polygon_data['features']:
        #     #     for i in range(len(p['geometry']['coordinates'][0][0])):
        #     #         geoCoords.append([p['geometry']['coordinates'][0][0][i][1], p['geometry']['coordinates'][0][0][i][0]])
        #
        #     # LineString
        #     for p in polygon_data['features']:
        #         for i in range(len(p['geometry']['coordinates'][0])):
        #             geoCoords.append([p['geometry']['coordinates'][0][i][1], p['geometry']['coordinates'][0][i][0]])
        #
        # else:

        for i in data['polygon']:
            geoCoords.append([i.get("lat"), i.get("long")])

        if len(data['obstacles']) > 0:
            geoObstacles = [[] for _ in range(len(data['obstacles']))]
            for i in range(len(data['obstacles'])):

                for j in data['obstacles'][i]:
                    geoObstacles[i].append([j.get("lat"), j.get("long")])

        altitude = data['altitude']
        sidelap = data['sidelap']

        scanDist = float("{:.2f}".format(covered(altitude) * (1 - sidelap / 100)))
        print("Scanning Distance:", scanDist)

        droneNo = data['droneNo']
        portions = data['rPortions']
        pathsStrictlyInPoly = data['pathsStrictlyInPoly']

        randomInitPos = False  # If false define in WGS84 the initialPos of the drones
        notEqualPortions = True
        initial_positions = []
        for i in data['initialPos']:
            initial_positions.append([i.get("lat"), i.get("long")])

        """ ----------------------------------- End of real world parameters --------------------------------------- """

        # Convert geographical to local cartesian coordinates
        NED_Coords = ConvCoords(geoCoords, geoObstacles).polygonWGS84ToNED()
        cartUnrotated = NED_Coords

        if len(geoObstacles) > 0:
            obstNED = ConvCoords(geoCoords, geoObstacles).obstaclesToNED()
        else:
            obstNED = []  # [np.complex64(x) for x in range(0)]

        # Rotation and shift optimization (SimulatedAnnealing)
        start = time.time()
        optimalParameters = SimulatedAnnealing()
        optimalParameters.run(cartUnrotated, obstNED, scanDist)
        rotate = Rotate()
        theta = optimalParameters.getOptimalTheta()
        shiftX = optimalParameters.getOptimalShiftX()
        shiftY = optimalParameters.getOptimalShiftY()
        rotate.setTheta(theta)
        cart = rotate.rotatePolygon(cartUnrotated)

        cartObst = [[] for _ in range(len(obstNED))]

        for i in range(len(obstNED)):
            cartObst[i] = rotate.rotatePolygon(obstNED[i])

        print("Time needed to find the optimal solution: ", time.time() - start)
        print(
            " - Optimal theta: ", theta, "\n - Optimal shift in X axis: ", shiftX, "\n - Optimal shift in Y axis: ",
            shiftY,
            "\n")

        # Build grid for paths
        NodesInPoly_var = NodesInPoly(cart, cartObst, scanDist=scanDist,
                                      pathsStrictlyInPoly=pathsStrictlyInPoly,
                                      hideInfo=False, shiftX=shiftX, shiftY=shiftY)

        megaNodesIn = NodesInPoly_var.getMegaNodesInCount()

        megaNodes = NodesInPoly_var.getMegaNodes()

        subNodes = NodesInPoly_var.getSubNodes()

        print("User's defined polygon area: ",
              NodesInPoly_var.getPolygonArea(), " square meters\n")

        # Cases that cannot run
        if megaNodesIn < 1:
            print(
                "\n\n !!! The defined area is too small to fit path for the asked scanning density!" + " Try again a larger area or a smaller scanning density !!! \n\n")
        elif megaNodesIn < droneNo:
            print(
                "\n\n !!! Not enough space to have at least minimum length paths for every drone !!!\n" + "With this configuration you can deploy " + megaNodesIn + " drones at most\n" + "    Number of drones automatically changed to " + megaNodesIn + " in order to have a solution\n" + "\n   If you are not satisfied with the solution you could try to rerun for:\n   - Larger area\n   - Smaller scanning distance\n\n")
            droneNo = megaNodesIn

        # DARP parameters (line 107 at DARPinPOLY.java)
        l = len(megaNodes)
        m = len(megaNodes[0])

        """ In DARPgrid 0 stands for free space 
            1 stands for Obstacle
            2 stands for Drone
        """
        DARPgrid = megaNodes[:, :, 2].astype(int)

        """ ADDON for DARP_initial_positions module """
        grid = np.arange(0, l * m).reshape(l, m)
        # print(grid)
        # obs_pos_to_grid = [(ix, iy) for ix, row in enumerate(DARPgrid) for iy, i in enumerate(row) if i == 1]  # find obstacles position into the grid
        obs_pos_to_grid = np.where(DARPgrid == 1)
        obs_pos_to_grid = np.asarray(obs_pos_to_grid).T
        obs_pos = []
        for elem in obs_pos_to_grid:
            obs_pos.append(grid[elem[0], elem[1]])

        rows = l
        cols = m

        # Check for grid connectivity (?)
        G2G = ConnectComponent()
        connectivityTest = np.abs(DARPgrid - 1)
        G2G.compactLabeling(connectivityTest, m, l, True)
        if G2G.getMaxLabel() > 1:
            print("\n\n !!! The environment grid MUST not have unreachable and/or closed shape regions !!! \n\n")
            exit()

        # Parameters to run DARP
        MaxIter = 80000
        CCvariation = 0.01
        randomLevel = 0.0001
        dcells = 2
        importance = False

        ####################################################################################################################
        #                                  Run with optimal or random initial positions
        ####################################################################################################################

        # Optuna optimization
        optimization = optimize(rows, cols, MaxIter, CCvariation, randomLevel, dcells, importance, notEqualPortions,
                                portions,
                                obs_pos, False, droneNo, DARPgrid)

        optimization.optimize()

        NEDdata = GridPathsToNED(optimization, droneNo, subNodes, rotate)
        init_posNED = NEDdata.init_posGRIDToNED()
        """ WaypointsNED are in the form of WaypointsNED[DroneNo][0]"""
        WaypointsNED = NEDdata.getWaypointsNED()
        # print("Initial positions", init_posNED)
        # print("Waypoints", WaypointsNED)
        """ Visualize NED Paths """
        PlotNEDPaths(cartUnrotated, obstNED, droneNo, WaypointsNED, init_posNED).plot()

    else:
        """ ----------------------------------- Run on grid parameters --------------------------------------------- """
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
            default=[2],
            type=int,
            nargs=1,
            help='Insert desired number of drones')
        args = argparser.parse_args()

        rows, cols = args.grid
        num_drones = args.num_drones[0]

        for obstacle in args.obs_pos:
            if obstacle < 0 or obstacle >= rows * cols:
                print("Obstacles should be inside the Grid.")
                sys.exit(3)

        portions = []

        if args.nep:
            for portion in args.portions:
                portions.append(portion)
        else:
            for drone in range(num_drones):
                portions.append(1 / num_drones)

        if num_drones != len(portions):
            print("Portions should be defined for each drone")
            sys.exit(4)

        s = sum(portions)
        if abs(s - 1) >= 0.0001:
            print("Sum of portions should be equal to 1.")
            sys.exit(1)

        """ -------------------------------- End of grid parameters ------------------------------------------------ """

        MaxIter = 20000
        CCvariation = 0.01
        randomLevel = 0.0001
        dcells = 2
        importance = False

        print("\nInitial Conditions Defined:")
        print("Grid Dimensions:", rows, cols)
        print("Robot Number:", num_drones)
        print("Portions for each Robot:", portions, "\n")

        optimization = optimize(rows, cols, MaxIter, CCvariation, randomLevel, dcells, importance, args.nep, portions,
                                args.obs_pos, args.vis, num_drones)
        optimization.optimize()
        optimization.export_results()

    """
########################################################################################################################
----------------------------------------------  AirSim -----------------------------------------------------------------
########################################################################################################################
    """
    # Write execution time - flight time
    file_name_exec_time = "Flight_time.txt"
    completeName_exe_time = os.path.join('', file_name_exec_time)
    file_exec_time = open(completeName_exe_time, "w")


    def GetMultirotorState(DroneName):
        getdata = client.getMultirotorState(DroneName)
        time.sleep(0.1)  # sleep to avoid BufferError
        return getdata


    import threading


    class Thread_class(threading.Thread):
        def __init__(self, running):
            threading.Thread.__init__(self)
            self.running = running
            self.daemon = True
            self.start()

        def run(self):
            while self.running:
                self.update_path()

        def update_path(self):
            global getpose
            global resulting_path
            # global ned_dist

            global turn

            for i in range(droneNo):
                try:


                    getpose = GetMultirotorState('Drone{}'.format(i + 1))

                    current_ned = np.array(
                        (getpose.kinematics_estimated.position.x_val, getpose.kinematics_estimated.position.y_val))
                    nextWP_ned = np.array((path_drone[i][0].x_val, path_drone[i][0].y_val))
                    ned_dist = np.linalg.norm(current_ned - nextWP_ned)

                    # print(
                    # 	f'Curent position: [{getpose.kinematics_estimated.position.x_val}, {getpose.kinematics_estimated.position.y_val}], '
                    # 	f'Next waypoint: [{path_No1[0].x_val}.{path_No1[0].y_val}], Distance: {ned_dist}')

                    if ned_dist < distance_threshold:
                        del path_drone[i][0]
                        print("WPs to go for Drone{}:".format(i + 1), len(path_drone[i]))

                    if len(path_drone[i]) == 0:
                        #get_time = client.getMultirotorState('Drone{}'.format(i + 1))
                        end_date_time = datetime.fromtimestamp(getpose.timestamp // 1000000000)
                        print('Last WP !! --- Drone{} ends to follow the path at {} ---'.format(i + 1,
                                                                                                end_date_time - start_date_time))

                        # Execution time
                        Execution_time = end_date_time - start_date_time

                        file_exec_time.write(
                            ''.join('Overall execution time for Drone{} is {} sec'.format(i + 1, Execution_time)))
                        file_exec_time.write(''.join('\n'))




                    if ned_dist < corner_radius:  # Corner Radius MODE
                        resulting_path = client.moveOnPathAsync(path_drone[i], 1, 500,
                                                                airsim.DrivetrainType.ForwardOnly,
                                                                airsim.YawMode(False, 0),
                                                                vehicle_name='Drone{}'.format(i + 1))
                        # print("Corner Radius mode for Drone {}".format(i + 1))
                        turn = True  # Drone is in Turn
                    else:

                        resulting_path = client.moveOnPathAsync(path_drone[i], velocity, 500,
                                                                airsim.DrivetrainType.ForwardOnly,
                                                                airsim.YawMode(False, 0),
                                                                vehicle_name='Drone{}'.format(i + 1))

                except:
                    pass


    if AirSim:

        # connect to the AirSim simulator
        import airsim

        # connect to Drone client
        client = airsim.MultirotorClient()
        client.confirmConnection()

        # enable Drone1 - default drone when opening AirSim
        client.enableApiControl(True, "Drone1")
        client.armDisarm(True, "Drone1")

        # add the corresponding drones
        for i in range(droneNo - 1):
            done = client.simAddVehicle(vehicle_name="Drone{}".format(i + 2), vehicle_type="simpleflight",
                                        pose=airsim.Pose(airsim.Vector3r(2 + i, 0, 0),
                                                         airsim.Quaternionr(0, 0, 0, 0)))

            client.enableApiControl(True, "Drone{}".format(i + 2))
            client.armDisarm(True, "Drone{}".format(i + 2))

        z = - altitude
        # store waypoints to list
        path_drone = [[] for _ in range(droneNo)]
        init_pos_drone = []

        # ------------------ Read path from mCPP ---------------------------------------------------------------------------

        for i in range(droneNo):
            init_pos_drone.append(airsim.Vector3r(init_posNED[i][0], init_posNED[i][1], -altitude))
            for j in range(len(WaypointsNED[i][0])):
                # path_No1.append(airsim.Vector3r(WaypointsNED[0][0][i][0] - 120, WaypointsNED[0][0][i][1] - 70, z))
                path_drone[i].append(airsim.Vector3r(WaypointsNED[i][0][j][0], WaypointsNED[i][0][j][1], z))

        # # Visualize waypoints
        """ Press T for path visualization"""
        client.simPlotPoints(points=init_pos_drone, size=50, is_persistent=True)
        client.simPlotLineStrip(points=path_drone[0], color_rgba=[1.0, 2.0, 0, 0], thickness=30, is_persistent=True)
        client.simPlotLineStrip(points=path_drone[1], color_rgba=[1.0, -1.0, 1, 1], thickness=30, is_persistent=True)
        client.simPlotLineStrip(points=path_drone[2], color_rgba=[1, 0, 0, 1], thickness=30, is_persistent=True)
        # for i in range(droneNo):
        #     print(i)
        #     client.simPlotLineStrip(points=path_drone[i], color_rgba=[1.0, i - 2.0, i - 0, i], thickness=30, is_persistent=True)

        airsim.wait_key('Press any key to takeoff')
        for i in range(droneNo):
            f1 = client.takeoffAsync(vehicle_name="Drone{}".format(i + 1)).join()

        print("make sure we are hovering at {} meters...".format(-z))
        for i in range(droneNo):
            hovering = client.moveToZAsync(z, 5, vehicle_name='Drone{}'.format(i + 1)).join()

        # send missions to drones / send initial positions to the drones
        for i in range(droneNo):
            # mission = client.moveOnPathAsync(path_drone[i], velocity, 500, airsim.DrivetrainType.ForwardOnly,
            #                                  airsim.YawMode(False, 0), 3 + 3 / 2, vehicle_name='Drone{}'.format(i + 1))

            initial_pos_on_AirSim = client.moveToPositionAsync(path_drone[i][0].x_val, path_drone[i][0].y_val, path_drone[i][0].z_val, velocity, 500, airsim.DrivetrainType.ForwardOnly,
                                             airsim.YawMode(False, 0), 3 + 3 / 2, vehicle_name='Drone{}'.format(i + 1))

        # Wait until first WP is reached
        movetopath = True
        # movetopath = [True for _ in range(droneNo)]
        movetopath_status = [False for _ in range(droneNo)]
        while movetopath:
            for i in range(droneNo):

                if movetopath_status[i] is False:

                    get_initial_pose = client.getMultirotorState('Drone{}'.format(i + 1))

                    current_ned = np.array(
                        (get_initial_pose.kinematics_estimated.position.x_val,
                         get_initial_pose.kinematics_estimated.position.y_val))
                    nextWP_ned = np.array((path_drone[i][0].x_val, path_drone[i][0].y_val))
                    ned_dist = np.linalg.norm(current_ned - nextWP_ned)

                    if ned_dist < distance_threshold:
                        # start measuring execution time

                        # get_time = client.getMultirotorState('Drone{}'.format(i + 1))
                        # from datetime import datetime
                        #
                        # start_date_time = datetime.fromtimestamp(get_time.timestamp // 1000000000)
                        # print(" --- First WP !! --- Drone{} starts to follow the path at {} ---".format(i + 1,
                        #                                                                                 start_date_time))

                        del path_drone[i][0]
                        movetopath_status[i] = True

                        # movetopath = [False if x == i else x for x in movetopath]
                        print("Drone{}: WPs to go:".format(i + 1), len(path_drone[i]))

                        movetopath = False in (elem is True for elem in movetopath_status)
                        onpath = all(movetopath_status)


                        print("status", movetopath_status)
                        print("moveonpath", movetopath)
                        print("onpath", onpath)
                        # onpath = [True in (elem is False for elem in movetopath) for _ in range(droneNo)]


        for i in range(droneNo):
            get_time = client.getMultirotorState('Drone{}'.format(i + 1))
            from datetime import datetime

            start_date_time = datetime.fromtimestamp(get_time.timestamp // 1000000000)
            print(" --- First WP !! --- Drone{} starts to follow the path at {} ---".format(i + 1,
                                                                                            start_date_time))

        # Start Thread for updating the path -- Background process
        Thread_class(True)

        moveonpath_status = [False for _ in range(droneNo)]
        while onpath:
            for i in range(droneNo):

                if len(path_drone[i]) == 0:
                    moveonpath_status[i] = True
                    # End of execution time
                    # end = time.time()

                    # get_time = client.getMultirotorState('Drone{}'.format(i + 1))
                    # end_date_time = datetime.fromtimestamp(get_time.timestamp // 1000000000)
                    # print('Last WP !! --- Drone{} ends to follow the path at {} ---'.format(i + 1,
                    #                                                                           end_date_time - start_date_time))
                    #
                    # # Execution time
                    # Execution_time = end_date_time - start_date_time
                    #
                    # file_exec_time.write(
                    #     ''.join('Overall execution time for Drone{} is {} sec'.format(i + 1, Execution_time)))
                    # file_exec_time.write(''.join('\n'))
                    # onpath will be False if any Drone finishes its path
                    onpath = not all(moveonpath_status)
                    #onpath = [False if x == i else x for x in onpath]

        # END
        airsim.wait_key('Press any key to reset to original state')
        for i in range(droneNo):
            client.armDisarm(False, "Drone{}".format(i + 1))
            client.reset()

            # that's enough fun for now. let's quit cleanly
            client.enableApiControl(False, "Drone{}".format(i + 1))
        print(
            " -------------- Overall execution for every drone is written in the Flight_time.txt ------------------- ")
        file_exec_time.close()
