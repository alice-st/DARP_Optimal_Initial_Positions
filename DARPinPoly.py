import sys
import argparse
import numpy as np
from DARP.darp import DARP
from DARP.kruskal import Kruskal
from DARP.CalculateTrajectories import CalculateTrajectories
from DARP.Visualization import visualize_paths

from DARP.turns import turns


class DARPinPoly(DARP):

    def __init__(self, nx, ny, DARPgrid, notEqualPortions,  positions, portions, obs_pos, visualization, megaNodes, subNodes,
                 randomInitPos, initial_positions, theta, shiftX, shiftY, initializeDARPGrid, rotate,
                 MaxIter=80000, CCvariation=0.01,
                 randomLevel=0.0001, dcells=2, importance=False):

        self.missionWaypointsNED = [[] for _ in range(len(portions))]

        self.darp_instance = DARP(nx, ny, notEqualPortions, positions, portions, obs_pos, visualization, DARPgrid, MaxIter=MaxIter, CCvariation=CCvariation,
                                  randomLevel=randomLevel, dcells=dcells,
                                  importance=importance)

        # if not self.darp_instance.success:
        #     print("DARP did not manage to find a solution for the given configuration!")
        #     sys.exit(0)

        # Check if DARP could find a solution - if not --> rerun for random initial positions
        # success, DARPAssignmentMatrix = poly.update()

        # Divide areas based on robots initial positions
        self.DARP_success, self.iterations = self.darp_instance.divideRegions()


        if self.DARP_success:
            print(" ~ DARP finished with success ~ ")
            self.darp_instance.A = self.darp_instance.A
        else:
            randomInitPos = True
            count = 0
            while self.DARP_success is False and count < 5:
                count += 1
                print("\n!!! DARP will rerun for random initial positions !!! ")
                DARPgrid = megaNodes[:, :, 2].astype(int)

                l = len(megaNodes)
                m = len(megaNodes[0])

                """ ADDON for DARP_initial_positions module """
                grid = np.arange(0, l * m).reshape(l, m)

                # Put drones in random initial positions

                DARPgrid = initializeDARPGrid(randomInitPos, nx, ny, initial_positions, megaNodes, theta, shiftX,
                                              shiftY, DARPgrid)

                positions_DARPgrid = np.where(DARPgrid == 2)

                positions_DARPgrid = np.asarray(positions_DARPgrid).T
                positions = []
                for elem in positions_DARPgrid:
                    positions.append(grid[elem[0], elem[1]])

                self.darp_instance = DARP(nx, ny, notEqualPortions, positions, portions, obs_pos, visualization,
                                          DARPgrid, MaxIter=MaxIter, CCvariation=CCvariation,
                                          randomLevel=randomLevel, dcells=dcells,
                                          importance=importance)
                self.DARP_success, self.iterations = self.darp_instance.divideRegions()

            # self.darp_instance.success, DARPAssignmentMatrix = poly.update()
            if count == 5:
                print(
                    "DARP did not manage to find a solution for the given configuration!\n Try to alter one or more of:" +
                    "\n  - Scanning distance\n  - Number of drones \n  - Given area\n\n")
                exit()

        # // Calculate paths for all drones, for all modes (see below) and keep the paths with the minimum turns
        allDirectionsWaypoints = [[[] for _ in range(4)] for _ in range(self.darp_instance.droneNo)]

        mode_to_drone_turns = dict()

        for mode in range(4):
            MSTs = self.calculateMSTs(self.darp_instance.BinaryRobotRegions, self.darp_instance.droneNo, self.darp_instance.rows, self.darp_instance.cols, mode)
            AllRealPaths = []
            for r in range(self.darp_instance.droneNo):
                ct = CalculateTrajectories(self.darp_instance.rows, self.darp_instance.cols, MSTs[r])
                ct.initializeGraph(self.CalcRealBinaryReg(self.darp_instance.BinaryRobotRegions[r], self.darp_instance.rows, self.darp_instance.cols), True)
                ct.RemoveTheAppropriateEdges()
                ct.CalculatePathsSequence(4 * self.darp_instance.initial_positions[r][0] * self.darp_instance.cols + 2 * self.darp_instance.initial_positions[r][1])
                AllRealPaths.append(ct.PathSequence)

            TypesOfLines = np.zeros((self.darp_instance.rows * 2, self.darp_instance.cols * 2, 2))
            for r in range(self.darp_instance.droneNo):
                flag = False
                for connection in AllRealPaths[r]:
                    if flag:
                        if TypesOfLines[connection[0]][connection[1]][0] == 0:
                            indxadd1 = 0
                        else:
                            indxadd1 = 1

                        if TypesOfLines[connection[2]][connection[3]][0] == 0 and flag:
                            indxadd2 = 0
                        else:
                            indxadd2 = 1
                    else:
                        if not (TypesOfLines[connection[0]][connection[1]][0] == 0):
                            indxadd1 = 0
                        else:
                            indxadd1 = 1
                        if not (TypesOfLines[connection[2]][connection[3]][0] == 0 and flag):
                            indxadd2 = 0
                        else:
                            indxadd2 = 1

                    flag = True
                    if connection[0] == connection[2]:
                        if connection[1] > connection[3]:
                            TypesOfLines[connection[0]][connection[1]][indxadd1] = 2
                            TypesOfLines[connection[2]][connection[3]][indxadd2] = 3
                        else:
                            TypesOfLines[connection[0]][connection[1]][indxadd1] = 3
                            TypesOfLines[connection[2]][connection[3]][indxadd2] = 2

                    else:
                        if connection[0] > connection[2]:
                            TypesOfLines[connection[0]][connection[1]][indxadd1] = 1
                            TypesOfLines[connection[2]][connection[3]][indxadd2] = 4
                        else:
                            TypesOfLines[connection[0]][connection[1]][indxadd1] = 4
                            TypesOfLines[connection[2]][connection[3]][indxadd2] = 1


            # // Clockwise path
            for i in range(len(TypesOfLines)):
                for j in range(len(TypesOfLines[0])):
                    subNodes[i][j][2] = TypesOfLines[i][j][0]

            # // From TypesOfLines to lists of waypoints
            subCellsAssignment = np.zeros((2 * self.darp_instance.rows, 2 * self.darp_instance.cols))
            for i in range(self.darp_instance.rows):
                for j in range(self.darp_instance.cols):
                    subCellsAssignment[2 * i][2 * j] = self.darp_instance.A[i][j]
                    subCellsAssignment[2 * i + 1][2 * j] = self.darp_instance.A[i][j]
                    subCellsAssignment[2 * i][2 * j + 1] = self.darp_instance.A[i][j]
                    subCellsAssignment[2 * i + 1][2 * j + 1] = self.darp_instance.A[i][j]

            drone_turns = turns(AllRealPaths)
            drone_turns.count_turns()
            mode_to_drone_turns[mode] = drone_turns

            for k in range(self.darp_instance.droneNo):
                iWaypoints = []
                xInit = 2 * self.darp_instance.initial_positions[k][0]
                yInit = 2 * self.darp_instance.initial_positions[k][1]

                i = xInit
                j = yInit

                iWaypoints.append([subNodes[xInit][yInit][0], subNodes[xInit][yInit][1]])
                condition = True
                while condition:

                    prevState = subNodes[i][j][2]

                    if subNodes[i][j][2] == 1.0:
                        i -= 1
                    elif subNodes[i][j][2] == 2.0:
                        j -= 1
                    elif subNodes[i][j][2] == 3.0:
                        j += 1
                    elif subNodes[i][j][2] == 4.0:
                        i += 1

                    if prevState != subNodes[i][j][2] or (
                            subNodes[i][j][0] == iWaypoints[0][0] and subNodes[i][j][1] == iWaypoints[0][1]):
                        iWaypoints.append([subNodes[i][j][0], subNodes[i][j][1]])

                    condition = not (i == xInit and j == yInit)
                WP = rotate.rotateBackWaypoints(iWaypoints)

                # WP.append(iWaypoints)
                allDirectionsWaypoints[k][mode].append(WP)

        # image = visualize_paths(AllRealPaths, subCellsAssignment, self.darp_instance.droneNo, self.darp_instance.color)
        # image.visualize_paths(mode)

        # print("\nResults:\n")
        # for mode, val in mode_to_drone_turns.items():
        #     print(mode, val)

        # self.darp_instance.missionWaypointsNED = [[] for _ in range(droneNo)]
        # // Keep orientation with less turns
        for i in range(self.darp_instance.droneNo):
            ind = 0
            min = len(allDirectionsWaypoints[i][0][0])
            for j in range(1, 4):
                if len(allDirectionsWaypoints[i][j][0]) < min:
                    min = len(allDirectionsWaypoints[i][j][0])
                    ind = j
            self.missionWaypointsNED[i].append(allDirectionsWaypoints[i][ind][0])

        # for i in range(droneNo):
        #     print("NED_Coords:", len(self.darp_instance.missionWaypointsNED[i][0]))




    def CalcRealBinaryReg(self, BinaryRobotRegion, rows, cols):
        temp = np.zeros((2 * rows, 2 * cols))
        RealBinaryRobotRegion = np.zeros((2 * rows, 2 * cols), dtype=bool)
        for i in range(2 * rows):
            for j in range(2 * cols):
                temp[i, j] = BinaryRobotRegion[(int(i / 2))][(int(j / 2))]
                if temp[i, j] == 0:
                    RealBinaryRobotRegion[i, j] = False
                else:
                    RealBinaryRobotRegion[i, j] = True

        return RealBinaryRobotRegion

    def calculateMSTs(self, BinaryRobotRegions, droneNo, rows, cols, mode):
        MSTs = []
        for r in range(self.darp_instance.droneNo):
            k = Kruskal(rows, cols)
            k.initializeGraph(self.darp_instance.BinaryRobotRegions[r, :, :], True, mode)
            k.performKruskal()
            MSTs.append(k.mst)
        return MSTs

    def getWaypointsNED(self):
        return self.missionWaypointsNED