"""
You have two options to run Coverage Path Planning(CPP) task:
"""

"""
    1. Run CPP on a square grid: real_world = False (Please refer to https://github.com/alice-st/DARP)

        All of the parameters are set in main():

        To modify the Grid Dimensions, use:
          -grid x y

    where x, y are the desired rows and columns of the Grid respectively (default: 10, 10).

        To modify the number of Robots and their Initial Positions, use:
        -in_pos a b c

    where a, b, c, are the cells' numbers in the Grid (default: 0, 3, 9) (row=0,column=0 --> cell=0, row=0,column=1 --> cell=1 etc.)

        To assign different portions to each Robot (not Equal), use:
        -nep -portions p_a p_b p_c

    where p_a p_b p_c are the portions assigned to Robots a, b and c respectively. Their sum should be equal to 1. (default: 0.2, 0.3, 0.5)
    If -nep is activated (set to True), the algorithm runs for not equal territories with 20%, 30% and 50% coverage per robot. Otherwise, the algorithm runs for equal territories with 33,33% coverage per robot.

        To use different positions for the obstacles in the Grid, use:
         -obs_pos o1 o2 o3
    where o1 o2 and o3 are the positions of the obstacles in the Grid. Obstacle positions should not overlap with Robots' initial positions. (default: 5, 6, 7) (row=0,column=0 --> cell=0, row=0,column=1 --> cell=1 etc.)

        To visualize the results, use:
        -vis True


    2. Run CPP on real world data: real_world = True (Define parameters on input_variables.json)

    The algorithm receives (from .json) as input the following:
    
    -   The number of robots/vehicles
    -   The desired sidelap in % 
    -   A polygon Region of Interest (ROI), formatted in WGS84 coordinate system
    -   A set of obstacles (polygons formatted in WGS84 coordinate system) inside the ROI (optional)
    -   A boolean variable named pathsStrictlyInPoly, to select mode between (paths strictly in poly/better coverage)
    -   The initial positions of the vehicles (optional - if not provided, random will be used instead | Note that the number of the initial positions should always be the same as the number of robots/vehicles)
    -   The desired percentages for proportional area allocation (optional - if not provided, equal will be used instead | Note that the number of the percentages should always be the same as the number of robots/vehicles and their sum should be 1)
        
    As an output, the algorithm provides set of waypoints (path), for each vehicle involved in the mission, in order to cooperatively completely cover the ROI.

      
########################################################################################################################
--------------------------- Dynamic spawn and Autonomous simulation of drones ------------------------------------------
########################################################################################################################
  
         - Option 2 is also compatible with the AirSim simulator (https://microsoft.github.io/AirSim/)

"""

# So .. what is your option ?
real_world = True
AirSim = True  # AirSim can be True ONLY if real_world = True

# IF AirSim is True:
distance_threshold = 4  # If AirSim is True, define the distance threshold from Turn-Wps (Usually in range [0.1,2])
velocity = 3  # in m/s
corner_radius = velocity + 3
