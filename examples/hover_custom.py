import numpy as np
from numpy import sin, cos, pi

from dronehover.bodies.custom_bodies import Custombody

from dronehover.optimization import Hover

if __name__ == "__main__":
    # Import drone body
    props = [{"loc":[0.110*cos(1/4*pi), 0.110*sin(1/4*pi), 0], "dir": [0, 0, -1, "ccw"], "propsize": 5},
                      {"loc":[0.110*cos(3/4*pi), 0.110*sin(3/4*pi), 0], "dir": [0, 0, -1, "cw"], "propsize": 5},
                      {"loc":[0.110*cos(5/4*pi), 0.110*sin(5/4*pi), 0], "dir": [0, 0, -1, "ccw"], "propsize": 5},
                      {"loc":[0.110*cos(7/4*pi), 0.110*sin(7/4*pi), 0], "dir": [0, 0, -1, "cw"], "propsize": 5}]
    
    
    drone = Custombody(props, [np.array([0,0,0])]*4)         # Automatic inertia computation
    
    # drone = Custombody(props,         # Override automatic computation
    #                    mass=1, 
    #                    cg=[0,0,0], 
    #                    Ix=0.01, 
    #                    Iy=0.01, 
    #                    Iz=0.05, 
    #                    Ixy=0, 
    #                    Ixz=0, 
    #                    Iyz=0)
    
    

    # Define hovering optimizer for drone
    sim = Hover(drone)

    # print(sim.Bm)
    
    # Compute most efficient hover
    sim.compute_hover(verbose=True)
