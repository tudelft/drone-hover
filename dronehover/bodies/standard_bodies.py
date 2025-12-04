import numpy as np
from numpy import sin, cos, pi
from numpy.linalg import norm as norm

from dronehover import prop_lib
                
# Standard x config quadcopter
class Quadcopter:
    def __init__(self, length):        

        self.props = [{"loc":[length*cos(1/4*pi), length*sin(1/4*pi), 0], "dir": [0, 0, -1, "ccw"], "propsize": 5},
                      {"loc":[length*cos(3/4*pi), length*sin(3/4*pi), 0], "dir": [0, 0, -1, "cw"], "propsize": 5},
                      {"loc":[length*cos(5/4*pi), length*sin(5/4*pi), 0], "dir": [0, 0, -1, "ccw"], "propsize": 5},
                      {"loc":[length*cos(7/4*pi), length*sin(7/4*pi), 0], "dir": [0, 0, -1, "cw"], "propsize": 5}]

        self.get_props()
        self.get_inertia()

    def get_inertia(self):
        controller_mass = 0.300 # based on 4S, 2200 mAh lipo
        beam_density = 1500*0.005*0.01 # kg/m, carbon fiber plates, 5mm thickness, 10mm width

        self.mass = controller_mass

        
        for prop in self.props:
            size = prop["propsize"]
            prop_mass = prop_lib[f"prop{size}"]["mass"]
            self.mass += beam_density*np.linalg.norm(np.array(prop["loc"])) # Add mass of carbon beam
            self.mass += prop_mass

        self.cg = np.zeros(3)
        for prop in self.props:
            prop_mass = prop_lib[f"prop{size}"]["mass"]
            beam_mass = beam_density*np.linalg.norm(np.array(prop["loc"]))
            self.cg += prop_mass/self.mass * np.array(prop["loc"])
            self.cg += beam_mass/self.mass * np.array(prop["loc"]) * 0.5

        self.Ix = norm(np.cross(np.array([1,0,0]),self.cg))**2 * controller_mass + 1/12 * controller_mass * (0.036**2 + 0.035**2)
        self.Iy = norm(np.cross(np.array([0,1,0]),self.cg))**2 * controller_mass + 1/12 * controller_mass * (0.105**2 + 0.035**2)
        self.Iz = norm(np.cross(np.array([0,0,1]),self.cg))**2 * controller_mass + 1/12 * controller_mass * (0.105**2 + 0.036**2)

        self.Ixy = 0
        self.Ixz = 0
        self.Iyz = 0

        for prop in self.props:
            size = prop["propsize"]
            prop_mass = prop_lib[f"prop{size}"]["mass"]
            pos = np.asarray(prop["loc"])
            r = pos - self.cg

            self.Ix += norm(np.cross(np.array([1,0,0]),r))**2  * prop_mass      # I due to motors
            self.Ix += 1/12 * norm(np.cross(np.array([1,0,0]), pos))**2 * beam_density*pos[0]    # I due to beam
            self.Ix += beam_density*norm(pos) * norm(np.cross(np.array([1,0,0]), (pos/2 - self.cg)))**2     # I due to parallel axis theorem

            self.Iy += norm(np.cross(np.array([0,1,0]),r))**2 * prop_mass
            self.Iy += 1/12 * norm(np.cross(np.array([0,1,0]), pos))**2 * beam_density*pos[1] 
            self.Iy += beam_density*norm(pos) * norm(np.cross(np.array([0,1,0]), (pos/2 - self.cg)))**2

            self.Iz += norm(np.cross(np.array([0,0,1]),r))**2 * prop_mass
            self.Iz += 1/12 * norm(np.cross(np.array([0,0,1]), pos))**2 * beam_density*norm(pos)
            self.Iz += beam_density*norm(pos) * norm(np.cross(np.array([0,0,1]), (pos/2 - self.cg)))**2

        self.cg = self.cg.tolist()

    def get_props(self):
        for i, prop in enumerate(self.props):
            size = prop["propsize"]
            self.props[i]["constants"] = prop_lib[f"prop{size}"]["constants"]
            self.props[i]["wmax"] = prop_lib[f"prop{size}"]["wmax"]

        

# Standard tricopter without tilt rotor
class Tricopter:
    def __init__(self, length):
        
        self.props = [{"loc":[length, 0, 0], "dir": [0, 0, -1, "ccw"], "propsize": 5},
                      {"loc":[length*cos(2/3*pi), length*sin(2/3*pi), 0], "dir": [0, 0, -1, "cw"], "propsize": 5},
                      {"loc":[length*cos(4/3*pi), length*sin(4/3*pi), 0], "dir": [0, 0, -1, "ccw"], "propsize": 5}]
        
        self.get_props()
        self.get_inertia()

    def get_inertia(self):
        controller_mass = 0.300 # based on 4S, 2200 mAh lipo
        beam_density = 1500*0.005*0.01 # kg/m, carbon fiber plates, 5mm thickness, 10mm width

        self.mass = controller_mass

        for prop in self.props:
            size = prop["propsize"]
            prop_mass = prop_lib[f"prop{size}"]["mass"]
            self.mass += beam_density*np.linalg.norm(np.array(prop["loc"])) # Add mass of carbon beam
            self.mass += prop_mass

        self.cg = np.zeros(3)
        for prop in self.props:
            prop_mass = prop_lib[f"prop{size}"]["mass"]
            beam_mass = beam_density*np.linalg.norm(np.array(prop["loc"]))
            self.cg += prop_mass/self.mass * np.array(prop["loc"])
            self.cg += beam_mass/self.mass * np.array(prop["loc"]) * 0.5

        self.Ix = norm(np.cross(np.array([1,0,0]),self.cg))**2 * controller_mass + 1/12 * controller_mass * (0.036**2 + 0.035**2)
        self.Iy = norm(np.cross(np.array([0,1,0]),self.cg))**2 * controller_mass + 1/12 * controller_mass * (0.105**2 + 0.035**2)
        self.Iz = norm(np.cross(np.array([0,0,1]),self.cg))**2 * controller_mass + 1/12 * controller_mass * (0.105**2 + 0.036**2)

        self.Ixy = 0
        self.Ixz = 0
        self.Iyz = 0

        for prop in self.props:
            size = prop["propsize"]
            prop_mass = prop_lib[f"prop{size}"]["mass"]
            pos = np.asarray(prop["loc"])
            r = pos - self.cg

            self.Ix += norm(np.cross(np.array([1,0,0]),r))**2  * prop_mass      # I due to motors
            self.Ix += 1/12 * norm(np.cross(np.array([1,0,0]), pos))**2 * beam_density*pos[0]    # I due to beam
            self.Ix += beam_density*norm(pos) * norm(np.cross(np.array([1,0,0]), (pos/2 - self.cg)))**2     # I due to parallel axis theorem

            self.Iy += norm(np.cross(np.array([0,1,0]),r))**2 * prop_mass
            self.Iy += 1/12 * norm(np.cross(np.array([0,1,0]), pos))**2 * beam_density*pos[1] 
            self.Iy += beam_density*norm(pos) * norm(np.cross(np.array([0,1,0]), (pos/2 - self.cg)))**2

            self.Iz += norm(np.cross(np.array([0,0,1]),r))**2 * prop_mass
            self.Iz += 1/12 * norm(np.cross(np.array([0,0,1]), pos))**2 * beam_density*norm(pos)
            self.Iz += beam_density*norm(pos) * norm(np.cross(np.array([0,0,1]), (pos/2 - self.cg)))**2

        self.cg = self.cg.tolist()

    def get_props(self):
        for i, prop in enumerate(self.props):
            size = prop["propsize"]
            self.props[i]["constants"] = prop_lib[f"prop{size}"]["constants"]
            self.props[i]["wmax"] = prop_lib[f"prop{size}"]["wmax"]
        

# Standard hexacopter  
class Hexacopter:
    def __init__(self, length):
        
        self.props = [{"loc":[length, 0, 0], "dir": [0, 0, -1, "ccw"], "propsize": 4},
                      {"loc":[length*cos(1/3*pi), length*sin(1/3*pi), 0], "dir": [0, 0, -1, "cw"], "propsize": 5},
                      {"loc":[length*cos(2/3*pi), length*sin(2/3*pi), 0], "dir": [0, 0, -1, "ccw"], "propsize": 5},
                      {"loc":[length*cos(pi), length*sin(pi), 0], "dir": [0, 0, -1, "cw"], "propsize": 5},
                      {"loc":[length*cos(4/3*pi), length*sin(4/3*pi), 0], "dir": [0, 0, -1, "ccw"], "propsize": 5},
                      {"loc":[length*cos(5/3*pi), length*sin(5/3*pi), 0], "dir": [0, 0, -1, "cw"], "propsize": 5}]
        
        self.get_props()
        self.get_inertia()

    def get_inertia(self):
        controller_mass = 0.300 # based on 4S, 2200 mAh lipo
        beam_density = 1500*0.005*0.01 # kg/m, carbon fiber plates, 5mm thickness, 10mm width

        self.mass = controller_mass

        for prop in self.props:
            size = prop["propsize"]
            prop_mass = prop_lib[f"prop{size}"]["mass"]
            self.mass += beam_density*np.linalg.norm(np.array(prop["loc"])) # Add mass of carbon beam
            self.mass += prop_mass

        self.cg = np.zeros(3)
        for prop in self.props:
            prop_mass = prop_lib[f"prop{size}"]["mass"]
            beam_mass = beam_density*np.linalg.norm(np.array(prop["loc"]))
            self.cg += prop_mass/self.mass * np.array(prop["loc"])
            self.cg += beam_mass/self.mass * np.array(prop["loc"]) * 0.5

        self.Ix = norm(np.cross(np.array([1,0,0]),self.cg))**2 * controller_mass + 1/12 * controller_mass * (0.036**2 + 0.035**2)
        self.Iy = norm(np.cross(np.array([0,1,0]),self.cg))**2 * controller_mass + 1/12 * controller_mass * (0.105**2 + 0.035**2)
        self.Iz = norm(np.cross(np.array([0,0,1]),self.cg))**2 * controller_mass + 1/12 * controller_mass * (0.105**2 + 0.036**2)

        self.Ixy = 0
        self.Ixz = 0
        self.Iyz = 0

        for prop in self.props:
            size = prop["propsize"]
            prop_mass = prop_lib[f"prop{size}"]["mass"]
            pos = np.asarray(prop["loc"])
            r = pos - self.cg

            self.Ix += norm(np.cross(np.array([1,0,0]),r))**2  * prop_mass      # I due to motors
            self.Ix += 1/12 * norm(np.cross(np.array([1,0,0]), pos))**2 * beam_density*pos[0]    # I due to beam
            self.Ix += beam_density*norm(pos) * norm(np.cross(np.array([1,0,0]), (pos/2 - self.cg)))**2     # I due to parallel axis theorem

            self.Iy += norm(np.cross(np.array([0,1,0]),r))**2 * prop_mass
            self.Iy += 1/12 * norm(np.cross(np.array([0,1,0]), pos))**2 * beam_density*pos[1]
            self.Iy += beam_density*norm(pos) * norm(np.cross(np.array([0,1,0]), (pos/2 - self.cg)))**2

            self.Iz += norm(np.cross(np.array([0,0,1]),r))**2 * prop_mass
            self.Iz += 1/12 * norm(np.cross(np.array([0,0,1]), pos))**2 * beam_density*norm(pos)
            self.Iz += beam_density*norm(pos) * norm(np.cross(np.array([0,0,1]), (pos/2 - self.cg)))**2

        self.cg = self.cg.tolist()

    def get_props(self):
        for i, prop in enumerate(self.props):
            size = prop["propsize"]
            self.props[i]["constants"] = prop_lib[f"prop{size}"]["constants"]
            self.props[i]["wmax"] = prop_lib[f"prop{size}"]["wmax"]

# Standard Octacopter
class Octacopter:
    def __init__(self, length):
        
        self.props = [{"loc":[length, 0, 0], "dir": [0, 0, -1, "ccw"], "propsize": 4},
                      {"loc":[length*cos(1/4*pi), length*sin(1/4*pi), 0], "dir": [0, 0, -1, "cw"], "propsize": 5},
                      {"loc":[length*cos(2/4*pi), length*sin(2/4*pi), 0], "dir": [0, 0, -1, "ccw"], "propsize": 5},
                      {"loc":[length*cos(3/4*pi), length*sin(3/4*pi), 0], "dir": [0, 0, -1, "cw"], "propsize": 5},
                      {"loc":[length*cos(pi), length*sin(pi), 0], "dir": [0, 0, -1, "ccw"], "propsize": 5},
                      {"loc":[length*cos(5/4*pi), length*sin(5/4*pi), 0], "dir": [0, 0, -1, "cw"], "propsize": 5},
                      {"loc":[length*cos(6/4*pi), length*sin(6/4*pi), 0], "dir": [0, 0, -1, "ccw"], "propsize": 5},
                      {"loc":[length*cos(7/4*pi), length*sin(7/4*pi), 0], "dir": [0, 0, -1, "cw"], "propsize": 5}]
        
        self.get_props()
        self.get_inertia()

    def get_inertia(self):
        controller_mass = 0.300 # based on 4S, 2200 mAh lipo
        beam_density = 1500*0.005*0.01 # kg/m, carbon fiber plates, 5mm thickness, 10mm width

        self.mass = controller_mass

        for prop in self.props:
            size = prop["propsize"]
            prop_mass = prop_lib[f"prop{size}"]["mass"]
            self.mass += beam_density*np.linalg.norm(np.array(prop["loc"])) # Add mass of carbon beam
            self.mass += prop_mass

        self.cg = np.zeros(3)
        for prop in self.props:
            prop_mass = prop_lib[f"prop{size}"]["mass"]
            beam_mass = beam_density*np.linalg.norm(np.array(prop["loc"]))
            self.cg += prop_mass/self.mass * np.array(prop["loc"])
            self.cg += beam_mass/self.mass * np.array(prop["loc"]) * 0.5

        self.Ix = norm(np.cross(np.array([1,0,0]),self.cg))**2 * controller_mass + 1/12 * controller_mass * (0.036**2 + 0.035**2)
        self.Iy = norm(np.cross(np.array([0,1,0]),self.cg))**2 * controller_mass + 1/12 * controller_mass * (0.105**2 + 0.035**2)
        self.Iz = norm(np.cross(np.array([0,0,1]),self.cg))**2 * controller_mass + 1/12 * controller_mass * (0.105**2 + 0.036**2)

        self.Ixy = 0
        self.Ixz = 0
        self.Iyz = 0

        for prop in self.props:
            size = prop["propsize"]
            prop_mass = prop_lib[f"prop{size}"]["mass"]
            pos = np.asarray(prop["loc"])
            r = pos - self.cg

            self.Ix += norm(np.cross(np.array([1,0,0]),r))**2  * prop_mass      # I due to motors
            self.Ix += 1/12 * norm(np.cross(np.array([1,0,0]), pos))**2 * beam_density*pos[0]    # I due to beam
            self.Ix += beam_density*norm(pos) * norm(np.cross(np.array([1,0,0]), (pos/2 - self.cg)))**2     # I due to parallel axis theorem

            self.Iy += norm(np.cross(np.array([0,1,0]),r))**2 * prop_mass
            self.Iy += 1/12 * norm(np.cross(np.array([0,1,0]), pos))**2 * beam_density*pos[1] 
            self.Iy += beam_density*norm(pos) * norm(np.cross(np.array([0,1,0]), (pos/2 - self.cg)))**2

            self.Iz += norm(np.cross(np.array([0,0,1]),r))**2 * prop_mass
            self.Iz += 1/12 * norm(np.cross(np.array([0,0,1]), pos))**2 * beam_density*norm(pos)
            self.Iz += beam_density*norm(pos) * norm(np.cross(np.array([0,0,1]), (pos/2 - self.cg)))**2

        self.cg = self.cg.tolist()

    def get_props(self):
        for i, prop in enumerate(self.props):
            size = prop["propsize"]
            self.props[i]["constants"] = prop_lib[f"prop{size}"]["constants"]
            self.props[i]["wmax"] = prop_lib[f"prop{size}"]["wmax"]