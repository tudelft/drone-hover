import numpy as np
from numpy.linalg import norm as norm
from dronehover import prop_lib

class Custombody:
    def __init__(self, props, mountpoints=None, mass=None, cg=None, Ix=None, Iy=None, Iz=None, Ixy=None, Ixz=None, Iyz=None):
        """Class for custom drone bodies

        Args:
            mass (float): Mass of the drone
            Ix (float): Moment of inertia about x axis
            Iy (float): Moment of inertia about y axis
            Iz (float): Moment of inertia about z axis
            Ixy (float): Products of inertia (x-y)
            Ixz (float): Products of inertia (x-z)
            Iyz (float): Products of inertia (y-z)
            props (dict): Propeller properties
        """        
        
        self.props = props
        self.mountpoints = mountpoints

        if self.mountpoints is None:
            self.mountpoints = [np.array([0, 0, 0])] * len(self.props)

        if self.mountpoints is not None and len(self.props) != len(self.mountpoints):
            raise Exception("Number of mounting points has to be equal to number of arms!")
        

        self.get_props()

        if mass == None:
            self.get_inertia()
        else:
            self.mass = mass
            self.cg = cg
            self.Ix = Ix
            self.Iy = Iy
            self.Iz = Iz
            self.Ixy = Ixy
            self.Ixz = Ixz
            self.Iyz = Iyz

    def get_inertia(self):
        controller_mass = 0.300 # based on 4S, 2200 mAh lipo
        beam_density = 1500*0.005*0.01 # kg/m, carbon fiber plates, 5mm thickness, 10mm width

        self.mass = controller_mass

        for prop, mountpoint in zip(self.props, self.mountpoints):
            size = prop["propsize"]
            prop_mass = prop_lib[f"prop{size}"]["mass"]
            self.mass += beam_density*norm(np.array(prop["loc"]) - mountpoint) # Add mass of carbon beam
            self.mass += prop_mass

        self.cg = np.zeros(3)
        for prop, mountpoint in zip(self.props, self.mountpoints):
            prop_mass = prop_lib[f"prop{size}"]["mass"]
            beam_mass = beam_density*norm(np.array(prop["loc"]) - mountpoint)
            self.cg += prop_mass/self.mass * np.array(prop["loc"])
            self.cg += beam_mass/self.mass * (np.array(prop["loc"]) + mountpoint) * 0.5

        self.Ix = norm(np.cross(np.array([1,0,0]),self.cg))**2 * controller_mass + 1/12 * controller_mass * (0.036**2 + 0.035**2)
        self.Iy = norm(np.cross(np.array([0,1,0]),self.cg))**2 * controller_mass + 1/12 * controller_mass * (0.105**2 + 0.035**2)
        self.Iz = norm(np.cross(np.array([0,0,1]),self.cg))**2 * controller_mass + 1/12 * controller_mass * (0.105**2 + 0.036**2)
        self.Ixy = 0
        self.Ixz = 0
        self.Iyz = 0

        for prop, mountpoint in zip(self.props, self.mountpoints):
            size = prop["propsize"]
            prop_mass = prop_lib[f"prop{size}"]["mass"]
            pos = np.asarray(prop["loc"])
            arm_length = norm(pos - mountpoint)
            arm_midpoint = (pos + mountpoint)/2
            r = pos - self.cg

            self.Ix += norm(np.cross(np.array([1,0,0]),r))**2  * prop_mass      # I due to motors
            self.Ix += 1/12 * norm(np.cross(np.array([1,0,0]), pos))**2 * beam_density*arm_length    # I due to beam
            self.Ix += beam_density*arm_length  * norm(np.cross(np.array([1,0,0]), (arm_midpoint - self.cg)))**2     # I due to parallel axis theorem

            self.Iy += norm(np.cross(np.array([0,1,0]),r))**2 * prop_mass
            self.Iy += 1/12 * norm(np.cross(np.array([0,1,0]), pos))**2 * beam_density*arm_length 
            self.Iy += beam_density*arm_length  * norm(np.cross(np.array([0,1,0]), (arm_midpoint - self.cg)))**2

            self.Iz += norm(np.cross(np.array([0,0,1]),r))**2 * prop_mass
            self.Iz += 1/12 * norm(np.cross(np.array([0,0,1]), pos))**2 * beam_density*arm_length 
            self.Iz += beam_density*arm_length  * norm(np.cross(np.array([0,0,1]), (arm_midpoint - self.cg)))**2

            self.Ixy -= r[0]*r[1]*prop_mass
            self.Ixy -= 1/12 * pos[0]*pos[1]*beam_density*arm_length 
            self.Ixy -= beam_density*arm_length  * (arm_midpoint[0]/2 - self.cg[0])*(arm_midpoint[1]/2 - self.cg[1])
            
            self.Ixz -= r[0]*r[2]*prop_mass
            self.Ixz -= 1/12 * pos[0]*pos[2]*beam_density*arm_length 
            self.Ixz -= beam_density*arm_length  * (arm_midpoint[0]/2 - self.cg[0])*(arm_midpoint[2]/2 - self.cg[2])
            
            self.Iyz -= r[1]*r[2]*prop_mass
            self.Iyz -= 1/12 * pos[1]*pos[2]*beam_density*arm_length 
            self.Iyz -= beam_density*arm_length  * (arm_midpoint[1]/2 - self.cg[1])*(arm_midpoint[2]/2 - self.cg[2])

        self.cg = self.cg.tolist()

    def get_props(self):
        for i, prop in enumerate(self.props):
            size = prop["propsize"]
            self.props[i]["constants"] = prop_lib[f"prop{size}"]["constants"]
            self.props[i]["wmax"] = prop_lib[f"prop{size}"]["wmax"]
        