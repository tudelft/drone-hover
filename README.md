# dronehover

Compute the hovering capabilities of drones with arbitrary configurations.

**Updates**:

[24 September 2024]
1. `Custombody` and standard body classes is now able to compute mass, inertia and C.G. location given propeller locations. To enable this, simply leave out `mass, cg, Ix, Iy, Iz, Ixy, Ixz, Iyz` when calling the class.
2. Automatic computation can be overridden by defining the mass, inertia and C.G. properties when calling the class.
3. Automatic computation override only available on `Custombody`. Standard bodies does not have this override feature yet.
4. See the section on Defining drone bodies for more details.


[28 May 2024]
1. Packaged library - dronehover
2. Changed definition of propeller direction to `"ccw"` or `"cw"`.

[20 May 2024]
1. Motor commands are now proportionate to the square of motor RPM.
2. Propeller forces are now defined using force and moment constants rather than maximum thrust and moments.

## Installation
Create a virtual environment and run `pip install .`

Run example `python3 examples/hover_quad.py` to test.


## Defining drone bodies
The drone has a body-fixed coordinate system which follows the North-East-Down (NED) convention ($x$ axis pointing to the front, $y$ axis pointing to the right, and $z$ axis pointing down). Propeller positions and directions are defined using this coordinate system. The C.G. of the drone may not necessarily coincide with the origin of the coordinate system, and needs to be defined/computed.

Drones are defined using classes, and require propeller properties as class variables.

Propeller properties are defined using dictionaries, and require the following keywords:

`"loc":[x,y,z]`: List that defines the $(x,y,z)$ coordinates of the propeller in body-fixed axis.

`"dir":[x,y,z,r]`: List that defines the direction of thrust and rotation for the propeller. Includes 4 numbers, first 3 numbers are the $(x,y,z)$ vector defining the thrust direction, and the entry indicates counterclockwise (ccw) or clockwise (cw) rotation (as viewed from the top of the propeller). Direction $(x,y,z)$ does not need to be unit vector as the optimizer will scale it automatically.

`"propsize`: Size of propeller in inches. Propeller constants and motor mass extracted from a propeller library.

Example: 

    props = [{"loc":[length*cos(1/4*pi), length*sin(1/4*pi), 0], "dir": [0, 0, -1, "ccw"], "propsize": 4},
             {"loc":[length*cos(3/4*pi), length*sin(3/4*pi), 0], "dir": [0, 0, -1, "cw"], "propsize": 4},
             {"loc":[length*cos(5/4*pi), length*sin(5/4*pi), 0], "dir": [0, 0, -1, "ccw"], "propsize": 4},
             {"loc":[length*cos(7/4*pi), length*sin(7/4*pi), 0], "dir": [0, 0, -1, "cw"], "propsize": 4}]

There are 2 ways to define the drone body.
1. Creating a class that follows the format as seen in `drone_hover.standard_bodies`.
2. Call the `Custombody` object

When using `Custombody`, inertia properties are optional parameters. Inertia properties of the drones are computed automatically. If inertia properties are defined, the automatic computation will be overridden. 

Inertia properties are the mass and moment of inertia of the drone, and are defined using variables. C.G. location is also defined as a list.

Example:

    from drone_hover.custom_bodies import Custombody

    drone = Custombody(props)   # Automatic computation of inertia properties

    drone = Custombody(props, mass, cg, Ix, Iy, Iz, Ixy, Ixz, Iyz)      # User defined inertia properties

Custombody assumes that the root of each arm is at (0, 0, 0). If specific mounting point is required, the mounting point should be defined as a list of array, and added to the Custombody function. It is important to note that every propeller in `props` require its own mounting point. Hence, the length of `props` and `mounting_points` have to be equal.

Example:

    from drone_hover.custom_bodies import Custombody

    mounting_points = [np.array(0, 0, 1)] * 4 # Assuming 4 arms

    drone = Custombody(props)   # Automatic computation of inertia properties


## Propeller Library

The propeller constants are compiled into a library (dictionary) and can be found in the `__init__.py` file.

## Propeller Commands

This code utilizes 2 levels of mapping for the propeller commands.
1. The propeller angular velocity is normalized such that $f:\omega \rightarrow \hat{\omega}$, where $\omega \in [0.02\omega_{max}, \omega_{max}]$ and $\hat{\omega} \in [0.02, 1]$. The factor 0.02 is arbitrarily selected to be the idling speed of the propeller. This mapping embeds the propeller information into the propeller effectiveness matrices. 
2. When giving actual commands to the drone, it is more convinient to give a command $u \in [0,1]$. Hence, a second map $g:\hat{\omega} \rightarrow u$ is defined.

This is done to ensure that the equations remain linear (to $\omega^2$). Optimization will be performed using $\hat{\omega}$, while actual controls will be performed using $u$.

## Optimization

Optimization is performed using `scipy.optimize.minimize` module, using the SLSQP algorithm.

## Current capabilities: 

- Determine whether a drone can hover statically, while spinning, or not able to hover at all.
- Works on drones with arbitrary configurations (e.g. number of propellers, location of propellers, direction of propellers, etc.).
- Computes the input commands for most efficient hover.
- Computes the maximum thrust to weight ratio at hovering configuration
- Computes the cost of most efficient hover.

## Limitations:

- Spinning hover optimization does not work when force is aligned with torque for all values of input commands. SLSQP require constraints to be twice differentiable. To consider alternative optimization algorithms.
