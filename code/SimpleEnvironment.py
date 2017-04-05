import numpy, openravepy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class Control(object):
    def __init__(self, omega_left, omega_right, duration):
        self.ul = omega_left
        self.ur = omega_right
        self.dt = duration

class Action(object):
    def __init__(self, control, footprint):
        self.control = control
        self.footprint = footprint

class SimpleEnvironment(object):

    def __init__(self, herb, resolution):
        self.herb = herb
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5., -numpy.pi], [5., 5., numpy.pi]]
        lower_limits, upper_limits = self.boundary_limits
        self.discrete_env = DiscreteEnvironment(resolution, lower_limits, upper_limits)

        self.resolution = resolution
        self.ConstructActions()

    def GenerateFootprintFromControl(self, start_config, control, stepsize=0.01):

        # Extract the elements of the control
        ul = control.ul
        ur = control.ur
        dt = control.dt

        start_config = numpy.array(start_config)
        # Initialize the footprint
        config = start_config.copy()
        footprint = [numpy.array([0., 0., config[2]])]
        timecount = 0.0
        while timecount < dt:
            # Generate the velocities based on the forward model
            xdot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.cos(config[2])
            ydot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.sin(config[2])
            tdot = self.herb.wheel_radius * (-ul + ur) / self.herb.wheel_distance

            # Feed forward the velocities
            if timecount + stepsize > dt:
                stepsize = dt - timecount
            config = config + stepsize*numpy.array([xdot, ydot, tdot])
            if config[2] > numpy.pi:
                config[2] -= 2.*numpy.pi
            if config[2] < -numpy.pi:
                config[2] += 2.*numpy.pi

            footprint_config = config.copy()
            footprint_config[:2] -= start_config[:2]
            footprint.append(footprint_config)

            timecount += stepsize

        # Add one more config that snaps the last point in the footprint to the center of the cell
        nid = self.discrete_env.ConfigurationToNodeId(config)
        snapped_config = self.discrete_env.NodeIdToConfiguration(nid)
        snapped_config[:2] -= start_config[:2]
        footprint.append(snapped_config)
        # print "Last footprint config = ", config
        
        return footprint

    def PlotActionFootprints(self, idx):

        actions = self.actions[idx]
        fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])

        for action in actions:
            xpoints = [config[0] for config in action.footprint]
            ypoints = [config[1] for config in action.footprint]
            pl.plot(xpoints, ypoints, 'k')

        pl.ion()
        pl.show()



    def ConstructActions(self):
        print "Constructing actions"

        # Actions is a dictionary that maps orientation of the robot to
        #  an action set
        self.actions = dict()

        wc = [0., 0., 0.]
        grid_coordinate = self.discrete_env.ConfigurationToGridCoord(wc)

        # Iterate through each possible starting orientation
        for idx in range(int(self.discrete_env.num_cells[2])):
            self.actions[idx] = []
            grid_coordinate[2] = idx
            start_config = self.discrete_env.GridCoordToConfiguration(grid_coordinate)

            # TODO: Here you will construct a set of actions
            #  to be used during the planning process
            #

            # Control set for grid resolution 0.1
            pi = numpy.pi
            r = self.herb.wheel_radius
            L = self.herb.wheel_distance

            theta = [0.75*pi, 0.5*pi, 0.25*pi, -0.25*pi, -0.5*pi, -0.75*pi]
            point_rot = [[-1,1,abs(0.5*th*L/r)] for th in theta[0:4]] + [[1,-1,abs(0.5*th*L/r)] for th in theta[4:]]
            # point_rot = []
            # control_set = numpy.array([[1, 1, 0.4/r],[1, 1, 0.2/r],[-1, -1, 0.2/r], [0, 1, (pi/4)*L/r], [1, 0, (pi/4)*L/r]] + point_rot)
            control_set = numpy.array([[1, 1, 0.5]] + point_rot)

            for c in control_set:
                ctrl = Control(c[0], c[1], c[2])
                footprint = self.GenerateFootprintFromControl(start_config, ctrl, 0.001)
                print "idx", idx, "Footprint: ", footprint[0], footprint[-1]
                act = Action(ctrl, footprint)
                self.actions[idx].append(act)

        self.PlotActionFootprints(4)

    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodes

        successor_actions = dict()
        parent_config = numpy.array(self.discrete_env.NodeIdToConfiguration(node_id))
        parent_coord = self.discrete_env.NodeIdToGridCoord(node_id)
        # print parent_coord
        possible_actions = self.actions[parent_coord[2]]

        for act in possible_actions:
            fp = act.footprint
            # child_node_id = self.discrete_env.ConfigurationToNodeId(fp[-1])
            child_node_id = self.discrete_env.ConfigurationToNodeId(numpy.append(parent_config[:2],0)+fp[-1])
            with self.robot:
                config = self.discrete_env.NodeIdToConfiguration(child_node_id)
                self.herb.SetCurrentConfiguration(config)
                if (self.robot.GetEnv().CheckCollision(self.robot)) is False:
                    successors.append(child_node_id)
                    successor_actions[child_node_id] = (node_id, act) # Parent-Action pair

        return successors, successor_actions

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that
        # computes the distance between the configurations given
        # by the two node ids
        d = 0
        thetadiff = 0
        coordStart = numpy.array(self.discrete_env.NodeIdToConfiguration(start_id))
        coordEnd = numpy.array(self.discrete_env.NodeIdToConfiguration(end_id))
        # d = numpy.sqrt((coordStart[0] - coordEnd[0])**2+(coordStart[1] - coordEnd[1])**2)
        d = numpy.linalg.norm(coordEnd[:2] - coordStart[:2])
        thetadiff = abs(coordStart[2] - coordEnd[2])
        if d > 1.5:
            dist = d
        else:
            dist = d + thetadiff*5

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):

        cost = 0

        # TODO: Here you will implement a function that
        # computes the heuristic cost between the configurations
        # given by the two node ids
        cost = self.ComputeDistance(start_id, goal_id)

        return cost
