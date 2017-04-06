import numpy
import math

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = numpy.ceil((upper_limits[idx] - lower_limits[idx])/resolution[idx])


    def ConfigurationToNodeId(self, config):

        # TODO:
        # This function maps a node configuration in full configuration
        # space to a node in discrete space
        #
        node_id = self.GridCoordToNodeId(self.ConfigurationToGridCoord(config))
        return node_id

    def NodeIdToConfiguration(self, nid):

        # TODO:
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        config = self.GridCoordToConfiguration(self.NodeIdToGridCoord(nid))
        return config

    def ConfigurationToGridCoord(self, config):

        # TODO:
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        coord = [0] * self.dimension
        for idx in range(self.dimension):
            if config[idx]==self.upper_limits[idx]:
               coord[idx]=int(self.num_cells[idx]-1)
            else:
               coord[idx]=int(math.floor((config[idx]-self.lower_limits[idx])/self.resolution[idx]))
        return coord

    def GridCoordToConfiguration(self, coord):

        # TODO:
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        config = [0] * self.dimension
        for idx in range(self.dimension):
            config[idx]=(self.lower_limits[idx])+(self.resolution[idx]*coord[idx])+(self.resolution[idx]/2.0)
        return config

    def GridCoordToNodeId(self,coord):

        # TODO:
        # This function maps a grid coordinate to the associated
        # node id
        #node_id = numpy.ravel_multi_index((coord),dims=(self.num_cells),order='F')
        node_id = 0
        multiple = 1

        for idx in range(self.dimension):
            node_id += coord[idx] * multiple
            multiple = multiple * self.num_cells[idx]

        return node_id

    def NodeIdToGridCoord(self, node_id):

        # TODO:
        # This function maps a node id to the associated
        # grid coordinate
        coord = [0] * self.dimension
        #coord=numpy.unravel_index(node_id,dims=(self.num_cells),order='F')
        remaining = node_id
        product = numpy.prod(numpy.array(self.num_cells))
        for idx in xrange(self.dimension-1, -1, -1):
            product  = (product/self.num_cells[idx])
            coord[idx] = int(remaining/product)
            remaining = remaining%product

        return coord


        
