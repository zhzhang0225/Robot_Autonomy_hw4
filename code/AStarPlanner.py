import numpy as np
from Queue import PriorityQueue

class AStarPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()


    def Plan(self, start_config, goal_config):

        plan = []

        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        # plan.append(start_config)
        plan.append(goal_config)

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        init_state = self.planning_env.discrete_env.NodeIdToConfiguration(self.planning_env.discrete_env.ConfigurationToNodeId(start_config))
        goal_state = self.planning_env.discrete_env.NodeIdToConfiguration(self.planning_env.discrete_env.ConfigurationToNodeId(goal_config))

        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        print "Start state = ", start_config, ", int_state=", init_state
        print "Start ID = ", start_id

        # plan = np.empty((0, start_config.size()))
        goal_found = False
        frontier = PriorityQueue()
        states_visited = [start_id]
        came_from = {}
        came_from[start_id] = None
        plan_cost = {}
        plan_cost[start_id] = 0
        came_from_actions = {}

        if init_state == goal_state:
            return plan

        # Add initial neighbors to the frontier
        successors, succ_acts = self.planning_env.GetSuccessors(start_id)
        came_from_actions.update(succ_acts)
        for next_node in successors:
            # print next_node
            plan_cost[next_node] = plan_cost[start_id] + 1
            priority = plan_cost[next_node] + self.planning_env.ComputeHeuristicCost(next_node, goal_id)
            frontier.put((priority, next_node))
            came_from[next_node] = start_id

        while frontier:
            (cur_cost, cur_node) = frontier.get()
            states_visited.append(cur_node)
            # if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            #     self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(came_from[cur_node]), self.planning_env.discrete_env.NodeIdToConfiguration(cur_node))

            if cur_node == goal_id:
                goal_found = True
                print "Goal Found!"
                break

            successors = self.planning_env.GetSuccessors(cur_node)
            successors, succ_acts = self.planning_env.GetSuccessors(start_id)
            came_from_actions.update(succ_acts)
            for next_node in successors:
                new_cost = plan_cost[cur_node] + 1
                # self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(cur_node), self.planning_env.discrete_env.NodeIdToConfiguration(next_node))
                if next_node not in plan_cost  or  new_cost < plan_cost[next_node]:
                    plan_cost[next_node] = new_cost
                    priority = new_cost + self.planning_env.ComputeHeuristicCost(next_node, goal_id)
                    frontier.put((priority, next_node))
                    came_from[next_node] = cur_node

        while goal_found  and  came_from[cur_node] is not None:
            # parent = came_from[cur_node]
            # plan = np.append([self.planning_env.discrete_env.NodeIdToConfiguration(cur_node)], plan, axis=0)
            parent, action = came_from_actions[cur_node]
            plan = np.append([action], plan, axis=0)
            cur_node = parent

        print 'NUM OF EXPANDED NODES: ' + repr(len(states_visited))
        
        return plan
