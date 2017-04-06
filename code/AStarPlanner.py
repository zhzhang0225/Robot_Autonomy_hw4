import numpy as np
import time
from Queue import PriorityQueue
from SimpleEnvironment import Action

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

        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        print "Start pose = ", start_config
        print "Goal pose", goal_config
        print "Start ID = ", start_id, "Goal ID", goal_id
        print "Running A* search"
        start_time = time.time()

        goal_found = False
        frontier = PriorityQueue()
        states_visited = [start_id]
        came_from = {}
        came_from[start_id] = None
        plan_cost = {}
        plan_cost[start_id] = 0
        came_from_actions = {}

        if start_id == goal_id:
            return plan

        # Add initial neighbors to the frontier
        successors, succ_acts = self.planning_env.GetSuccessors(start_id)
        # print successors
        # came_from_actions.update(succ_acts)
        for next_node in successors:
            # print next_node
            plan_cost[next_node] = plan_cost[start_id] + 1
            priority = plan_cost[next_node] + self.planning_env.ComputeHeuristicCost(next_node, goal_id)
            frontier.put((priority, next_node))
            came_from[next_node] = start_id
            came_from_actions[next_node] = succ_acts[next_node]

        while frontier:
            (cur_cost, cur_node) = frontier.get()
            states_visited.append(cur_node)

            if cur_node == goal_id:
                goal_found = True
                print "Goal Found!"
                break

            successors, succ_acts = self.planning_env.GetSuccessors(cur_node)
            # print successors
            # came_from_actions.update(succ_acts)
            for next_node in successors:
                # print next_node
                if next_node == start_id:
                    continue
                new_cost = plan_cost[cur_node] + 1
                if (next_node not in came_from_actions) or  (new_cost < plan_cost[next_node]):
                    plan_cost[next_node] = new_cost
                    priority = new_cost + self.planning_env.ComputeHeuristicCost(next_node, goal_id)
                    frontier.put((priority, next_node))
                    came_from[next_node] = cur_node
                    came_from_actions[next_node] = succ_acts[next_node]

        print 'NUM OF EXPANDED NODES: ' + repr(len(states_visited))

        if goal_found:
            while cur_node != start_id or came_from[cur_node] is not None:
                parent, action = came_from_actions[cur_node]
                plan = [action] + plan
                cur_node = parent
        else:
            print "Path not found"

        planning_time = time.time() - start_time
        print "Planning time = ",planning_time, "secs"
        print "Returning path"

        return plan
