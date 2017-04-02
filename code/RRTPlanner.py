import numpy
import random
from RRTTree import RRTTree
import time

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
       

    def Plan(self, start_config, goal_config, epsilon = .001):
        start_time = time.time()
        tree = RRTTree(self.planning_env, start_config)
        plan = []

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        plan.append(start_config)
        plan.append(goal_config)

        currConfig = start_config;
        currID = tree.GetRootId();

        while (self.planning_env.ComputeDistance(currConfig,goal_config) > epsilon):
            if(random.random() < .9):
                newCurrConfig = self.planning_env.GenerateRandomConfiguration();
            else:
                newCurrConfig = goal_config;

            [nearID, nearConfig] = tree.GetNearestVertex(newCurrConfig);
            
            extension = self.planning_env.Extend(nearConfig, newCurrConfig)
            # print extension

            if (extension != None):
                currConfig = extension
                currID = tree.AddVertex(currConfig);
                tree.AddEdge(nearID, currID);


        goalID = tree.AddVertex(goal_config);
        tree.AddEdge(currID, goalID);

        currConfig = goal_config
        currID = goalID;
        while 1:
            currID = tree.edges[currID];
            currConfig = tree.vertices[currID];
            if (currID == tree.GetRootId()):
                break;
            else:
                plan.insert(1, currConfig);

        for config in plan:
            print "config = [%.2f, %.2f]" %(config[0], config[1])
        print("--- %s seconds ---" % (time.time() - start_time))
        print("--- %s path length ---" % len(plan))
        print("--- %s vertices ---" % len(tree.vertices))


        return plan;
