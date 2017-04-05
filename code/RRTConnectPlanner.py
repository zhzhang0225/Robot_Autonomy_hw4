import numpy
from RRTTree import RRTTree
import copy

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
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
        temp_plan = []        
        temp_start_config = copy.deepcopy(start_config)
        temp_plan.append(temp_start_config) 
        back_trace = []
        back_trace.append(0)              
        while 1:   
            if self.planning_env.Extend(temp_start_config, goal_config) != None:
                break

<<<<<<< HEAD
        print "startConfig = [%.2f, %.2f]" %(start_config[0], start_config[1])
        print "goalConfig = [%.2f, %.2f]" %(goal_config[0], goal_config[1])
        disc = True;
        while (disc):
            if(random.random() < 0.95):
                desConfig = self.planning_env.GenerateRandomConfiguration();
            else:
                desConfig = goal_config;
            expansions+=1
            [nearID, nearConfig] = ftree.GetNearestVertex(desConfig);            
            extension = self.planning_env.Extend(nearConfig, desConfig)
            if (extension != None):
                extIDf = ftree.AddVertex(extension);
                ftree.AddEdge(nearID, extIDf);
                #self.planning_env.PlotEdge(nearConfig, extension);
            
            #if(numpy.array_equal(extension, goal_config)):
            #    disc = False

            if(random.random() < 0.95):
                desConfig = self.planning_env.GenerateRandomConfiguration();
            else:
                desConfig = start_config;
            expansions+=1
            [nearID, nearConfig] = rtree.GetNearestVertex(desConfig);            
            extension = self.planning_env.Extend(nearConfig, desConfig)
            if (extension != None):
                extIDr = rtree.AddVertex(extension);
                rtree.AddEdge(nearID, extIDr);
                #self.planning_env.PlotEdge(nearConfig, extension);
            
            #if(numpy.array_equal(extension, start_config)):
            #    disc = False

            for i in range(len(ftree.vertices)):
                if (numpy.linalg.norm(numpy.array(desConfig) - numpy.array(ftree.vertices[i])) < 5):
                    lastlink = self.planning_env.Extend(extension, ftree.vertices[i])
                    if (numpy.array_equal(lastlink,ftree.vertices[i])):
                        lastIDr = rtree.AddVertex(lastlink);
                        rtree.AddEdge(extIDr, lastIDr);
                        #self.planning_env.PlotEdge(extension, lastlink);
                        disc = False
                        extIDf = i
                        expansions+=1
                        break
        lastlink_cp = lastlink
        rplan.insert(0,goal_config)
        while 1:
            if (extIDr == rtree.GetRootId()):
                break;
            else:
                rplan.insert(1, extension);
            extIDr = rtree.edges[extIDr];
            extension = rtree.vertices[extIDr];
=======
            end_config = self.planning_env.GenerateRandomConfiguration()
            neighbor = []
            if len(temp_plan) > 1:
                for i in range(len(temp_plan)):
                    neighbor.append(self.planning_env.ComputeDistance(temp_plan[i], end_config)) 
                z = neighbor.index(min(neighbor))      
                temp_start_config = copy.deepcopy(temp_plan[z])
                #back_trace.append(z)
                del neighbor      
            if self.planning_env.Extend(temp_start_config, end_config) != None:
                if len(temp_plan) > 1:
                    back_trace.append(z)
                temp_plan.append(end_config)    
                temp_start_config = copy.deepcopy(end_config)

>>>>>>> 9e472ca8435622c364567a9a252f652aba77eaa6

        y = len(temp_plan)-1        
        while y != 0 :        
            plan.insert(1,temp_plan[y])
            y = copy.deepcopy(back_trace[y-1])  
#--------- Visualize for PR2 -------------------------
        if len(start_config) == 2:        
            for i in range(len(back_trace)):
                self.planning_env.PlotEdge(temp_plan[back_trace[i]],temp_plan[i+1])
            self.planning_env.PlotEdge(temp_plan[-1],goal_config)
#--------- For Calculating Path Length ----------------

<<<<<<< HEAD
        plan.insert(0,start_config)
        while 1:
            if (extIDf == ftree.GetRootId()):
                break;
            else:
                plan.insert(1, lastlink);
            extIDf = ftree.edges[extIDf];
            lastlink = ftree.vertices[extIDf];


        for config in plan:
            print "fconfig = [%.2f, %.2f]" %(config[0], config[1])
        print("--- %s seconds ---" % (time.time() - start_time))
        print "Total expansions = %d" %(expansions)
        return plan
=======
        dist = 0
        for i in range(len(plan)-2):
            dist = dist + self.planning_env.ComputeDistance(plan[i],plan[i+1])    
        print dist
        
return plan
>>>>>>> 9e472ca8435622c364567a9a252f652aba77eaa6
