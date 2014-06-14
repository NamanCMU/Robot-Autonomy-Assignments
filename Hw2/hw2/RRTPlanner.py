import numpy
from RRTTree import RRTTree

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
        
        plan_temp = []
        plan_temp.append(start_config)
        tree.AddVertex(start_config)  
        plan_parent = []
        plan_parent.append(start_config);

        
        print "SC: ", start_config, " EC: ", goal_config
       
        count = 0;    
        self.env = self.planning_env.robot.GetEnv()
        robot_name = self.env.GetBodies()[0].GetName()
        
        with self.env:
            with self.planning_env.robot.CreateRobotStateSaver():
                while(1):
                    if count%10 == 0: # After every 10th iteration, take the goal as sample
                        sample_config = goal_config
                    else:
                        sample_config = self.planning_env.GenerateRandomConfiguration()

                    vid1, nearest_vertex = tree.GetNearestVertex(sample_config) # Finding the nearest vertex
                    sample_extend_config = self.planning_env.Extend(nearest_vertex,sample_config) # Checking for collision
            
                    plan_parent.append(nearest_vertex)
                    plan_temp.append(sample_extend_config)
                    vid2 = tree.AddVertex(sample_extend_config)  
                    tree.AddEdge(vid1,vid2)
                    
                    if (robot_name != 'Herb2'): # Visualize only for PR2
                        self.planning_env.PlotEdge(nearest_vertex,plan_temp[-1])

                    if self.planning_env.ComputeDistance(plan_temp[-1],goal_config) < 0.01: # Break Condition
                        break;
            
                    count = count + 1;
        
                parent = plan_parent[-1]
                plan.append(goal_config)
                plan.append(parent)

                plan_temp_list = [list(x) for x in plan_temp]

                while(1):
                    parent_index = plan_temp_list.index(list(parent))
                    parent = plan_parent[parent_index]
                    plan.append(parent)
                
                    if (parent == start_config).all():
                        break
        
                plan = plan[::-1]
                dist = 0.0
                for i in xrange(0,len(plan) - 1):
                    dist = dist + numpy.linalg.norm(plan[i + 1] - plan[i])
                print "Path Length: ", dist
                print "Number of vertices: ", len(plan)
                return plan
