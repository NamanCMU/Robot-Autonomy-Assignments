import numpy, operator
from RRTPlanner import RRTTree

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
            

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        plan_tempf = []
        plan_tempf.append(start_config)
        plan_tempr = []
        plan_tempr.append(goal_config)
        plan_parentf = []
        plan_parentf.append(start_config);
        plan_parentr = []
        plan_parentr.append(goal_config);
        
        
        ftree.AddVertex(start_config)  
        rtree.AddVertex(goal_config)  

        count = 0;
        self.env = self.planning_env.robot.GetEnv()
        robot_name = self.env.GetBodies()[0].GetName()

        with self.env:
            with self.planning_env.robot.CreateRobotStateSaver():
                while(1):
                    sample_config = self.planning_env.GenerateRandomConfiguration()

                    vidf1, nearest_vertexf = ftree.GetNearestVertex(sample_config) # Finding the nearest vertex
                    vidr1, nearest_vertexr = rtree.GetNearestVertex(sample_config) # Finding the nearest vertex

                    sample_extend_configf = self.planning_env.Extend(nearest_vertexf,sample_config) # Checking for collision
                    sample_extend_configr = self.planning_env.Extend(nearest_vertexr,sample_config) # Checking for collision

                    plan_tempf.append(sample_extend_configf)
                    plan_tempr.append(sample_extend_configr)
                
                    plan_parentf.append(nearest_vertexf)
                    plan_parentr.append(nearest_vertexr)
                    
                    vidf2 = ftree.AddVertex(sample_extend_configf)
                    vidr2 = rtree.AddVertex(sample_extend_configr)
                    ftree.AddEdge(vidf1,vidf2)
                    rtree.AddEdge(vidr1,vidr2)
                    
                    if (robot_name != 'Herb2'):
                        self.planning_env.PlotEdge(nearest_vertexf,plan_tempf[-1])
                        self.planning_env.PlotEdge(nearest_vertexr,plan_tempr[-1])

                    vidf1, nearest_vertex_other_f = rtree.GetNearestVertex(sample_extend_configf) # Finding the nearest vertex
                    vidr1, nearest_vertex_other_r = ftree.GetNearestVertex(sample_extend_configr) # Finding the nearest vertex

                    if self.planning_env.ComputeDistance(nearest_vertex_other_f,sample_extend_configf) < 0.01 or self.planning_env.ComputeDistance(nearest_vertex_other_r,sample_extend_configr) < 0.01 : # Break Condition
                        break;

                    if (sample_extend_configr == sample_extend_configf).all():
                        break

                    count = count + 1;

                print "Tree Created"
                planf = []
                planr = []
                planf.append(sample_extend_configf)
                planr.append(sample_extend_configr)
                planf.append(plan_parentf[-1])
                planr.append(plan_parentr[-1])

                parentf = plan_parentf[-1]
                parentr = plan_parentr[-1]

                plan_tempf_list = [list(x) for x in plan_tempf]
                plan_tempr_list = [list(x) for x in plan_tempr]

            
                while(1):
                    parentf_index = plan_tempf_list.index(list(parentf))
                    parentf = plan_parentf[parentf_index]
                    planf.append(parentf)
                    if (parentf == start_config).all():
                        break

                while(1):
                    parentr_index = plan_tempr_list.index(list(parentr))
                    parentr = plan_parentr[parentr_index]
                    planr.append(parentr)

                    if (parentr == goal_config).all():
                        break

                planf = planf[::-1]
                plan = planf + planr
                dist = 0.0
                for i in xrange(0,len(plan) - 1):
                    dist = dist + numpy.linalg.norm(plan[i + 1] - plan[i])
                print "Path Length: ", dist
                print "Number of vertices: ", len(plan)
                return plan
