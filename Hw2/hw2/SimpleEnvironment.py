import numpy
import pylab as pl
import time



class SimpleEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5.], [5., 5.]]

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.0], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # goal sampling probability
        self.p = 0.1

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        
    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits
        
        mu = 0
        sigma = 1.5
        while(1):
            config = numpy.random.normal(mu,sigma,2);
            if ((config > lower_limits).all() and (config < upper_limits).all()):
                return numpy.array(config)
            else:
                continue    
                
    def ComputeDistance(self, start_config, end_config):
        dist = numpy.linalg.norm(end_config - start_config);
        return dist
        
        pass

    def Extend(self, start_config, end_config):
        
        self.env = self.robot.GetEnv()
        robot_pose = self.robot.GetTransform();
        
        table = self.robot.GetEnv().GetBodies()[1]

        
        pt1 = start_config[0];
        pt2 = start_config[1];
        number_samples = 10;
        increment_x = (end_config[0] - start_config[0])/number_samples;
        increment_y = (end_config[1] - start_config[1])/number_samples;
        for i in xrange(0,number_samples):
            pt1 = pt1 + increment_x;
            pt2 = pt2 + increment_y;
            
            sample_point = numpy.array([pt1,pt2]);
            robot_pose[0][3] = sample_point[0];
            robot_pose[1][3] = sample_point[1];
            self.robot.SetTransform(robot_pose);
            
            inlier1 = self.env.CheckCollision(self.robot,table)    
            if inlier1:
                pt1_return = pt1 - 3*increment_x;
                pt2_return = pt2 - 3*increment_y;
                sample_point_return = numpy.array([pt1_return,pt2_return]) 
                return sample_point_return
        return end_config        

        pass

    def ShortenPath(self, path, timeout=5.0):
        
       time_initial = time.time()
       time_final = -1000
       print len(path)
       with self.env:
            with self.robot.CreateRobotStateSaver():
                while ((time_final - time_initial) < timeout):
                    for index in xrange(1,len(path)-1):
                        n1 = path[index-1];
                        n2 = path[index+1];
                        config = self.Extend(n1,n2)
                
                        if (config == n2).all():
                            del path[index]
                
                        if index >= len(path) -2:
                            break;
                    time_final = time.time()
                print len(path)
                dist = 0.0
                print "AFTER PATH SHORTENING"
                for i in xrange(0,len(path) - 1):
                    dist = dist + numpy.linalg.norm(path[i + 1] - path[i])
                print "Path Length: ", dist
                print "Number of vertices: ", len(path)
                return path


    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

