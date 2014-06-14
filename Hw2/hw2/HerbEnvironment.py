import numpy
import time

class HerbEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
        
        # goal sampling probability
        self.p = 0.1
        
    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        

    def GenerateRandomConfiguration(self):
        config = [0] * len(self.robot.GetActiveDOFIndices())
        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
        ratio = numpy.random.rand(7)
        config = lower_limits+(upper_limits-lower_limits)*ratio  
        return numpy.array(config)
    
    def ComputeDistance(self, start_config, end_config):
        dist = numpy.linalg.norm(end_config - start_config)
        return dist   

    def Extend(self, start_config, end_config):
            
            self.env = self.robot.GetEnv()
            table = self.robot.GetEnv().GetBodies()[1]
            config = start_config
            number_samples = 100;
            increment = (end_config - start_config)/number_samples;
            for i in range(0,number_samples):
                self.robot.SetActiveDOFValues(config)
                collision = self.env.CheckCollision(self.robot,table) or self.robot.CheckSelfCollision()
                if collision:
                    config = config - 3*increment
                    return numpy.array(config)
                config =config + increment    
            return numpy.array(config)  
        
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
                        
                        #config = numpy.array(["%.2f" % x for x in config])
                        #n2 = numpy.array(["%.2f" % y for y in n2])
                        
                        config_new = []
                        for i in xrange(0,len(config)):    
                            value = config[i]
                            value = "%.2f" % value
                            value = float(value)
                            config_new.append(value)
                        config_new = numpy.array(config_new)
                        
                        n2_new = []
                        for i in xrange(0,len(n2)):    
                            value = n2[i]
                            value = "%.2f" % value
                            value = float(value)
                            n2_new.append(value)
                        n2_new = numpy.array(n2_new)
                        

                        if (config_new == n2_new).all():
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
