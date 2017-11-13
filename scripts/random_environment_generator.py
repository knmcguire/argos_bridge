#!/usr/bin/env python

import rospy
import numpy as np

from std_srvs.srv import Empty


class EnvironmentGrid:
    def __init__(self):
        self.agentPresent=False;
    def setAgentPresence(self, agentPresent):
        self.agentPresent=agentPresent;
    def getAgentPresence(self):
        return self.agentPresent;
        
        
        
class RandomEnvironmentGenerator:
    
    width_environment = 14
    height_environment = 14
    chance_agent_gostraight = 0.85
    density_corridor = 0.4
    density_openings = 0.4
    size_rooms = 5
    starting_location_agents = np.matrix('4,1;5,14')
    iterations = 100
    visualize_agents = False
    create_rooms = True
    show_end_result = True
    make_ARGoS_environment = True
    
    def __init__(self):
        print "init"
        vGrid = np.vectorize(EnvironmentGrid)
        vInit = np.arange(self.width_environment*self.height_environment).reshape((self.width_environment,self.height_environment))
        mGrid = np.empty((self.width_environment,self.height_environment),dtype=object)
        mGrid[:,:] = vGrid(vInit);
        
        
        
        #for it in range(1,self.starting_location_agents.size()/2): 
         #   mGrid.item(self.starting_location_agents.item(it,1),self.starting_location_agents.item(it,2)).setAgentPresence(1)
        
                
    def generateRandomEnvironment(self):
        print "generate environment"
        
    def agentMakingCorridors(self):
        print "making corridors"
        

    def handleGenerator(self):
        print "get service"
        self.generateRandomEnvironment()

        
if __name__ == '__main__':
   # rospy.init_node("random_environment_generator")
    envgen = RandomEnvironmentGenerator()
    envgen.agentMakingCorridors()
    
    #s=rospy.Service('generate_new_environment',Empty,envgen.handleGenerator())
    #rospy.spin()
