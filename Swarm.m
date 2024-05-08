classdef Swarm < handle
    % SWARM - This class represents an ensemble of dynamic UAVs

    properties
        UAVs % a vector of Drone objects
        nbAgents % size of the above vector
        simulationScene % Scenario in which Swarm operates
        scenarioData % Flight data of the swarm
    end

    methods
        %%%%%%%%%% Constructor %%%%%%%%%%%%
        function self = Swarm(scenarioData)
            
            self.scenarioData = scenarioData;
            self.UAVs = [];
            self.nbAgents = length(scenarioData.Platforms);
            self.simulationScene = uavScenario(UpdateRate=scenarioData.UpdateRate, ...
                ReferenceLocation=scenarioData.ReferenceLocation,...
                StopTime=scenarioData.StopTime);
            
            % Initialize UAVs
            for i = 1:self.nbAgents
                uavFlightData = scenarioData.Platforms(i);
                UAV = Drone(self.simulationScene,uavFlightData);
                self.UAVs = [self.UAVs UAV];
            end
        end
        %%%%%%%%%% End Constructor %%%%%%%%%%%%
        
        %% Function which is responsible for definining GPS model and 
        %  mounting GPS sensor on each UAV in the Swarm
        function mountGPS(self)
            for i = 1:self.nbAgents
                self.UAVs(i).mountGPS();
            end
        end
       
        %% Function which is responsible for reading GSP sensor measurements 
        %  from each UAV in the Swarm
        function readGPS(self)
            for i = 1:self.nbAgents
                self.UAVs(i).readGPS();
            end
        end

        %% Function responsible for updating true data of the UAVs in a timestep
        function updateNavData(self)
            for i = 1:self.nbAgents
                self.UAVs(i).updateNavData();
            end
        end

        %% Function for checking if the motion of the UAVs ended (based on 
        %  the motion of the single, chosen UAV)
        function motionEnded = checkMotionEnded(self)
            if self.simulationScene.CurrentTime >= self.UAVs(1).uavPlatform.Trajectory.TimeOfArrival(end)
                motionEnded = 1;
            else 
                motionEnded = 0;
            end
        end
    end
end
