classdef Swarm < handle
    % SWARM - This class represents an ensemble of dynamic UAVs

    properties
        UAVs % a vector of Drone objects
        nbAgents % size of the above vector
        simulationScene % Scenario in which Swarm operates
        scenarioData % Flight data of the swarm
        trueLLAPositions % Vector containing LLA positions of each UAV
        truePositions  % Vector containing true scenario positions of each UAV
        neighbors % Cell array to hold neighbors' indices for each UAV
    end

    methods
        %%%%%%%%%% Constructor %%%%%%%%%%%%
        function self = Swarm(scenarioData)

            self.scenarioData = scenarioData;
            self.UAVs = [];
            self.trueLLAPositions = [];
            self.truePositions = [];
            self.nbAgents = length(scenarioData.Platforms);
            self.simulationScene = uavScenario(UpdateRate=scenarioData.UpdateRate, ...
                ReferenceLocation=scenarioData.ReferenceLocation,...
                StopTime=scenarioData.StopTime);

            swarmParameters = struct();
            swarmParameters.nbAgents = self.nbAgents;

            % Initialize UAVs
            for uavIndex = 1:self.nbAgents
                uavFlightData = scenarioData.Platforms(uavIndex);
                UAV = Drone(self.simulationScene,uavFlightData,self,swarmParameters);
                self.UAVs = [self.UAVs UAV];
                self.trueLLAPositions = [self.trueLLAPositions UAV.uavLLAVector];
                self.truePositions = [self.truePositions UAV.uavPosition];
            end

            self.neighbors = cell(1, self.nbAgents);
            self.updateAllNeighbors(); % Initial neighbors computation
        end
        %%%%%%%%%% End Constructor %%%%%%%%%%%%

        %% Function which is responsible for definining GPS model and
        %  mounting GPS sensor on each UAV in the Swarm
        function mountGPS(self)
            for uavIndex = 1:self.nbAgents
                self.UAVs(uavIndex).mountGPS();
            end
        end

        %% Function which is responsible for reading GSP sensor measurements
        %  from each UAV in the Swarm
        function readGPS(self)
            for uavIndex = 1:self.nbAgents
                self.UAVs(uavIndex).readGPS();
            end
        end

        %% Function responsible for updating true data of the UAVs in a timestep
        function updateNavData(self)
            for uavIndex = 1:self.nbAgents
                self.UAVs(uavIndex).updateNavData();
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

        %% Function which updates the neighbors' information for all UAVs
        function updateAllNeighbors(self)
            for uavIndex = 1:self.nbAgents
                self.neighbors{uavIndex} = self.checkVicinity(uavIndex, 10);
            end
        end

        %% Function which finds neighbors within maxRange of the specified UAV
        % uavIndex: Index of the UAV to check vicinity for (1-based index)
        % maxRange: Maximum range to consider another UAV as a neighbor
        function neighbors = checkVicinity(self, uavIndex, maxRange)

            % Initialize neighbors vector
            neighbors = [];

            % Coordinates of the UAV to check
            x = self.trueLLAPositions((uavIndex-1)*3 + 1);
            y = self.trueLLAPositions((uavIndex-1)*3 + 2);
            z = self.trueLLAPositions((uavIndex-1)*3 + 3);

            % Loop through all other UAVs to find neighbors within maxRange
            for i = 1:(length(self.trueLLAPositions) / 3)
                if i ~= uavIndex
                    % Coordinates of the other UAV
                    x_other = self.trueLLAPositions((i-1)*3 + 1);
                    y_other = self.trueLLAPositions((i-1)*3 + 2);
                    z_other = self.trueLLAPositions((i-1)*3 + 3);

                    % Calculate Euclidean distance
                    distance = sqrt((x - x_other)^2 + (y - y_other)^2 + (z - z_other)^2);

                    % Check if within maxRange
                    if distance <= maxRange
                        neighbors = [neighbors, i];
                    end
                end
            end
        end

        %% Function which simulates UWB range measurements to neighbors
        % uavIndex: Index of the UAV conducting the measurements (1-based index)
        % measurementError: Standard deviation of the measurement error
        function ranges = conductRangeMeasurements(self, uavIndex, measurementError)

            % Get neighbors from the neighbors cell array
            selfNeighbors = self.neighbors{uavIndex};

            % Initialize range measurements vector
            ranges = zeros(1, length(selfNeighbors));

            % Coordinates of the UAV conducting the measurements
            x = self.trueLLAPositions((uavIndex-1)*3 + 1);
            y = self.trueLLAPositions((uavIndex-1)*3 + 2);
            z = self.trueLLAPositions((uavIndex-1)*3 + 3);

            % Calculate the range to each neighbor
            for i = 1:length(selfNeighbors)
                neighborIndex = selfNeighbors(i);

                % Coordinates of the neighbor UAV
                x_other = self.trueLLAPositions((neighborIndex-1)*3 + 1);
                y_other = self.trueLLAPositions((neighborIndex-1)*3 + 2);
                z_other = self.trueLLAPositions((neighborIndex-1)*3 + 3);

                % Calculate true range
                trueRange = sqrt((x - x_other)^2 + (y - y_other)^2 + (z - z_other)^2);

                % Add measurement noise
                noisyRange = trueRange + measurementError * randn();

                % Store the noisy range
                ranges(i) = noisyRange;
            end
            self.UAVs(uavIndex).uwbRanges = ranges;
        end

        %% Function which updates true position vector of a specified UAV
        % uavIndex: Index of the UAV to update (1-based index)
        % newPosition: New [Lat Long Alt] position of the UAV
        function updateTruePositions(self)
            for uavIndex = 1:self.nbAgents
                % Update the true positions [Lat Long Alt]
                newLLAPosition = self.UAVs(uavIndex).uavLLAVector;
                self.trueLLAPositions((uavIndex-1)*3 + 1) = newLLAPosition(1);
                self.trueLLAPositions((uavIndex-1)*3 + 2) = newLLAPosition(2);
                self.trueLLAPositions((uavIndex-1)*3 + 3) = newLLAPosition(3);
                % Update the ground truth positions
                newPosition = self.UAVs(uavIndex).uavPosition;
                self.truePositions((uavIndex-1)*3 + 1) = newPosition(1);
                self.truePositions((uavIndex-1)*3 + 2) = newPosition(2);
                self.truePositions((uavIndex-1)*3 + 3) = newPosition(3);
            end
            % Update neighbors' information for all UAVs
            self.updateAllNeighbors();
        end

        %% Function which applies the EKF update based on GPS and UWB measurements
        % GPS and UWB measurements are carried out before and passed as an
        % input
        function extendedKalmanFilter(self)
            self.readGPS();
            for uavIndex = 1:self.nbAgents
                gpsMeasurements = self.UAVs(uavIndex).gpsPosition';
                uwbMeasurements = self.conductRangeMeasurements(uavIndex,0);
                self.UAVs(uavIndex).extendedKalmanFilter(gpsMeasurements,uwbMeasurements);
            end
        end

        function fuseWithNeighbors(self)
            for uavIndex = 1:self.nbAgents
                self.UAVs(uavIndex).fuseWithNeighbors();
            end
        end


    end
end

