% SWARM CLASS
% ===========
%
% Table of Contents
% -----------------
%
% Properties:
% -----------
%   - Swarm Scenario and Structure:
%       - scenarioData: Flight data of the swarm
%       - simulationScene: Scenario in which the swarm operates
%       - nbAgents: Number of UAVs in the scenario
%       - UAVs: Vector of Drone objects in the swarm
%
%   - Swarm State and Dynamics:
%       - trueLLAPositions: Vector containing LLA positions of each UAV
%       - truePositions: Vector containing true scenario positions of each UAV
%       - swarmInnerConnections: Cell array holding neighbors' indices for each UAV
%       - metropolisWeights: Cell array of Metropolis weights for each UAV
%
%   - Swarm Simulation Parameters and Simulation Output:
%       - swarmParameters: Struct containing parameters related to the swarm
%       - loggedData: Struct containing logged estimation data
%       - processedData: Struct containing processed data with calculated metrics
%       - timeStep: Current time in the simulation
%
% Methods:
% --------
%
% Constructor:
%   - Swarm: Initializes the Swarm object with scenario data, creates UAVs, and sets initial parameters.
%
% Sensor and Measurement Functions:
%   - gpsMount: Defines the GPS model and mounts the GPS sensor on each UAV in the Swarm.
%   - gpsConductMeasurement: Reads GPS sensor measurements from each UAV in the Swarm.
%   - uwbConductMeasurement: Reads UWB sensor measurements from each UAV in the Swarm.
%
% Swarm Update Functions:
%   - updateNavData: Updates the true navigation data (position, velocity, orientation) for each UAV.
%   - checkMotionEnded: Checks if the motion of the UAVs has ended, based on the time of arrival in their trajectories.
%   - updatePosAndDetermineNeighbors: Updates true positions of UAVs and determines their neighbors based on proximity.
%   - updateTruePositions: Updates the true position vector of each UAV in the Swarm.
%
% EKF and Estimation Functions:
%   - extendedKalmanFilter: Applies the Extended Kalman Filter update using GPS and UWB measurements.
%
% Weight and Fusion Functions:
%   - calculateMetropolisWeights: Calculates the Metropolis weights for each UAV based on its neighbors.
%   - getNeighborWeight: Retrieves the weight for a specific neighbor UAV.
%   - fuseWithNeighbors: Fuses the UAV's state with its neighbors using Extended Kalman Filter-based Covariance Intersection.
%   - fuseWithNeighborsEVCI: Fuses the UAV's state with its neighbors using PCA-based Covariance Intersection.
%
% Note:
% -----
% This table of contents provides a structured overview of the Swarm class for easy reference and navigation.
% ==========================================================================================================


classdef Swarm < handle
    % SWARM - This class represents an ensemble of dynamic UAVs

    properties
        scenarioData % Flight data of the swarm
        simulationScene % Scenario in which Swarm operates
        nbAgents % Number of UAVs in the scenario
        UAVs % A vector of Drone objects

        trueLLAPositions % Vector containing LLA positions of each UAV
        truePositions  % Vector containing true scenario positions of each UAV
        swarmInnerConnections % Cell array to hold neighbors' indices for each UAV
        metropolisWeights % Cell array of Metropolis weights for each UAV
        swarmParameters % Struct containing parameters related to the swarm
        loggedData % Struct containing logged estimation data
        processedData % Struct containing processed data with calculated metrics
        timeStep % Current time in the simulation
    end

    methods
        %%%%%%%%%% Constructor %%%%%%%%%%%%
        function self = Swarm(scenarioData)

            self.scenarioData = scenarioData;
            self.simulationScene = uavScenario(UpdateRate=scenarioData.UpdateRate, ...
                ReferenceLocation=scenarioData.ReferenceLocation,...
                StopTime=scenarioData.StopTime);
            self.nbAgents = length(scenarioData.Platforms);

            self.UAVs = [];
            self.trueLLAPositions = [];
            self.truePositions = [];
            self.swarmInnerConnections = cell(1, self.nbAgents);
            self.metropolisWeights = cell(1, self.nbAgents);

            self.swarmParameters = struct();
            self.swarmParameters.nbAgents = self.nbAgents;
            self.swarmParameters.maxRange = 5;

            self.timeStep = self.simulationScene.CurrentTime;

            % Initialize logData as a structure array with one element per UAV
            self.loggedData = repmat(struct('trueState', [], ...
                'estimatedState', [], ...
                'estimatedStateCI', [], ...
                'estimatedStateEVCI', [], ...
                'estimatedCovariance', [], ...
                'estimatedCovarianceCI', [], ...
                'estimatedCovarianceEVCI', []), 1, self.nbAgents);

            % Initialize the loggedData structure
            self.processedData = repmat(struct('rmseEKF', [], ...
                'rmseCI', [], ...
                'rmseEVCI', [], ...
                'ateEKF', [], ...
                'ateCI', [], ...
                'ateEVCI', [], ...
                'rpeEKF', [], ...
                'rpeCI', [], ...
                'rpeEVCI', [], ...
                'nisEKF', [], ...
                'nisCI', [], ...
                'nisEVCI', [], ...
                'reducedVectors',[]), 1, self.nbAgents);

            % Initialize UAVs
            for uavIndex = 1:self.nbAgents
                % Assign flight data to specific UAV - extract from
                % scenarioData
                uavFlightData = scenarioData.Platforms(uavIndex);

                % Create required number of UAVs as Drone class objects
                UAV = Drone(self.simulationScene,uavFlightData,self,self.swarmParameters);

                % Add UAV to UAVs vector containing Drone objects
                self.UAVs = [self.UAVs UAV];

                % Append data about positions to the dedicated vectors
                % which will hold coordinates of the entire Swarm
                self.trueLLAPositions = [self.trueLLAPositions UAV.uavLLAVector];
                self.truePositions = [self.truePositions UAV.uavPosition];
            end

            self.determineInnerConnections(); % Initial neighbors computation
        end
        %%%%%%%%%% End Constructor %%%%%%%%%%%%

        %% Function which is responsible for definining GPS model and
        %  mounting GPS sensor on each UAV in the Swarm
        function gpsMount(self)
            for uavIndex = 1:self.nbAgents
                self.UAVs(uavIndex).gpsMount();
            end
        end

        %% Function which is responsible for reading GSP sensor measurements
        %  from each UAV in the Swarm
        function gpsConductMeasurement(self)
            for uavIndex = 1:self.nbAgents
                self.UAVs(uavIndex).gpsConductMeasurement();
            end
        end

        %% Function which is responsible for reading UWB sensor measurements
        %  from each UAV in the Swarm
        function uwbConductMeasurement(self)
            for uavIndex = 1:self.nbAgents
                self.UAVs(uavIndex).uwbConductMeasurement();
            end
        end

        %% Function responsible for updating true data of the UAVs in a timestep
        function updateNavData(self)
            for uavIndex = 1:self.nbAgents
                self.UAVs(uavIndex).updateNavData();
                self.timeStep = self.simulationScene.CurrentTime;
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

        %% Function which finds neighbors within maxRange of the specified UAV
        % uavIndex: Index of the UAV to check vicinity for (1-based index)
        % maxRange: Maximum range to consider another UAV as a neighbor
        function determineInnerConnections(self)
            for uavIndex = 1:self.nbAgents

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
                        if distance <= self.swarmParameters.maxRange
                            neighbors = [neighbors, i];
                        end
                        distance
                    end
                end
                self.swarmInnerConnections{uavIndex} = neighbors;
            end
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

                % Update the true scenario positions [x y z]
                newPosition = self.UAVs(uavIndex).uavPosition;
                self.truePositions((uavIndex-1)*3 + 1) = newPosition(1);
                self.truePositions((uavIndex-1)*3 + 2) = newPosition(2);
                self.truePositions((uavIndex-1)*3 + 3) = newPosition(3);
            end
            % Update neighbors' information for all UAVs
            self.determineInnerConnections();

            % Calculate Metropolis weights for all UAVs
            self.calculateMetropolisWeights();
        end

        %% Function which applies the EKF update based on GPS and UWB measurements
        % GPS and UWB measurements are carried out before and passed as an
        % input
        function extendedKalmanFilter(self)
            self.gpsConductMeasurement();
            self.uwbConductMeasurement();
            for uavIndex = 1:self.nbAgents
                gpsMeasurements = self.UAVs(uavIndex).gpsMeasurements;
                uwbMeasurements = self.UAVs(uavIndex).uwbMeasurements;
                self.UAVs(uavIndex).extendedKalmanFilter(gpsMeasurements,uwbMeasurements,'None');
                self.UAVs(uavIndex).extendedKalmanFilter(gpsMeasurements,uwbMeasurements,'CI');
                self.UAVs(uavIndex).extendedKalmanFilter(gpsMeasurements,uwbMeasurements,'EVCI');
            end
        end

        %% Function to calculate Metropolis weights for each UAV
        %  Output: metropolisWeights - cell array for each UAV
        function calculateMetropolisWeights(self)
            for i = 1:self.nbAgents
                % Get the list of neighbors for the current UAV
                neighborsList = self.swarmInnerConnections{i};
                numNeighbors = length(neighborsList);

                % Initialize weight vector for the current UAV
                weights = zeros(1, numNeighbors + 1);

                % Calculate the weights for the neighbors
                for j = 1:numNeighbors
                    neighborIndex = neighborsList(j);
                    numNeighborsNeighbor = length(self.swarmInnerConnections{neighborIndex});
                    weights(j) = 1 / (max(numNeighbors, numNeighborsNeighbor) + 1);
                end

                % Calculate the weight for the UAV itself
                weights(end) = 1 - sum(weights(1:numNeighbors));

                % Store the weights
                self.metropolisWeights{i} = weights;
            end
        end

        %% Function to get the weight for a specific neighbor
        %  Input: uavIndex, neighborIndex, metropolisWeights
        %  Output: neighborWeight for specific UAV
        function neighborWeight = getNeighborWeight(self, uavIndex, neighborIndex, metropolisWeights)
            % Find the position of the neighbor in the neighbors list
            neighborsList = self.swarmInnerConnections{uavIndex};
            pos = find(neighborsList == neighborIndex);

            if isempty(pos)
                error('Neighbor not found');
            else
                % Return the corresponding weight
                neighborWeight = metropolisWeights{uavIndex}(pos);
            end
        end

        %% Data fusion with UAVs around - collects data from nieghbors and fuse using CI
        function fuseWithNeighborsCI(self)
            for uavIndex = 1:self.nbAgents
                self.UAVs(uavIndex).fuseWithNeighborsCI();
            end
            for uavIndex = 1:self.nbAgents
                self.UAVs(uavIndex).uavStateVectorCI = self.UAVs(uavIndex).uavStateVectorTMP;
                self.UAVs(uavIndex).uavCovarianceMatrixCI = self.UAVs(uavIndex).uavCovarianceMatrixTMP;
            end
        end

        %% Data fusion with UAVs around - collects data from nieghbors using
        %  eigenvalue decomposition and data reduction, then fuse using CI
        function fuseWithNeighborsEVCI(self)
            for uavIndex = 1:self.nbAgents
                self.UAVs(uavIndex).fuseWithNeighborsEVCI();
            end
            for uavIndex = 1:self.nbAgents
                self.UAVs(uavIndex).uavStateVectorEVCI = self.UAVs(uavIndex).uavStateVectorTMP;
                self.UAVs(uavIndex).uavCovarianceMatrixEVCI = self.UAVs(uavIndex).uavCovarianceMatrixTMP;
            end
        end

        %% Method to log data for each UAV at each time step
        %  Output: loggedData struct
        function logUAVData(self)
            for uavIndex = 1:self.nbAgents
                self.loggedData(uavIndex).trueState(:, self.timeStep) = self.trueLLAPositions;
                self.loggedData(uavIndex).estimatedState(:, self.timeStep) = self.UAVs(uavIndex).uavStateVector;
                self.loggedData(uavIndex).estimatedStateCI(:, self.timeStep) = self.UAVs(uavIndex).uavStateVectorCI;
                self.loggedData(uavIndex).estimatedStateEVCI(:, self.timeStep) = self.UAVs(uavIndex).uavStateVectorEVCI;
                self.loggedData(uavIndex).estimatedCovariance(:, :, self.timeStep) = self.UAVs(uavIndex).uavCovarianceMatrix;
                self.loggedData(uavIndex).estimatedCovarianceCI(:, :, self.timeStep) = self.UAVs(uavIndex).uavCovarianceMatrixCI;
                self.loggedData(uavIndex).estimatedCovarianceEVCI(:, :, self.timeStep) = self.UAVs(uavIndex).uavCovarianceMatrixEVCI;
            end
        end

        %% Function to calculate metrics comparing no data fusion, CI and EVCI
        function calculateMetrics(self)
            for uavIndex = 1:self.nbAgents

                % Extract true and estimated states
                trueState = self.loggedData(uavIndex).trueState;
                estimatedState = self.loggedData(uavIndex).estimatedState;
                estimatedStateCI = self.loggedData(uavIndex).estimatedStateCI;
                estimatedStateEVCI = self.loggedData(uavIndex).estimatedStateEVCI;

                % Calculate RMSE for each method
                self.processedData(uavIndex).rmseEKF = sqrt(mean((trueState - estimatedState).^2, 1));
                self.processedData(uavIndex).rmseCI = sqrt(mean((trueState - estimatedStateCI).^2, 1));
                self.processedData(uavIndex).rmseEVCI = sqrt(mean((trueState - estimatedStateEVCI).^2, 1));

                % Calculate ATE for each method
                self.processedData(uavIndex).ateEKF = sqrt(sum((trueState - estimatedState).^2, 1));
                self.processedData(uavIndex).ateCI = sqrt(sum((trueState - estimatedStateCI).^2, 1));
                self.processedData(uavIndex).ateEVCI = sqrt(sum((trueState - estimatedStateEVCI).^2, 1));

                % Calculate RPE for each method
                if self.timeStep > 1
                    for t = 2:self.timeStep
                        deltaTrue = trueState(:, t) - trueState(:, t-1);
                        deltaEKF = estimatedState(:, t) - estimatedState(:, t-1);
                        deltaCI = estimatedStateCI(:, t) - estimatedStateCI(:, t-1);
                        deltaEVCI = estimatedStateEVCI(:, t) - estimatedStateEVCI(:, t-1);

                        self.processedData(uavIndex).rpeEKF(t-1) = sqrt(sum((deltaTrue - deltaEKF).^2));
                        self.processedData(uavIndex).rpeCI(t-1) = sqrt(sum((deltaTrue - deltaCI).^2));
                        self.processedData(uavIndex).rpeEVCI(t-1) = sqrt(sum((deltaTrue - deltaEVCI).^2));
                    end
                end

                % Calculate NIS for each method
                for t = 1:self.timeStep
                    P_EKF = self.loggedData(uavIndex).estimatedCovariance(:, :, t);
                    P_CI = self.loggedData(uavIndex).estimatedCovarianceCI(:, :, t);
                    P_EVCI = self.loggedData(uavIndex).estimatedCovarianceEVCI(:, :, t);

                    innovationEKF = trueState(:, t) - estimatedState(:, t);
                    innovationCI = trueState(:, t) - estimatedStateCI(:, t);
                    innovationEVCI = trueState(:, t) - estimatedStateEVCI(:, t);

                    self.processedData(uavIndex).nisEKF(t) = innovationEKF' / P_EKF * innovationEKF;
                    self.processedData(uavIndex).nisCI(t) = innovationCI' / P_CI * innovationCI;
                    self.processedData(uavIndex).nisEVCI(t) = innovationEVCI' / P_EVCI * innovationEVCI;
                end
            end
        end

        %% Method to plot RMSE
        function plotRMSE(self)
            figure;
            hold on;
            for uavIndex = 1:self.nbAgents
                plot(self.processedData(uavIndex).rmseEKF, 'r', 'DisplayName', ['UAV ' num2str(uavIndex) ' EKF']);
                plot(self.processedData(uavIndex).rmseCI, 'g', 'DisplayName', ['UAV ' num2str(uavIndex) ' CI']);
                plot(self.processedData(uavIndex).rmseEVCI, 'b', 'DisplayName', ['UAV ' num2str(uavIndex) ' EVCI']);
            end
            xlabel('Time Step');
            ylabel('RMSE');
            legend('show');
            title('RMSE Comparison');
            grid on;
        end

        %% Method to plot ATE
        function plotATE(self)
            figure;
            hold on;
            for uavIndex = 1:self.nbAgents
                plot(self.processedData(uavIndex).ateEKF, 'r', 'DisplayName', ['UAV ' num2str(uavIndex) ' EKF']);
                plot(self.processedData(uavIndex).ateCI, 'g', 'DisplayName', ['UAV ' num2str(uavIndex) ' CI']);
                plot(self.processedData(uavIndex).ateEVCI, 'b', 'DisplayName', ['UAV ' num2str(uavIndex) ' EVCI']);
            end
            xlabel('Time Step');
            ylabel('ATE');
            legend('show');
            title('ATE Comparison');
            grid on;
        end

        %% Method to plot RPE
        function plotRPE(self)
            figure;
            hold on;
            for uavIndex = 1:self.nbAgents
                plot(self.processedData(uavIndex).rpeEKF, 'r', 'DisplayName', ['UAV ' num2str(uavIndex) ' EKF']);
                plot(self.processedData(uavIndex).rpeCI, 'g', 'DisplayName', ['UAV ' num2str(uavIndex) ' CI']);
                plot(self.processedData(uavIndex).rpeEVCI, 'b', 'DisplayName', ['UAV ' num2str(uavIndex) ' EVCI']);
            end
            xlabel('Time Step');
            ylabel('RPE');
            legend('show');
            title('RPE Comparison');
            grid on;
        end

        %% Method to plot NIS
        function plotNIS(self)
            figure;
            hold on;
            for uavIndex = 1:self.nbAgents
                plot(self.processedData(uavIndex).nisEKF, 'r', 'DisplayName', ['UAV ' num2str(uavIndex) ' EKF']);
                plot(self.processedData(uavIndex).nisCI, 'g', 'DisplayName', ['UAV ' num2str(uavIndex) ' CI']);
                plot(self.processedData(uavIndex).nisEVCI, 'b', 'DisplayName', ['UAV ' num2str(uavIndex) ' EVCI']);
            end
            xlabel('Time Step');
            ylabel('NIS');
            legend('show');
            title('NIS Comparison');
            grid on;
        end
    end
end

