% SWARM CLASS
% ===========
%
% Table of Contents
% -----------------
%
% Properties:
% -----------
%   - Swarm Scenario, Structure and Parameters:
%       - swarmFlightData: Flight data of the swarm
%       - swarmSimulationScene: Scenario in which the swarm operates
%       - swarmParameters: Struct containing parameters related to the swarm
%       - nbAgents: Number of UAVs in the scenario
%       - UAVs: Vector of Drone objects in the swarm
%
%   - Swarm State and Dynamics:
%       - swarmTrueLLAPositions: Vector containing LLA positions of each UAV
%       - swarmTruePositions: Vector containing true scenario positions of each UAV
%       - swarmTrueVelocites: Vector containing true scenario velocities of each UAV
%       - swarmInnerConnections: Cell array holding neighbors' indices for each UAV
%       - swarmMetropolisWeights: Cell array of Metropolis weights for each UAV
%
%   - Swarm Simulation Output:
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
%   - gpsCheckNoise: Function which checks wheter the condtions conditions are met to apply noise.
%   - uwbConductMeasurement: Reads UWB sensor measurements from each UAV in the Swarm.
%
% Swarm Update Functions:
%   - updateNavData: Updates the true navigation data (position, velocity, orientation) for each UAV.
%   - checkMotionEnded: Checks if the motion of the UAVs has ended, based on the time of arrival in their trajectories.
%   - updatePosAndDetermineNeighbors: Updates true positions of UAVs and determines their neighbors based on proximity.
%   - updateTrueStates: Updates the true position vector of each UAV in the Swarm.
%
% EKF and Estimation Functions:
%   - extendedKalmanFilter: Applies the Extended Kalman Filter update using GPS and UWB measurements.
%
% Weight and Fusion Functions:
%   - calculateMetropolisWeights: Calculates the Metropolis weights for each UAV based on its neighbors.
%   - getNeighborWeight: Retrieves the weight for a specific neighbor UAV.
%   - fuseWithNeighborsCI: Fuses the UAV's state with its neighbors using Extended Kalman Filter-based Covariance Intersection.
%   - fuseWithNeighborsEVCI: Fuses the UAV's state with its neighbors using PCA-based Covariance Intersection.
%
% Data Logging and Metric Calculation:
%   - logUAVData: Logs true and estimated states for each UAV at each time step.
%   - calculateMetrics: Calculates metrics (RMSE, ATE, RPE, NIS) comparing different estimation methods.
%
% Visualization Functions:
%   - plotRMSE: Plots RMSE comparison of different estimation methods over time.
%   - plotATE: Plots ATE comparison of different estimation methods over time.
%   - plotRPE: Plots RPE comparison of different estimation methods over time.
%   - plotNIS: Plots NIS comparison of different estimation methods over time.
%   - plotSwarmEstimations: Plots estimated positions of all UAVs from the perspective of a specific UAV.
%
% Note:
% -----
% This class manages the dynamics, state estimation, sensor measurements, data fusion, and visualization
% for a swarm of UAVs within a simulation environment.
% ==========================================================================================================


classdef Swarm < handle
    % SWARM - This class represents an ensemble of dynamic UAVs

    properties
        swarmFlightData         % Flight data of the swarm
        swarmSimulationScene    % Scenario in which Swarm operates
        swarmParameters         % Struct containing parameters related to the swarm

        swarmNbAgents           % Number of UAVs in the scenario
        UAVs                    % A vector of Drone objects

        swarmTrueLLAPositions   % Vector containing LLA positions of each UAV
        swarmTruePositions      % Vector containing true scenario positions of each UAV
        swarmTrueVelocities      % Vector containing true scenario velocities of each UAV
        swarmInnerConnections   % Cell array to hold neighbors' indices for each UAV
        swarmMetropolisWeights  % Cell array of Metropolis weights for each UAV

        simLoggedData           % Struct containing logged estimation data
        simProcessedData        % Struct containing processed data with calculated metrics
        simTimeStep             % Current time in the simulation
    end

    methods
        %%%%%%%%%% Constructor %%%%%%%%%%%%
        function self = Swarm(scenarioData)

            % Initialize scenario data and simulation scene
            self.swarmFlightData = scenarioData;
            self.swarmSimulationScene = uavScenario(UpdateRate=scenarioData.UpdateRate, ...
                ReferenceLocation=scenarioData.ReferenceLocation, ...
                StopTime=scenarioData.StopTime,MaxNumFrames=20);
            self.swarmNbAgents = length(scenarioData.Platforms);

            % Initialize arrays and structures
            self.UAVs = [];
            self.swarmTrueLLAPositions = [];
            self.swarmTruePositions = [];
            self.swarmTrueVelocities = [];
            self.swarmInnerConnections = cell(1, self.swarmNbAgents);
            self.swarmMetropolisWeights = cell(1, self.swarmNbAgents);
            
            % Initialize swarm parameters structure
            self.swarmParameters = repmat(struct('nbAgents', [], ...
                'estimationModel', [], ...
                'maxCommunicationRange', [], ...
                'evciPosReductionThreshold', [],...
                'evciVelReductionThreshold', [],...
                'noisePresence', [],...
                'timeOfNoisePresence', []),1);

            % Initialize logged data structure
            self.simLoggedData = repmat(struct('trueState', [], ...
                'estimatedState', [], ...
                'estimatedStateCI', [], ...
                'estimatedStateEVCI', [], ...
                'estimatedCovariance', [], ...
                'estimatedCovarianceCI', [], ...
                'estimatedCovarianceEVCI', []), 1, self.swarmNbAgents);

            % Initialize processed data structure
            self.simProcessedData = repmat(struct('rmseEKF', [], ...
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
                'reducedVectors',[]), 1, self.swarmNbAgents);

            self.swarmParameters.nbAgents = self.swarmNbAgents;
            self.swarmParameters.estimationModel = 'PV';
            self.swarmParameters.maxRange = 100;
            self.swarmParameters.evciPosReductionThreshold = 1000;
            self.swarmParameters.evciVelReductionThreshold = 10;
            self.swarmParameters.noisePresence = 0;
            self.swarmParameters.timeOfNoisePresence = [5 10];

            % Initialize UAVs and their positions
            for uavIndex = 1:self.swarmNbAgents
                uavFlightData = scenarioData.Platforms(uavIndex);
                UAV = Drone(self.swarmSimulationScene, uavFlightData, self, self.swarmParameters);
                self.UAVs = [self.UAVs UAV];
                self.swarmTrueLLAPositions = [self.swarmTrueLLAPositions UAV.uavLLAVector];
                self.swarmTruePositions = [self.swarmTruePositions UAV.uavTruePosition];
                self.swarmTrueVelocities = [self.swarmTrueVelocities UAV.uavTrueVelocity];
            end

            % Mount GPS sensor on each UAV
            self.gpsMount();

            % Determine initial neighbors' connections
            self.determineInnerConnections();

            % Update simulation time
            self.simTimeStep = self.swarmSimulationScene.CurrentTime;
        end
        %%%%%%%%%% End Constructor %%%%%%%%%%%%

        %% Function which is responsible for definining GPS model and
        %  mounting GPS sensor on each UAV in the Swarm
        function gpsMount(self)
            for uavIndex = 1:self.swarmNbAgents
                self.UAVs(uavIndex).gpsMount();
            end
        end

        %% Function which is responsible for reading GSP sensor measurements
        %  from each UAV in the Swarm
        function gpsConductMeasurement(self)
            for uavIndex = 1:self.swarmNbAgents
                self.UAVs(uavIndex).gpsConductMeasurement();
            end
        end

        %% Function which checks wheter the condtions conditions are met 
        %  to apply noise or restore original state of the gpsSensor 
        %  (the conditions are time of noise)
        %  Input: uavIndices - indices of UAVs with degraded GPS
        function gpsCheckNoise(self,uavIndices)
            if (self.simTimeStep >= self.swarmParameters.timeOfNoisePresence(1)) && ...
                    (self.simTimeStep <= self.swarmParameters.timeOfNoisePresence(2)) && ...
                    (self.swarmParameters.noisePresence == 0)
                for uavIndex = uavIndices
                    self.UAVs(uavIndex).gpsAddNoise();
                end
                self.swarmParameters.noisePresence = 1;
            elseif (self.simTimeStep >= self.swarmParameters.timeOfNoisePresence(2)) && ...
                    (self.swarmParameters.noisePresence == 1)
                for uavIndex = uavIndices
                    self.UAVs(uavIndex).gpsDeleteNoise();
                end
            end
        end

        %% Function which is responsible for reading UWB sensor measurements
        %  from each UAV in the Swarm
        function uwbConductMeasurement(self)
            for uavIndex = 1:self.swarmNbAgents
                self.UAVs(uavIndex).uwbConductMeasurement();
            end
        end

        %% Function responsible for updating true data of the UAVs in a timestep
        function updateNavData(self)
            for uavIndex = 1:self.swarmNbAgents
                self.UAVs(uavIndex).updateNavData();
                self.simTimeStep = self.swarmSimulationScene.CurrentTime;
            end
        end

        %% Function for checking if the motion of the UAVs ended (based on
        %  the motion of the single, chosen UAV)
        function motionEnded = checkMotionEnded(self)
            if self.swarmSimulationScene.CurrentTime >= self.UAVs(1).uavPlatform.Trajectory.TimeOfArrival(end)
                motionEnded = 1;
            else
                motionEnded = 0;
            end
        end

        %% Function which finds neighbors within maxRange of the specified UAV
        %  uavIndex: Index of the UAV to check vicinity for (1-based index)
        %  maxRange: Maximum range to consider another UAV as a neighbor
        function determineInnerConnections(self)
            for uavIndex = 1:self.swarmNbAgents

                % Initialize neighbors vector
                neighbors = [];

                % Coordinates of the UAV to check
                x = self.swarmTruePositions((uavIndex-1)*3 + 1);
                y = self.swarmTruePositions((uavIndex-1)*3 + 2);
                z = self.swarmTruePositions((uavIndex-1)*3 + 3);

                % Loop through all other UAVs to find neighbors within maxRange
                for i = 1:(length(self.swarmTruePositions) / 3)
                    if i ~= uavIndex
                        % Coordinates of the other UAV
                        x_other = self.swarmTruePositions((i-1)*3 + 1);
                        y_other = self.swarmTruePositions((i-1)*3 + 2);
                        z_other = self.swarmTruePositions((i-1)*3 + 3);

                        % Calculate Euclidean distance
                        distance = sqrt((x - x_other)^2 + (y - y_other)^2 + (z - z_other)^2);

                        % Check if within maxRange
                        if distance <= self.swarmParameters.maxRange
                            neighbors = [neighbors, i];
                        end
                    end
                end
                self.swarmInnerConnections{uavIndex} = neighbors;
            end
        end

        %% Function which updates true position vector of a specified UAV
        %  uavIndex: Index of the UAV to update (1-based index)
        %  newPosition: New [Lat Long Alt] position of the UAV
        function updateTrueStates(self)
            for uavIndex = 1:self.swarmNbAgents
                % Update the true positions [Lat Long Alt]
                newLLAPosition = self.UAVs(uavIndex).uavLLAVector;
                self.swarmTrueLLAPositions((uavIndex-1)*3 + 1) = newLLAPosition(1);
                self.swarmTrueLLAPositions((uavIndex-1)*3 + 2) = newLLAPosition(2);
                self.swarmTrueLLAPositions((uavIndex-1)*3 + 3) = newLLAPosition(3);

                % Update the true scenario positions [x y z]
                newPosition = self.UAVs(uavIndex).uavTruePosition;
                self.swarmTruePositions((uavIndex-1)*3 + 1) = newPosition(1);
                self.swarmTruePositions((uavIndex-1)*3 + 2) = newPosition(2);
                self.swarmTruePositions((uavIndex-1)*3 + 3) = newPosition(3);

                % Update the true scenario velcoities [x y z]
                newVelocity = self.UAVs(uavIndex).uavTrueVelocity;
                self.swarmTrueVelocities((uavIndex-1)*3 + 1) = newVelocity(1);
                self.swarmTrueVelocities((uavIndex-1)*3 + 2) = newVelocity(2);
                self.swarmTrueVelocities((uavIndex-1)*3 + 3) = newVelocity(3);
            end
            % Update neighbors' information for all UAVs
            self.determineInnerConnections();

            % Calculate Metropolis weights for all UAVs
            self.calculateMetropolisWeights();
        end

        %% Function which applies the EKF update based on GPS and UWB measurements
        %  GPS and UWB measurements are carried out before and passed as an
        %  input
        function extendedKalmanFilter(self)
            for uavIndex = 1:self.swarmNbAgents
                self.UAVs(uavIndex).extendedKalmanFilter('None');
                self.UAVs(uavIndex).extendedKalmanFilter('CI');
                self.UAVs(uavIndex).extendedKalmanFilter('EVCI');
            end
        end

        %% Function to calculate Metropolis weights for each UAV
        %  Output: metropolisWeights - cell array for each UAV
        function calculateMetropolisWeights(self)
            % Initialize the Metropolis weights cell array
            self.swarmMetropolisWeights = cell(1, self.swarmNbAgents);

            % Iterate over each UAV
            for i = 1:self.swarmNbAgents
                % Get the list of neighbors for the current UAV
                neighborsList = self.swarmInnerConnections{i};
                numNeighbors = length(neighborsList);

                % Initialize weight vector for the current UAV
                weights = zeros(1, self.swarmNbAgents);

                % Special case: if there are no neighbors, the weight is 1 for itself
                if numNeighbors == 0
                    weights(i) = 1;
                else
                    % Calculate the weights for the neighbors
                    for j = 1:numNeighbors
                        neighborIndex = neighborsList(j);
                        numNeighborsNeighbor = length(self.swarmInnerConnections{neighborIndex});

                        % Metropolis weight formula for neighbors
                        weights(neighborIndex) = 1 / (max(numNeighbors, numNeighborsNeighbor) + 1);
                    end

                    % Calculate the weight for the UAV itself
                    weights(i) = 1 - sum(weights);
                end

                % Store the weights for the current UAV
                self.swarmMetropolisWeights{i} = weights;
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
            for uavIndex = 1:self.swarmNbAgents
                [fusedState, fusedCovariance] = self.UAVs(uavIndex).fuseWithNeighborsCI();
            end
            for uavIndex = 1:self.swarmNbAgents
                if ~isempty(self.swarmInnerConnections{uavIndex})
                    self.UAVs(uavIndex).uavStateVectorCI = fusedState;
                    self.UAVs(uavIndex).uavCovarianceMatrixCI = fusedCovariance;
                end
            end
        end

        %% Data fusion with UAVs around - collects data from nieghbors using
        %  eigenvalue decomposition and data reduction, then fuse using CI
        function fuseWithNeighborsEVCI(self)
            for uavIndex = 1:self.swarmNbAgents
                [fusedState, fusedCovariance] = self.UAVs(uavIndex).fuseWithNeighborsEVCI();
            end
            for uavIndex = 1:self.swarmNbAgents
                if ~isempty(self.swarmInnerConnections{uavIndex})
                    self.UAVs(uavIndex).uavStateVectorEVCI = fusedState;
                    self.UAVs(uavIndex).uavCovarianceMatrixEVCI = fusedCovariance;
                end
            end
        end

        %% Method to log data for each UAV at each time step
        %  Output: loggedData struct
        function logUAVData(self)
            for uavIndex = 1:self.swarmNbAgents
                switch self.swarmParameters.estimationModel
                    case 'P'
                        self.simLoggedData(uavIndex).trueState(:, self.simTimeStep*self.swarmSimulationScene.UpdateRate) = self.swarmTruePositions;
                    case 'PV'
                        velOffset = self.swarmParameters.nbAgents * 3;
                        self.simLoggedData(uavIndex).trueState(1:velOffset, self.simTimeStep*self.swarmSimulationScene.UpdateRate) = self.swarmTruePositions;
                        self.simLoggedData(uavIndex).trueState(velOffset+1:velOffset*2, self.simTimeStep*self.swarmSimulationScene.UpdateRate) = self.swarmTrueVelocities;
                end
                self.simLoggedData(uavIndex).estimatedState(:, self.simTimeStep*self.swarmSimulationScene.UpdateRate) = self.UAVs(uavIndex).uavStateVector;
                self.simLoggedData(uavIndex).estimatedStateCI(:, self.simTimeStep*self.swarmSimulationScene.UpdateRate) = self.UAVs(uavIndex).uavStateVectorCI;
                self.simLoggedData(uavIndex).estimatedStateEVCI(:, self.simTimeStep*self.swarmSimulationScene.UpdateRate) = self.UAVs(uavIndex).uavStateVectorEVCI;
                self.simLoggedData(uavIndex).estimatedCovariance(:, :, self.simTimeStep*self.swarmSimulationScene.UpdateRate) = self.UAVs(uavIndex).uavCovarianceMatrix;
                self.simLoggedData(uavIndex).estimatedCovarianceCI(:, :, self.simTimeStep*self.swarmSimulationScene.UpdateRate) = self.UAVs(uavIndex).uavCovarianceMatrixCI;
                self.simLoggedData(uavIndex).estimatedCovarianceEVCI(:, :, self.simTimeStep*self.swarmSimulationScene.UpdateRate) = self.UAVs(uavIndex).uavCovarianceMatrixEVCI;
            end
        end

        %% Function to calculate metrics comparing no data fusion, CI and EVCI
        function calculateMetrics(self)
            for uavIndex = 1:self.swarmNbAgents

                % Extract true and estimated states
                trueState = self.simLoggedData(uavIndex).trueState;
                estimatedState = self.simLoggedData(uavIndex).estimatedState;
                estimatedStateCI = self.simLoggedData(uavIndex).estimatedStateCI;
                estimatedStateEVCI = self.simLoggedData(uavIndex).estimatedStateEVCI;

                % Calculate RMSE for each method
                self.simProcessedData(uavIndex).rmseEKF = sqrt(mean((trueState - estimatedState).^2, 1));
                self.simProcessedData(uavIndex).rmseCI = sqrt(mean((trueState - estimatedStateCI).^2, 1));
                self.simProcessedData(uavIndex).rmseEVCI = sqrt(mean((trueState - estimatedStateEVCI).^2, 1));

                % Calculate ATE for each method
                self.simProcessedData(uavIndex).ateEKF = sqrt(sum((trueState - estimatedState).^2, 1));
                self.simProcessedData(uavIndex).ateCI = sqrt(sum((trueState - estimatedStateCI).^2, 1));
                self.simProcessedData(uavIndex).ateEVCI = sqrt(sum((trueState - estimatedStateEVCI).^2, 1));

                % Calculate RPE for each method
                if self.simTimeStep > 1
                    for t = 2:self.simTimeStep
                        deltaTrue = trueState(:, t) - trueState(:, t-1);
                        deltaEKF = estimatedState(:, t) - estimatedState(:, t-1);
                        deltaCI = estimatedStateCI(:, t) - estimatedStateCI(:, t-1);
                        deltaEVCI = estimatedStateEVCI(:, t) - estimatedStateEVCI(:, t-1);

                        self.simProcessedData(uavIndex).rpeEKF(t-1) = sqrt(sum((deltaTrue - deltaEKF).^2));
                        self.simProcessedData(uavIndex).rpeCI(t-1) = sqrt(sum((deltaTrue - deltaCI).^2));
                        self.simProcessedData(uavIndex).rpeEVCI(t-1) = sqrt(sum((deltaTrue - deltaEVCI).^2));
                    end
                end

                % Calculate NIS for each method
                for t = 1:self.simTimeStep
                    P_EKF = self.simLoggedData(uavIndex).estimatedCovariance(:, :, t);
                    P_CI = self.simLoggedData(uavIndex).estimatedCovarianceCI(:, :, t);
                    P_EVCI = self.simLoggedData(uavIndex).estimatedCovarianceEVCI(:, :, t);

                    innovationEKF = trueState(:, t) - estimatedState(:, t);
                    innovationCI = trueState(:, t) - estimatedStateCI(:, t);
                    innovationEVCI = trueState(:, t) - estimatedStateEVCI(:, t);

                    self.simProcessedData(uavIndex).nisEKF(t) = innovationEKF' / P_EKF * innovationEKF;
                    self.simProcessedData(uavIndex).nisCI(t) = innovationCI' / P_CI * innovationCI;
                    self.simProcessedData(uavIndex).nisEVCI(t) = innovationEVCI' / P_EVCI * innovationEVCI;
                end
            end
        end

        %% Method to plot RMSE
        function plotRMSE(self)
            figure;
            set(gcf,'color','w')
            hold on;
            color = {[0.75 0.25 0.2]; [0.6 0.8 1]};
            for uavIndex = 1:self.swarmNbAgents
                % plot(self.processedData(uavIndex).rmseEKF, 'r', 'DisplayName', ['UAV ' num2str(uavIndex) ' EKF']);
                plot(self.simProcessedData(uavIndex).rmseCI, 'DisplayName',...
                    ['UAV ' num2str(uavIndex) ' CI'],'Color',color{1}','LineWidth', 4);
                plot(self.simProcessedData(uavIndex).rmseEVCI, 'DisplayName',...
                    ['UAV ' num2str(uavIndex) ' EVCI'],'Color',color{2},'LineWidth', 4);
            end
            xlabel('Time Step');
            ylabel('RMSE');
            title("RMSE Over Time ","FontSize",14);
            % subtitle("Threshold: " + self.swarmParameters.evciReductionThreshold,...
            %     "FontWeight","normal" );
            legend('show');
            grid on;
        end

        %% Method to plot ATE
        function plotATE(self)
            figure;
            hold on;
            for uavIndex = 1:self.swarmNbAgents
                % plot(self.processedData(uavIndex).ateEKF, 'r', 'DisplayName', ['UAV ' num2str(uavIndex) ' EKF']);
                plot(self.simProcessedData(uavIndex).ateCI, 'g', 'DisplayName', ['UAV ' num2str(uavIndex) ' CI']);
                plot(self.simProcessedData(uavIndex).ateEVCI, 'b', 'DisplayName', ['UAV ' num2str(uavIndex) ' EVCI']);
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
            for uavIndex = 1:self.swarmNbAgents
                % plot(self.processedData(uavIndex).rpeEKF, 'r', 'DisplayName', ['UAV ' num2str(uavIndex) ' EKF']);
                plot(self.simProcessedData(uavIndex).rpeCI, 'g', 'DisplayName', ['UAV ' num2str(uavIndex) ' CI']);
                plot(self.simProcessedData(uavIndex).rpeEVCI, 'b', 'DisplayName', ['UAV ' num2str(uavIndex) ' EVCI']);
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
            for uavIndex = 1:self.swarmNbAgents
                % plot(self.processedData(uavIndex).nisEKF, 'r', 'DisplayName', ['UAV ' num2str(uavIndex) ' EKF']);
                plot(self.simProcessedData(uavIndex).nisCI, 'g', 'DisplayName', ['UAV ' num2str(uavIndex) ' CI']);
                plot(self.simProcessedData(uavIndex).nisEVCI, 'b', 'DisplayName', ['UAV ' num2str(uavIndex) ' EVCI']);
            end
            xlabel('Time Step');
            ylabel('NIS');
            legend('show');
            title('NIS Comparison');
            grid on;
        end

        %% Method to plot swarm estimations from the perspective of a specific UAV
        function plotSwarmEstimations(self, uavIndex)
            if uavIndex < 1 || uavIndex > self.swarmNbAgents
                error('Invalid UAV index. Must be between 1 and %d.', self.swarmNbAgents);
            end

            % Initialize figure
            figure;
            hold on;

            % Iterate over each UAV to plot their trajectories
            for agentIndex = 1:self.swarmNbAgents
                % Extract true positions for the current UAV
                plotTruePositions = self.simLoggedData(agentIndex).trueState((agentIndex-1)*3 + (1:3), :);

                % Extract estimated positions from the perspective of uavIndex
                estimatedPositionsEKF = self.simLoggedData(uavIndex).estimatedState((agentIndex-1)*3 + (1:3), :);
                estimatedPositionsCI = self.simLoggedData(uavIndex).estimatedStateCI((agentIndex-1)*3 + (1:3), :);
                estimatedPositionsEVCI = self.simLoggedData(uavIndex).estimatedStateEVCI((agentIndex-1)*3 + (1:3), :);

                % Plot true positions
                plot3(plotTruePositions(1, :), plotTruePositions(2, :), plotTruePositions(3, :), ...
                    '-', 'DisplayName', ['True UAV ' num2str(agentIndex)], 'LineWidth', 1.5);

                % Plot EKF estimated positions
                % plot3(estimatedPositionsEKF(1, :), estimatedPositionsEKF(2, :), estimatedPositionsEKF(3, :), ...
                %     '--', 'DisplayName', ['EKF UAV ' num2str(agentIndex)], 'LineWidth', 1.5);

                % Plot CI estimated positions
                plot3(estimatedPositionsCI(1, :), estimatedPositionsCI(2, :), estimatedPositionsCI(3, :), ...
                    ':', 'DisplayName', ['CI UAV ' num2str(agentIndex)], 'LineWidth', 1.5);

                % Plot EVCI estimated positions
                plot3(estimatedPositionsEVCI(1, :), estimatedPositionsEVCI(2, :), estimatedPositionsEVCI(3, :), ...
                    '-.', 'DisplayName', ['EVCI UAV ' num2str(agentIndex)], 'LineWidth', 1.5);
            end

            % Set plot properties
            xlabel('X Position (meters)');
            ylabel('Y Position (meters)');
            zlabel('Z Position (meters)');
            title(['Swarm Position Estimations from UAV ' num2str(uavIndex)]);
            legend('show');
            grid on;
            axis equal;
            hold off;
        end
    end
end

