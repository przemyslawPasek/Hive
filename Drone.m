% DRONE CLASS
% ===========
%
% Table of Contents
% -----------------
%
% Properties:
%   - UAV identification properties and scenario variables
%       - simulationScene: Scenario in which UAV operates
%       - uavPlatform: UAV platform (scenario)
%       - uavFlightData: UAV platform data in scenario
%       - uavIndex: Number of the UAV in the swarm
%       - uavSI: The starting index for the UAV's coordinates
%       - swarm: Reference to the Swarm object containing all UAVs
%       - swarmParameters: Parameters related to the swarm
%
%   - UAV State
%       - uavMotionVector: UAV motion vector with 16 elements
%       - uavPosition: 3D position [X, Y, Z]
%       - uavVelocity: 3D velocity [Vx, Vy, Vz]
%       - uavOrientation: 3D orientation [quaternion vector]
%       - uavLLAVector: 3D position [Lat Long Alt]
%       - uavLatitude: UAV true latitude
%       - uavLongtitude: UAV true longitude
%       - uavAltitude: UAV true altitude
%       - uavPitch: UAV true pitch
%       - uavRoll: UAV true roll
%       - uavYaw: UAV true yaw
%       - uavGroundspeed: UAV true groundspeed
%
%   - GPS Data
%       - gpsPosition: GPS measured position
%       - gpsVelocity: GPS measured velocity
%       - gpsGroundspeed: GPS measured groundspeed
%       - gpsCourse: GPS course
%       - gpsIsUpdated: GPS update status
%       - gpsSampleTime: GPS sample time
%
%   - UWB Sensor Data
%       - uwbRanges: Ranges to UAV neighbors
%
%   - Sensor Models
%       - GPS: GPS sensor model
%       - GPSsensor: Mounted GPS sensor
%       - insGnssSensor: INS/GNSS sensor (to be defined)
%       - uwbSensor: UWB sensor (to be defined)
%
%   - State Variables
%       - uavStateVector: State vector for estimation
%       - uavCovarianceMatrix: Covariance matrix for estimation
%       - uavStateVectorCI: Fused state vector after Covariance Intersection
%       - uavCovarianceMatrixCI: Fused covariance matrix after Covariance Intersection
%
%   - Time Step
%       - timeStep: Simulation time step
%
% Methods:
% --------
%
% Constructor:
%   - Drone: Initializes the Drone object with simulation scene, flight data, swarm, and swarm parameters
%
% Sensor Functions:
%   - mountGPS: Defines the GPS model and mounts the GPS sensor on the UAV
%   - readGPS: Reads the GPS sensor measurements
%
% UAV State Functions:
%   - transformOrientation: Transforms true orientation from quaternion to yaw, pitch, roll angles
%   - calculateGroundspeed: Calculates true groundspeed from velocity components
%   - updateNavData: Updates true UAV data in a timestep
%
% EKF Functions:
%   - extendedKalmanFilter: Applies the Extended Kalman Filter update using GPS and UWB measurements
%   - constructObservationMatrix: Constructs the observation matrix H for EKF
%
% Data fusion functions:
%   - fuseWithNeighborsCI: Fuses the UAV's state with its neighbors using Covariance Intersection
%   - fuseWithNeighborsEVCI: Fuses data with neighbors using PCA-based Covariance Intersection
%   - covarianceIntersection: Applies Covariance Intersection to fuse multiple state estimates
%   - collectNeighborsData: Collects state and covariance data from the UAV's neighbors
%   - pcaCompression: Performs PCA-based data compression on the covariance matrix
%   - reconstructCovarianceMatrix: Reconstructs the covariance matrix from received PCA-compressed data
%
% Mesh Update:
%   - updateMesh: Sets up and updates the platform mesh with position and orientation
%
% Note:
% -----
% This table of contents provides a structured overview of the Drone class for easy reference and navigation.
% ==========================================================================================================

classdef Drone < handle
    properties

        % UAV identification properties and scenario variables
        simulationScene % Scenario in which UAV operates
        uavPlatform % UAV platform (scenario)
        uavFlightData % UAV platform data in scenario
        uavIndex % Number of the UAV in the swarm
        uavSI % The starting index for the UAV's coordinates in the state vector (SI - Starting Index)
        swarm % Reference to the Swarm object containing all UAVs
        swarmParameters % Parameters related to the swarm

        % UAV State
        uavMotionVector % UAV motion vector with 16 elements
        % [x y z vx vy vz ax ay az q1 q2 q3 q4 wx wy wz]

        uavPosition % 3D position [X, Y, Z]
        uavVelocity % 3D velocity [Vx, Vy, Vz]
        uavOrientation % 3D orientation [4 elements quaternion vector)

        uavLLAVector % 3D position [Lat Long Alt]
        uavLatitude % UAV true latitude
        uavLongtitude % UAV true longtitude
        uavAltitude % UAV true altitude
        uavPitch % UAV true pitch
        uavRoll % UAV true roll
        uavYaw % UAV true yaw
        uavGroundspeed % UAV true groundspeed (calculated from uavOrientation)

        % GPS data acquired with GPS sensor
        gpsPosition
        gpsVelocity
        gpsGroundspeed
        gpsCourse
        gpsIsUpdated
        gpsSampleTime

        % Ranges to the UAV neighbors acquired with UWB sensor [r1 r2 ...]
        uwbRanges

        % Sensor Models
        GPS
        GPSsensor
        insGnssSensor
        uwbSensor

        % State variables
        uavStateVector
        uavCovarianceMatrix

        % Fused state variables
        uavStateVectorCI
        uavCovarianceMatrixCI
        uavStateVectorEVCI
        uavCovarianceMatrixEVCI
        uavStateVectorTMP
        uavCovarianceMatrixTMP

        % Time Step
        timeStep
    end

    methods
        %%%%%%%%%% Constructor %%%%%%%%%%%%
        function self = Drone(simulationScene,uavFlightData,swarm,swarmParameters)
            self.uavFlightData = uavFlightData;
            self.uavIndex = str2double(regexp(uavFlightData.Name, '\d+', 'match'));
            self.uavSI = (self.uavIndex - 1) * 3 + 1;
            self.swarm = swarm;
            self.swarmParameters = swarmParameters;

            % Initialize UAV platform
            self.uavPlatform = uavPlatform(uavFlightData.Name,simulationScene,...
                ReferenceFrame=uavFlightData.ReferenceFrame, ...
                Trajectory=uavFlightData.Trajectory);

            % Initialize motion
            % Read uavMotionVector (vector with 16 elements) and uavLLAVector
            [self.uavMotionVector,self.uavLLAVector] = read(self.uavPlatform);

            self.uavPosition = self.uavMotionVector(1,1:3); % Assign uavPosition in local frame (vector with 3 elements)
            self.uavVelocity = self.uavMotionVector(1,4:6); % Assign uavVelocity (vector with 3 elements)
            self.uavOrientation = self.uavMotionVector(1,10:13); % Quaternion vector for orientation

            self.uavLatitude = self.uavLLAVector(1); % Assign uavLatitude (real value)
            self.uavLongtitude = self.uavLLAVector(2); % Assign uavLongtitude (real value)
            self.uavAltitude = self.uavLLAVector(3); % Assign uavAltitude (real value)

            % Transform orientation from quaternion to uavYaw, uawPitch,
            % uavRoll (real values)
            self.transformOrientation();

            % Calculate uavGroundspeed based on 3D velocities from
            % uavVelocity
            self.calculateGroundspeed();

            % Initialize state variables for the estimation
            self.uavStateVector = zeros(self.swarmParameters.nbAgents*3,1);
            self.uavCovarianceMatrix = eye(self.swarmParameters.nbAgents*3)*1000;

            self.uavStateVector(self.uavSI:self.uavSI+2) = self.uavLLAVector';

            self.uavStateVectorCI = self.uavStateVector;
            self.uavStateVectorEVCI = self.uavStateVector;

            self.uavCovarianceMatrixCI = self.uavCovarianceMatrix;
            self.uavCovarianceMatrixEVCI = self.uavCovarianceMatrix;

            % Set up platform mesh. Add a rotation to orient the mesh to the UAV body frame.
            updateMesh(self.uavPlatform,"quadrotor",{10},[1 0 0],self.uavPosition,self.uavOrientation);
        end
        %%%%%%%%%% End Constructor %%%%%%%%%%%%

        %% Function which is responsible for definining GPS model and mounting GPS sensor on UAV
        %  Output: GPS, GPSsensor
        function mountGPS(self)
            self.GPS = gpsSensor('SampleRate',2,'PositionInputFormat','Local','ReferenceLocation',[0 0 0]);
            self.GPSsensor = uavSensor('GPS',self.uavPlatform,self.GPS,'MountingLocation',[0 0.1 0],'UpdateRate',1);
        end

        %% Function which is responsible for reading GSP sensor measurements
        %  Output: gpsPosition, gpsVelocity, gpsGroundspeed, gpsCourse
        function readGPS(self)
            [self.gpsIsUpdated,self.gpsSampleTime, self.gpsPosition,...
                self.gpsVelocity,self.gpsGroundspeed,self.gpsCourse] = ...
                read(self.GPSsensor);
        end

        %% Function which is responsible for transforming true orientation of
        %  the UAV from quaternion (uavOrientation) to yaw, pitch, roll angles
        %  Output: uavYaw, uawPitch, uavRoll
        function transformOrientation(self)
            [self.uavYaw, self.uavPitch, self.uavRoll] = ...
                quat2angle(self.uavOrientation);
        end

        %% Function which is resposnible for calculating true groudspeed of
        %  the UAV from velocities components in xyz dimensions
        %  Output: uavGroundspeed*
        function calculateGroundspeed(self)
            self.uavGroundspeed = sqrt(sum(self.uavVelocity.^2));
        end

        %% Function responsible for updating true data of the UAV in a timestep
        %  Output: uavMotionVector, uavPosition, uavVelocity,
        %  uavOrientation,uavLatitude, uavLongtitude, uavAltitude, uavYaw,
        %  uawPitch, uavRoll, uavGroudspeed
        function updateNavData(self)
            [self.uavMotionVector,self.uavLLAVector] = read(self.uavPlatform);
            self.uavPosition = self.uavMotionVector(1,1:3);
            self.uavVelocity = self.uavMotionVector(1,4:6);
            self.uavOrientation = self.uavMotionVector(1,10:13);

            self.uavLatitude = self.uavLLAVector(1);
            self.uavLongtitude = self.uavLLAVector(2);
            self.uavAltitude = self.uavLLAVector(3);

            self.transformOrientation();
            self.calculateGroundspeed();
        end

        %% Function which applies the EKF update based on GPS and UWB measurements
        % gpsMeasurements: GPS measurements vector [x, y, z]
        % uwbMeasurements: UWB range measurements vector [range1, range2, ...]
        % dt: Time step for the filter
        function extendedKalmanFilter(self,gpsMeasurements,uwbMeasurements,fusionAlgorithm)

            if strcmp(fusionAlgorithm,'None')
                stateVector = self.uavStateVector;
                covarianceMatrix = self.uavCovarianceMatrix;
            elseif strcmp(fusionAlgorithm,'CI')
                stateVector = self.uavStateVectorCI;
                covarianceMatrix = self.uavCovarianceMatrixCI;
            elseif strcmp(fusionAlgorithm,'EVCI')
                stateVector = self.uavStateVectorEVCI;
                covarianceMatrix = self.uavCovarianceMatrixEVCI;
            end

            F = eye(length(stateVector));
            Q = 0.01 * eye(length(stateVector));

            %%%%%%%%%% Prediction %%%%%%%%%%%%
            uavPredictedStateVector = F * stateVector;
            uavPredictedCovarianceMatrix = F * covarianceMatrix * F' + Q;

            nbAgents = self.swarmParameters.nbAgents;

            %%%%%%%%%% Measurement %%%%%%%%%%%%
            z = [gpsMeasurements; uwbMeasurements]; % Build the full measurement vector z

            %%%%%%%%%% H definition %%%%%%%%%%%%
            H = self.constructObservationMatrix(nbAgents, length(z), uavPredictedStateVector);

            % Measurement noise covariance (R)
            R_gps = 0.1 * eye(length(gpsMeasurements));
            R_uwb = 0.2 * eye(length(uwbMeasurements));
            R = blkdiag(R_gps, R_uwb); % Block diagonal matrix combining GPS and UWB noise

            %%%%%%%%%% Update %%%%%%%%%%%%

            y = z - H * uavPredictedStateVector; % Innovation or measurement residual
            S = H * uavPredictedCovarianceMatrix * H' + R;  % Innovation covariance
            K = uavPredictedCovarianceMatrix * H' / S;  % Kalman gain

            uavUpdatedStateVector = uavPredictedStateVector + K * y;
            I = eye(size(K, 1));
            uavUpdatedCovarianceMatrix = (I - K * H) * uavPredictedCovarianceMatrix;

            if strcmp(fusionAlgorithm,'None')
                self.uavStateVector = uavUpdatedStateVector;
                self.uavCovarianceMatrix = uavUpdatedCovarianceMatrix;
            elseif strcmp(fusionAlgorithm,'CI')
                self.uavStateVectorCI = uavUpdatedStateVector;
                self.uavCovarianceMatrixCI = uavUpdatedCovarianceMatrix;
            elseif strcmp(fusionAlgorithm,'EVCI')
                self.uavStateVectorEVCI = uavUpdatedStateVector;
                self.uavCovarianceMatrixEVCI = uavUpdatedCovarianceMatrix;
            end
        end

        %% Function which constructs the observation matrix H for EKF
        % numUAVs: Number of UAVs
        % measurementLength: Total length of the measurement vector
        % predictedState: Predicted state vector
        function H = constructObservationMatrix(~, numUAVs, measurementLength, predictedState)

            H = zeros(measurementLength, numUAVs * 3); % Initialize H as a sparse matrix

            % Insert identity matrix for GPS measurements (self measurements)
            H(1:3, 1:3) = eye(3); % First three rows map to the first UAV's coordinates

            % Calculate indices for UWB measurements and populate H
            rowIndex = 4; % Start after GPS measurements
            for i = 2:numUAVs
                % Compute distance components for UWB
                dx = predictedState(1) - predictedState((i-1)*3 + 1);
                dy = predictedState(2) - predictedState((i-1)*3 + 2);
                dz = predictedState(3) - predictedState((i-1)*3 + 3);
                range = sqrt(dx^2 + dy^2 + dz^2);

                % Avoid division by zero
                if range == 0
                    range = 1e-6;
                end

                % Populate H with partial derivatives for range measurements
                H(rowIndex, 1) = dx / range; % Partial derivative w.r.t x1
                H(rowIndex, 2) = dy / range; % Partial derivative w.r.t y1
                H(rowIndex, 3) = dz / range; % Partial derivative w.r.t z1
                H(rowIndex, (i-1)*3 + 1) = -dx / range; % Partial derivative w.r.t xi
                H(rowIndex, (i-1)*3 + 2) = -dy / range; % Partial derivative w.r.t yi
                H(rowIndex, (i-1)*3 + 3) = -dz / range; % Partial derivative w.r.t zi

                rowIndex = rowIndex + 1; % Move to next row for next UWB measurement
            end
        end

        %% Funtion which conducts data fusion using classic Covariance Intersection algorithm
        %  Dependency: collectNeighborsData() - gather state and covariance 
        %  from neighbors
        %  covaraianceIntersection() - conduct CI with ulaltered dataset
        function fuseWithNeighborsCI(self)

            % Collect neighbor data
            neighborsData = self.collectNeighborsData();
            % Apply Covariance Intersection
            [fusedState, fusedCovariance] = self.covarianceIntersection(neighborsData);

            % Update own state and covariance with the fused values
            self.uavStateVectorTMP = fusedState;
            self.uavCovarianceMatrixTMP = fusedCovariance;
        end

        %% Funtion which conducts data fusion using EVCI algorithm
        %  Dependency: pcaCompression() - conduct PCA compression and data
        %  reduction
        %  covaraianceIntersection() - conduct CI with prepared dataset
        function fuseWithNeighborsEVCI(self)

            % Get the indices of the neighbors
            neighborIndices = self.swarm.swarmInnerConnections{self.uavIndex};
            swarmWeights = self.swarm.metropolisWeights{self.uavIndex};

            % Initialize cell arrays to store states and covariances
            neighborsData.stateVectors = cell(1, length(neighborIndices) + 1);
            neighborsData.covarianceMatrices = cell(1, length(neighborIndices) + 1);
            neighborsData.weights = cell(1, length(neighborIndices) + 1);

            % Add the state and covariance of the UAV itself
            neighborsData.stateVectors{self.uavIndex} = self.uavStateVectorEVCI;
            neighborsData.covarianceMatrices{self.uavIndex} = self.uavCovarianceMatrixEVCI;
            neighborsData.weights{self.uavIndex} = swarmWeights(self.uavIndex);

            reductionThreshold = 1000000;
            for neighborIndex = neighborIndices
                % Get the neighbor UAV's data (simulated as received data)
                neighborDrone = self.swarm.UAVs(neighborIndex); % Access the neighbor UAV

                % Step 1: Compress the neighbor's data
                [transmittedMatrix, transmittedEigenvalues] = neighborDrone.pcaCompression(reductionThreshold);

                % Step 2: Simulate transmission and reception
                receivedMatrix = transmittedMatrix;

                % Reconstruct or directly use received data
                if ~isempty(receivedMatrix)
                    dataToBeFused = true;
                    numDiscardedComponents = length(self.uavStateVector)-length(transmittedEigenvalues);
                    PapproxNeighbor = self.reconstructCovarianceMatrix(receivedMatrix, numDiscardedComponents);
                    if ~isempty(PapproxNeighbor)
                        neighborsData.stateVectors{neighborIndex} = self.swarm.UAVs(neighborIndex).uavStateVectorEVCI;
                        neighborsData.covarianceMatrices{neighborIndex} = PapproxNeighbor;
                        neighborsData.weights{neighborIndex} = swarmWeights(neighborIndex);
                    end
                end
            end

            % Perform Covariance Intersection or other fusion with valid data
            if dataToBeFused == true
                [fusedState, fusedCovariance] = self.covarianceIntersection(neighborsData);
                % Update own state and covariance with the fused values
                self.uavStateVectorTMP = fusedState;
                self.uavCovarianceMatrixTMP = fusedCovariance;
            end
        end

        %% Funtion which applies Covariance Intersection to fuse multiple state estimates
        %  Input: neighborsData - struct with fields 'states' and 'covariances' from neighboring UAVs
        %  Output: fusedState, fusedCovraiance
        function [fusedState, fusedCovariance] = covarianceIntersection(~, neighborsData)

            nbDronesToBeFused = size(neighborsData.stateVectors,2);

            % Initial fused state and covariance
            fusedState = zeros(size(neighborsData.stateVectors{1}));
            fusedCovariance = zeros(size(neighborsData.covarianceMatrices{1}));

            % Initialize temporary fusion variables
            fusedCovInv = zeros(size(fusedCovariance));

            for fusedDroneIndex = 1:nbDronesToBeFused
                fusedCovInv = fusedCovInv + neighborsData.weights{fusedDroneIndex} * neighborsData.covarianceMatrices{fusedDroneIndex}^(-1);
            end
            % Invert to get the fused covariance matrix
            fusedCovariance = inv(fusedCovInv);

            for fusedDroneIndex = 1:nbDronesToBeFused
                fusedState =  fusedState + neighborsData.weights{fusedDroneIndex} * neighborsData.covarianceMatrices{fusedDroneIndex}^(-1) * neighborsData.stateVectors{fusedDroneIndex};
            end
            fusedState = fusedCovariance * fusedState;
        end

        %% Funtion which gathers the states and covariances from the neighbors
        %  Input: uavIndex, neighborIndex, metropolisWeights
        %  Output: neighborWeight for specific UAV
        % Returns the states and covariances of the drone's neighbors
        function neighborsData = collectNeighborsData(self)

            % Get the indices of the neighbors
            neighborIndices = self.swarm.swarmInnerConnections{self.uavIndex};
            swarmWeights = self.swarm.metropolisWeights{self.uavIndex};

            % Initialize cell arrays to store states and covariances
            neighborsData.stateVectors = cell(1, length(neighborIndices) + 1);
            neighborsData.covarianceMatrices = cell(1, length(neighborIndices) + 1);
            neighborsData.weights = cell(1, length(neighborIndices) + 1);

            % Add the state and covariance of the UAV itself
            neighborsData.stateVectors{self.uavIndex} = self.uavStateVectorCI;
            neighborsData.covarianceMatrices{self.uavIndex} = self.uavCovarianceMatrixCI;
            neighborsData.weights{self.uavIndex} = swarmWeights(self.uavIndex);

            % Collect state and covariance from each neighbor
            for neighborIndex = neighborIndices
                neighborsData.stateVectors{neighborIndex} = self.swarm.UAVs(neighborIndex).uavStateVectorCI;
                neighborsData.covarianceMatrices{neighborIndex} = self.swarm.UAVs(neighborIndex).uavCovarianceMatrixCI;
                neighborsData.weights{neighborIndex} = swarmWeights(neighborIndex);
            end
        end

        %% PCA-based data compression
        %  Input: reductionThreshold
        %  Output: transmittedMatrix - matrix with eigenvector coeefs
        %  intended for transmission
        %  significantEigenvalues - eigenvalues intended for transmission
        function [transmittedMatrix, significantEigenvalues] = pcaCompression(self, reductionThreshold)
  
            % Print the eigenvalues for debugging
            eigenvalues = eig(self.uavCovarianceMatrixEVCI);
            disp('Eigenvalues before PCA:');
            disp(eigenvalues);

            % Perform PCA on the covariance matrix
            [pcaCoefficients,pcaEigenvalues] = pcacov(self.uavCovarianceMatrixEVCI);

            % Determine the number of components to retain
            numSignificant = sum(pcaEigenvalues > reductionThreshold);

            % Transmit only reduced components
            % Select the significant components
            significantComponents = pcaCoefficients(:,(1+numSignificant):size(pcaCoefficients,2));
            significantEigenvalues = pcaEigenvalues((1+numSignificant):length(pcaEigenvalues));

            % Prepare the matrix to be transmitted
            transmittedMatrix = significantComponents * sqrt(diag(significantEigenvalues));
        end

        %% Reconstruct the covariance matrix from received data
        %  Input: receivedMatrix - matrix received from neighbor UAV (as
        %  transmittedMatrix in pcaCompression method)
        %  numDiscardedComponents - difference between a full vector of
        %  eigenvalues and significant ones
        %  Output: reconstructedCovarianceMatrix
        function reconstructedCovarianceMatrix = reconstructCovarianceMatrix(~, receivedMatrix, numDiscardedComponents)
            % Calculate the orthonormal basis for the null space of the received matrix
            insertedCoefficients = null(receivedMatrix');

            % Create the artificial eigenvalues for the discarded components
            insertedEigenvalues = 1e6 * ones(numDiscardedComponents, 1);

            % Create the matrix C
            C = insertedCoefficients * diag(insertedEigenvalues) * insertedCoefficients';

            % Create the matrix A from the received matrix
            A = receivedMatrix * receivedMatrix';

            % Reconstruct the approximate covariance matrix
            reconstructedCovarianceMatrix = C + A;
        end
    end
end

