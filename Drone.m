%% DRONE CLASS
% ===========
%
% Table of Contents
% -----------------
%
% Properties:
%   - UAV Identification and Scenario Variables:
%       - swarm: Reference to the Swarm object containing all UAVs
%       - swarmParameters: Parameters related to the swarm
%       - uavPlatform: UAV platform (Swarm scenario element)
%       - uavFlightData: UAV platform data in scenario
%       - uavIndex: Number of the UAV in the swarm
%       - uavSI: The starting index for the UAV's coordinates in the state vector
%
%   - UAV True State:
%       - uavMotionVector: UAV motion vector with 16 elements [x, y, z, vx, vy, vz, ax, ay, az, q1, q2, q3, q4, wx, wy, wz]
%       - uavTruePosition: 3D true position [X, Y, Z]
%       - uavTrueVelocity: 3D true velocity [Vx, Vy, Vz]
%       - uavTrueOrientation: 4-element quaternion vector [q1, q2, q3, q4]
%       - uavLLAVector: 3D position [Latitude, Longitude, Altitude]
%       - uavTrueLatitude: True latitude of the UAV
%       - uavTrueLongitude: True longitude of the UAV
%       - uavTrueAltitude: True altitude of the UAV
%       - uavTruePitch: True pitch angle of the UAV
%       - uavTrueRoll: True roll angle of the UAV
%       - uavTrueYaw: True yaw angle of the UAV
%       - uavTrueGroundspeed: True groundspeed of the UAV (calculated from true velocity)
%
%   - GPS Sensor Data:
%       - gps: GPS sensor object (gpsSensor object)
%       - gpsModule: Mounted GPS sensor module (uavSensor object)
%       - gpsMeasurements: GPS measured data
%       - gpsMeasurementsENU: GPS measurements in ENU (East-North-Up) coordinates
%       - gpsPosition: GPS measured position
%       - gpsVelocity: GPS measured velocity
%       - gpsGroundspeed: GPS measured groundspeed
%       - gpsCourse: GPS course over ground
%       - gpsIsUpdated: Flag indicating if GPS data is updated
%       - gpsSampleTime: Timestamp of the GPS measurement
%
%   - UWB Sensor Data:
%       - uwbMeasurements: UWB range measurements to neighboring UAVs
%
%   - State Variables:
%       - uavStateVector: State vector used for estimation [position, velocity, orientation]
%       - uavCovarianceMatrix: Covariance matrix corresponding to the state vector
%       - uavStateVectorCI: Fused state vector after Covariance Intersection (CI)
%       - uavCovarianceMatrixCI: Fused covariance matrix after CI
%       - uavStateVectorEVCI: Fused state vector after Eigenvalue-based Covariance Intersection (EVCI)
%       - uavCovarianceMatrixEVCI: Fused covariance matrix after EVCI
%
%   - Logging:
%       - evciReductionMetrics: Cell array storing the number of significant components during EVCI
%
% Methods:
% --------
%
% Constructor:
%   - Drone: Initializes the Drone object with simulation scene, flight data, swarm, and swarm parameters
%
% Sensor Functions:
%   - gpsMount: Defines the GPS model and mounts the GPS sensor on the UAV
%   - gpsConductMeasurement: Reads the GPS sensor measurements
%   - uwbConductMeasurement: Simulates UWB range measurements from the UAV to its neighbors
%
% UAV State Functions:
%   - transformOrientation: Transforms the true orientation from quaternion to yaw, pitch, roll angles
%   - calculateGroundspeed: Calculates the true groundspeed from the velocity components
%   - updateNavData: Updates the true UAV data in a timestep
%
% EKF Functions:
%   - extendedKalmanFilter: Applies the Extended Kalman Filter update using GPS and UWB measurements
%   - constructObservationMatrix: Constructs the observation matrix H for the EKF
%
% Data Fusion Functions:
%   - fuseWithNeighborsCI: Fuses the UAV's state with its neighbors using Covariance Intersection (CI)
%   - fuseWithNeighborsEVCI: Fuses data with neighbors using PCA-based Covariance Intersection (EVCI)
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
%
classdef Drone < handle
    properties
        % UAV Identification and Scenario Variables
        swarm               % Reference to the Swarm object containing all UAVs
        swarmParameters     % Parameters related to the swarm
        uavPlatform         % UAV platform (Swarm scenario element)
        uavFlightData       % UAV platform data in scenario
        uavIndex            % Number of the UAV in the swarm
        uavSI               % Starting index for the UAV's coordinates in the state vector

        % UAV True State
        uavMotionVector     % UAV motion vector with 16 elements
                            % [x y z vx vy vz ax ay az q1 q2 q3 q4 wx wy wz]
        uavTruePosition     % 3D true position [X, Y, Z]
        uavTrueVelocity     % 3D true velocity [Vx, Vy, Vz]
        uavTrueOrientation  % 4-element quaternion vector [q1, q2, q3, q4]
        uavLLAVector        % 3D position [Latitude, Longitude, Altitude]
        uavTrueLatitude     % True latitude of the UAV
        uavTrueLongtitude   % True longtitude of the UAV
        uavTrueAltitude     % True altitude of the UAV
        uavTruePitch        % True pitch angle of the UAV
        uavTrueRoll         % True roll angle of the UAV
        uavTrueYaw          % True yaw angle of the UAV
        uavTrueGroundspeed  % True groundspeed of the UAV (calculated from true velocity)

        % GPS Sensor Model
        gps                 % GPS sensor object (gpsSensor object)
        gpsModule           % Mounted GPS sensor module (uavSensor object)
        gpsMeasurements     % GPS measurements
        gpsMeasurementsENU  % GPS measurements in ENU coordinates

        % GPS Data
        gpsPosition         % GPS measured position
        gpsVelocity         % GPS measured velocity
        gpsGroundspeed      % GPS measured groundspeed
        gpsCourse           % GPS course over ground
        gpsIsUpdated        % Flag indicating if GPS data is updated
        gpsSampleTime       % Timestamp of the GPS measurement

        % UWB Sensor Data
        uwbMeasurements     % UWB range measurements to neighboring UAVs

        % State Variables
        uavStateVector          % State vector used for estimation
        uavCovarianceMatrix     % Covariance matrix corresponding to the state vector

        % Fused State Variables for Covariance Intersection (CI)
        uavStateVectorCI        % Fused state vector after Covariance Intersection (CI)
        uavCovarianceMatrixCI   % Fused covariance matrix after CI

        % Fused State Variables for Eigenvalue-based Covariance Intersection (EVCI)
        uavStateVectorEVCI      % Fused state vector after Eigenvalue-based Covariance Intersection (EVCI)
        uavCovarianceMatrixEVCI % Fused covariance matrix after EVCI

        % Logging Variables
        evciReductionMetrics % Cell array storing the number of significant components during EVCI
    end

    methods
        %%%%%%%%%% Constructor %%%%%%%%%%%%
        function self = Drone(simulationScene, uavFlightData, swarm, swarmParameters)
            % Initialize properties based on input parameters
            self.uavFlightData = uavFlightData;
            self.uavIndex = str2double(regexp(uavFlightData.Name, '\d+', 'match'));
            self.uavSI = (self.uavIndex - 1) * 3 + 1;
            self.swarm = swarm;
            self.swarmParameters = swarmParameters;

            % Initialize UAV platform
            self.uavPlatform = uavPlatform(uavFlightData.Name, simulationScene, ...
                ReferenceFrame = uavFlightData.ReferenceFrame, ...
                Trajectory = uavFlightData.Trajectory);

            % Read initial motion vector and LLA vector from UAV platform
            [self.uavMotionVector, self.uavLLAVector] = read(self.uavPlatform);

            % Extract position, velocity, and orientation from motion vector
            self.uavTruePosition = self.uavMotionVector(1, 1:3);
            self.uavTrueVelocity = self.uavMotionVector(1, 4:6);
            self.uavTrueOrientation = self.uavMotionVector(1, 10:13);

            % Extract latitude, longitude, altitude from LLA vector
            self.uavTrueLatitude = self.uavLLAVector(1);
            self.uavTrueLongtitude = self.uavLLAVector(2);
            self.uavTrueAltitude = self.uavLLAVector(3);

            % Convert orientation from quaternion to yaw, pitch, roll
            self.transformOrientation();

            % Calculate groundspeed from velocity components
            self.calculateGroundspeed();

            % Initialize state vector and covariance matrix for estimation
            self.uavStateVector = zeros(self.swarmParameters.nbAgents * 3, 1);
            self.uavCovarianceMatrix = eye(self.swarmParameters.nbAgents * 3) * 1000;

            % Set initial position in state vector
            self.uavStateVector(self.uavSI:self.uavSI+2) = self.uavTruePosition';

            % Initialize CI and EVCI state vectors and covariance matrices
            self.uavStateVectorCI = self.uavStateVector;
            self.uavStateVectorEVCI = self.uavStateVector;

            self.uavCovarianceMatrixCI = self.uavCovarianceMatrix;
            self.uavCovarianceMatrixEVCI = self.uavCovarianceMatrix;

            % Update UAV platform mesh with initial orientation and position
            updateMesh(self.uavPlatform, "quadrotor", {10}, [1 0 0], self.uavTruePosition, self.uavTrueOrientation);
        end
        %%%%%%%%%% End Constructor %%%%%%%%%%%%

        %% Function which is responsible for definining GPS model and mounting GPS sensor on UAV
        %  Output: GPS, GPSsensor
        function gpsMount(self)
            self.gps = gpsSensor('SampleRate',2,'PositionInputFormat','Local','ReferenceLocation',[0 0 0]);
            self.gpsModule = uavSensor('GPS',self.uavPlatform,self.gps,'MountingLocation',[0 0.1 0],'UpdateRate',1);
        end

        %% Function which is responsible for reading GSP sensor measurements
        %  Output: gpsPosition, gpsVelocity, gpsGroundspeed, gpsCourse
        function gpsConductMeasurement(self)
            [self.gpsIsUpdated,self.gpsSampleTime, self.gpsPosition,...
                self.gpsVelocity,self.gpsGroundspeed,self.gpsCourse] = ...
                read(self.gpsModule);
            self.gpsMeasurements = self.gpsPosition';
        end

        %% Function to convert GPS spherical coordinates from to ECEF 
        %  (Earth-Centered, Earth-Fixed) coordinates to get a global Cartesian position
        %  Input: gpsCoordinates - [Lat Long Alt] vector
        %  Output: [X,Y,Z] - ECEF coordinates
        function [X, Y, Z] = gpsToECEF(~,gpsCoordinates)
            % gpsCoordinates is a vector [lat, lon, alt]
            lat = gpsCoordinates(1);
            lon = gpsCoordinates(2);
            alt = gpsCoordinates(3);

            % WGS84 ellipsoid constants
            a = 6378137.0; % Semi-major axis in meters
            f = 1 / 298.257223563; % Flattening
            e2 = 2*f - f^2; % Square of eccentricity

            % Convert latitude and longitude from degrees to radians
            lat_rad = deg2rad(lat);
            lon_rad = deg2rad(lon);

            % Calculate prime vertical radius of curvature
            N = a / sqrt(1 - e2 * sin(lat_rad)^2);

            % Calculate ECEF coordinates
            X = (N + alt) * cos(lat_rad) * cos(lon_rad);
            Y = (N + alt) * cos(lat_rad) * sin(lon_rad);
            Z = (N * (1 - e2) + alt) * sin(lat_rad);
        end

        %% Function transform ECEF coordinates to the local ENU coordinate 
        %  system relative to the reference point
        %  Input: gpsCoordinates - [Lat Long Alt] vector
        %  Output: gpsMeasurementsENU - ENU [x y z] coordinates
        function gpsToENU(self, gpsCoordinates)

            refCoordinates = self.swarm.swarmSimulationScene.ReferenceLocation;
            % refCoordinates and gpsCoordinates are vectors [lat, lon, alt]
            latRef = refCoordinates(1);
            lonRef = refCoordinates(2);

            % Convert reference point to ECEF
            [Xr, Yr, Zr] = gpsToECEF(self,refCoordinates);

            % Convert GPS point to ECEF
            [X, Y, Z] = gpsToECEF(self,gpsCoordinates);

            % Compute offsets from the reference point
            dx = X - Xr;
            dy = Y - Yr;
            dz = Z - Zr;

            % Convert lat/lon reference point to radians
            latRef_rad = deg2rad(latRef);
            lonRef_rad = deg2rad(lonRef);

            % Transformation matrix from ECEF to ENU
            t = [-sin(lonRef_rad), cos(lonRef_rad), 0;
                -sin(latRef_rad) * cos(lonRef_rad), -sin(latRef_rad) * sin(lonRef_rad), cos(latRef_rad);
                cos(latRef_rad) * cos(lonRef_rad), cos(latRef_rad) * sin(lonRef_rad), sin(latRef_rad)];

            % Apply the transformation
            enu = t * [dx; dy; dz];

            % Extract ENU coordinates
            self.gpsMeasurementsENU(1,1) = enu(2);
            self.gpsMeasurementsENU(2,1) = enu(1);
            self.gpsMeasurementsENU(3,1) = enu(3);
        end

        %% Function which simulates UWB range measurements to neighbors
        %  Output: uwbMeasurements - vector of ranges to UAV neighbors
        function uwbConductMeasurement(self)

            % Get neighbors from the neighbors cell array
            selfNeighbors = self.swarm.swarmInnerConnections{self.uavIndex};

            % Initialize range measurements vector
            ranges = zeros(length(selfNeighbors),1);

            measurementError = 0;

            % Coordinates of the UAV conducting the measurements
            x = self.swarm.swarmTruePositions((self.uavIndex-1)*3 + 1);
            y = self.swarm.swarmTruePositions((self.uavIndex-1)*3 + 2);
            z = self.swarm.swarmTruePositions((self.uavIndex-1)*3 + 3);

            % Calculate the range to each neighbor
            for i = 1:length(selfNeighbors)
                neighborIndex = selfNeighbors(i);

                % Coordinates of the neighbor UAV
                x_other = self.swarm.swarmTruePositions((neighborIndex-1)*3 + 1);
                y_other = self.swarm.swarmTruePositions((neighborIndex-1)*3 + 2);
                z_other = self.swarm.swarmTruePositions((neighborIndex-1)*3 + 3);

                % Calculate true range
                trueRange = sqrt((x - x_other)^2 + (y - y_other)^2 + (z - z_other)^2);

                % Add measurement noise
                noisyRange = trueRange + measurementError * randn();

                % Store the noisy range
                ranges(i) = noisyRange;
            end
            self.uwbMeasurements = ranges;
        end

        %% Function which is responsible for transforming true orientation of
        %  the UAV from quaternion (uavOrientation) to yaw, pitch, roll angles
        %  Output: uavYaw, uawPitch, uavRoll
        function transformOrientation(self)
            [self.uavTrueYaw, self.uavTruePitch, self.uavTrueRoll] = ...
                quat2angle(self.uavTrueOrientation);
        end

        %% Function which is resposnible for calculating true groudspeed of
        %  the UAV from velocities components in xyz dimensions
        %  Output: uavGroundspeed*
        function calculateGroundspeed(self)
            self.uavTrueGroundspeed = sqrt(sum(self.uavTrueVelocity.^2));
        end

        %% Function responsible for updating true data of the UAV in a timestep
        %  Output: uavMotionVector, uavPosition, uavVelocity,
        %  uavOrientation,uavLatitude, uavLongtitude, uavAltitude, uavYaw,
        %  uawPitch, uavRoll, uavGroudspeed
        function updateNavData(self)
            [self.uavMotionVector,self.uavLLAVector] = read(self.uavPlatform);
            self.uavTruePosition = self.uavMotionVector(1,1:3);
            self.uavTrueVelocity = self.uavMotionVector(1,4:6);
            self.uavTrueOrientation = self.uavMotionVector(1,10:13);

            self.uavTrueLatitude = self.uavLLAVector(1);
            self.uavTrueLongtitude = self.uavLLAVector(2);
            self.uavTrueAltitude = self.uavLLAVector(3);

            self.transformOrientation();
            self.calculateGroundspeed();
        end

        %% Function which applies the EKF update based on GPS and UWB measurements
        %  Input: fusionAlgorithm - algorithm for which EKF conducts
        %  estimation (None, CI, EVCI) - based on this parameter results
        %  are assigned to the proper variables
        function extendedKalmanFilter(self, fusionAlgorithm)

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
            Q = 1000 * eye(length(stateVector));

            %%%%%%%%%% Prediction %%%%%%%%%%%%
            uavPredictedStateVector = F * stateVector;
            uavPredictedCovarianceMatrix = F * covarianceMatrix * F' + Q;

            nbAgents = self.swarmParameters.nbAgents;

            %%%%%%%%%% Measurement %%%%%%%%%%%%
            self.gpsConductMeasurement();
            self.gpsToENU(self.gpsMeasurements);
            self.uwbConductMeasurement();

            z = [self.gpsMeasurementsENU; self.uwbMeasurements]; % Build the full measurement vector z

            %%%%%%%%%% H definition %%%%%%%%%%%%
            H = self.constructObservationMatrix(nbAgents, length(z), uavPredictedStateVector);

            % Measurement noise covariance (R)
            R_gps = 0.1 * eye(length(self.gpsMeasurements));
            R_uwb = 0.2 * eye(length(self.uwbMeasurements));
            R = blkdiag(R_gps, R_uwb); % Block diagonal matrix combining GPS and UWB noise

            %%%%%%%%%% Update %%%%%%%%%%%%
            hz = zeros(size(z));
            hz(1:3) = uavPredictedStateVector((self.uavIndex-1)*3 + 1:self.uavIndex*3);

            % List of neighbors for the current UAV (self.uavIndex)
            neighborsList = self.swarm.swarmInnerConnections{self.uavIndex};

            % Start after GPS measurements (i.e., from row 4 onwards)
            rowIndex = 4;

            % Populate H with partial derivatives for each neighbor (UWB measurements)
            for neighborIndex = 1:length(neighborsList)
                neighborUAV = neighborsList(neighborIndex);  % Get the neighbor UAV index

                % Calculate the distance components for UWB measurement
                dx = uavPredictedStateVector((self.uavIndex-1)*3 + 1) - uavPredictedStateVector((neighborUAV-1)*3 + 1);
                dy = uavPredictedStateVector((self.uavIndex-1)*3 + 2) - uavPredictedStateVector((neighborUAV-1)*3 + 2);
                dz = uavPredictedStateVector((self.uavIndex-1)*3 + 3) - uavPredictedStateVector((neighborUAV-1)*3 + 3);
                hz(rowIndex) = sqrt(dx^2 + dy^2 + dz^2);

                % Move to the next row for the next UWB measurement
                rowIndex = rowIndex + 1;
            end

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
        function H = constructObservationMatrix(self, nbAgents, measurementLength, predictedState)
            % Initialize H with zeros
            H = zeros(measurementLength, nbAgents * 3);

            % Insert identity matrix for GPS measurements (self measurements)
            % First three rows map to the first UAV's coordinates
            H(1:3, (self.uavIndex-1)*3 + 1:self.uavIndex*3) = eye(3);

            % List of neighbors for the current UAV (self.uavIndex)
            neighborsList = self.swarm.swarmInnerConnections{self.uavIndex};

            % Start after GPS measurements (i.e., from row 4 onwards)
            rowIndex = 4;

            % Populate H with partial derivatives for each neighbor (UWB measurements)
            for neighborIndex = 1:length(neighborsList)
                neighborUAV = neighborsList(neighborIndex);  % Get the neighbor UAV index

                % Calculate the distance components for UWB measurement
                dx = predictedState((self.uavIndex-1)*3 + 1) - predictedState((neighborUAV-1)*3 + 1);
                dy = predictedState((self.uavIndex-1)*3 + 2) - predictedState((neighborUAV-1)*3 + 2);
                dz = predictedState((self.uavIndex-1)*3 + 3) - predictedState((neighborUAV-1)*3 + 3);
                range = sqrt(dx^2 + dy^2 + dz^2);

                % Avoid division by zero
                if range == 0
                    range = 1e-6;
                end

                % Populate H with partial derivatives for range measurement
                % Partial derivatives with respect to the current UAV (self.uavIndex)
                H(rowIndex, (self.uavIndex-1)*3 + 1) = dx / range; % ∂range/∂x_self
                H(rowIndex, (self.uavIndex-1)*3 + 2) = dy / range; % ∂range/∂y_self
                H(rowIndex, (self.uavIndex-1)*3 + 3) = dz / range; % ∂range/∂z_self

                % Partial derivatives with respect to the neighbor UAV
                H(rowIndex, (neighborUAV-1)*3 + 1) = -dx / range; % ∂range/∂x_neighbor
                H(rowIndex, (neighborUAV-1)*3 + 2) = -dy / range; % ∂range/∂y_neighbor
                H(rowIndex, (neighborUAV-1)*3 + 3) = -dz / range; % ∂range/∂z_neighbor

                % Move to the next row for the next UWB measurement
                rowIndex = rowIndex + 1;
            end
        end

        %% Funtion which conducts data fusion using classic Covariance Intersection algorithm
        %  Dependency: collectNeighborsData() - gather state and covariance from neighbors
        %  covaraianceIntersection() - conduct CI with ulaltered dataset
        function [fusedState, fusedCovariance] = fuseWithNeighborsCI(self)

            % Get the indices of the neighbors
            neighborIndices = self.swarm.swarmInnerConnections{self.uavIndex};
            if ~isempty(neighborIndices)
                swarmWeights = self.swarm.swarmMetropolisWeights{self.uavIndex};

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

                % Apply Covariance Intersection
                [fusedState, fusedCovariance] = self.covarianceIntersection(neighborsData);
            end
        end

        %% Funtion which conducts data fusion using EVCI algorithm
        %  Dependency: pcaCompression() - conduct PCA compression and data reduction
        %  covaraianceIntersection() - conduct CI with prepared dataset
        function [fusedState, fusedCovariance] = fuseWithNeighborsEVCI(self)

            % Get the indices of the neighbors
            neighborIndices = self.swarm.swarmInnerConnections{self.uavIndex};
            if ~isempty(neighborIndices)
                swarmWeights = self.swarm.swarmMetropolisWeights{self.uavIndex};

                % Initialize cell arrays to store states and covariances
                neighborsData.stateVectors = cell(1, length(neighborIndices) + 1);
                neighborsData.covarianceMatrices = cell(1, length(neighborIndices) + 1);
                neighborsData.weights = cell(1, length(neighborIndices) + 1);

                % Add the state and covariance of the UAV itself
                neighborsData.stateVectors{self.uavIndex} = self.uavStateVectorEVCI;
                neighborsData.covarianceMatrices{self.uavIndex} = self.uavCovarianceMatrixEVCI;
                neighborsData.weights{self.uavIndex} = swarmWeights(self.uavIndex);

                retainedComponentsLog = []; % Initialize logging for significant components

                reductionThreshold = 10000;
                for neighborIndex = neighborIndices
                    % Get the neighbor UAV's data (simulated as received data)
                    neighborDrone = self.swarm.UAVs(neighborIndex); % Access the neighbor UAV

                    % Step 1: Compress the neighbor's data
                    [transmittedMatrix, transmittedEigenvalues] = ...
                        neighborDrone.pcaCompression(self.swarm.swarmParameters.evciReductionThreshold);
                    
                    % Step 2: Simulate transmission and reception
                    receivedMatrix = transmittedMatrix;

                    % Log the number of significant components retained
                    retainedComponentsLog = [retainedComponentsLog, length(transmittedEigenvalues)];

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

                % Log the data reduction metrics for this step
                self.evciReductionMetrics{end+1} = retainedComponentsLog;

                % Perform Covariance Intersection or other fusion with valid data
                if dataToBeFused == true
                    [fusedState, fusedCovariance] = self.covarianceIntersection(neighborsData);
                end
            end
        end

        %% Funtion which applies Covariance Intersection to fuse multiple state estimates
        %  Input: neighborsData - struct with fields 'states' and 'covariances' from neighboring UAVs
        %  Output: fusedState, fusedCovraiance
        function [fusedState, fusedCovariance] = covarianceIntersection(self, neighborsData)

            neighborIndices = self.swarm.swarmInnerConnections{self.uavIndex};
            dronesToBeFused = [self.uavIndex neighborIndices];

            % Initial fused state and covariance
            fusedState = zeros(size(neighborsData.stateVectors{self.uavIndex}));
            fusedCovariance = zeros(size(neighborsData.covarianceMatrices{self.uavIndex}));

            % Initialize temporary fusion variables
            fusedCovInv = zeros(size(fusedCovariance));

            for fusedDroneIndex = dronesToBeFused
                fusedCovInv = fusedCovInv + neighborsData.weights{fusedDroneIndex} * neighborsData.covarianceMatrices{fusedDroneIndex}^(-1);
            end
            % Invert to get the fused covariance matrix
            fusedCovariance = inv(fusedCovInv);

            for fusedDroneIndex = dronesToBeFused
                fusedState =  fusedState + neighborsData.weights{fusedDroneIndex} * neighborsData.covarianceMatrices{fusedDroneIndex}^(-1) * neighborsData.stateVectors{fusedDroneIndex};
            end
            fusedState = fusedCovariance * fusedState;
        end

        %% Funtion which gathers the states and covariances from the neighbors
        %  Output: neighborsData - the states, covariances, weights of the UAV and its neighbors
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
        %  Input:   reductionThreshold
        %  Output:  transmittedMatrix - matrix with eigenvector coeefs intended for transmission
        %           significantEigenvalues - eigenvalues intended for transmission
        function [transmittedMatrix, significantEigenvalues] = pcaCompression(self, reductionThreshold)

            % Print the eigenvalues for debugging
            eigenvalues = eig(self.uavCovarianceMatrixEVCI);
            disp('Eigenvalues before PCA:');
            disp(eigenvalues);

            % Ensure that covariance matrix is semi
            covarianceMatrix = nearestSPD(self.uavCovarianceMatrixEVCI);

            % Perform PCA on the covariance matrix
            [pcaCoefficients,pcaEigenvalues] = pcacov(covarianceMatrix);
            
            pcaEigenvalues
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
        %  Input:   receivedMatrix - matrix received from neighbor UAV (as
        %           transmittedMatrix in pcaCompression method)
        %           numDiscardedComponents - difference between a full vector of eigenvalues and significant ones
        %  Output:  reconstructedCovarianceMatrix
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


