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
%   - Auxiliary Variables:
%       - uavEigenvaluesPosToTransEVCI: Vector of significant eigenvalues related to positions
%       - uavEigenvaluesVelToTransEVCI: Vector of significant eigenvalues related to velocities
%       - uavCovarianceMatrixPosToTransEVCI: Coefficients matrix related to positions to be transmitted
%       - uavCovarianceMatrixVelToTransEVCI: Voefficients matrix related to positions to be transmitted
%       - uavPosReductionThreshold: 
%       - uavVelReductionThreshold: 
%       - uavEstimationModel: Variable telling which estimation model is utilized P or PV
%       - uavVelocityOffset: Variable specifying the number of coordinates in the state vector before the velocity components
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
%   - gpsAddNoise: Modify gpsSensor model in order to apply additonal noise
%   - gpsDeleteNoise: Restore original settings of gpsSensor (neglect additional noise)
%   - uwbConductMeasurement: Simulates UWB range measurements from the UAV to its neighbors
%
% UAV State Functions:
%   - transformOrientation: Transforms the true orientation from quaternion to yaw, pitch, roll angles
%   - calculateGroundspeed: Calculates the true groundspeed from the velocity components
%   - updateNavData: Updates the true UAV data in a timestep
%
% EKF Functions:
%   - extendedKalmanFilter: Applies the Extended Kalman Filter update using GPS and UWB measurements
%   - calculateSwarmModel: Function which calculates estimation variables (Swarm model)
%   - constructObservationMatrix: Constructs the observation matrix H for the EKF
%
% Data Fusion Functions:
%   - fuseWithNeighborsCI: Fuses the UAV's state with its neighbors using Covariance Intersection (CI)
%   - fuseWithNeighborsEVCI: Fuses data with neighbors using PCA-based Covariance Intersection (EVCI)
%   - covarianceIntersection: Applies Covariance Intersection to fuse multiple state estimates
%   - prepareDataToBeSentEVCI: Function which conducts PCA compression and prepares data to be sent to another nodes
%   - collectNeighborsData: Collects state and covariance data from the UAV's neighbors
%   - pcaCompression: Performs PCA-based data compression on the covariance matrix
%   - determineReductionThreshold: Determinines reduction threshold in the EVCI algorithm (under development)
%   - reconstructCovarianceMatrix: Reconstructs the covariance matrix from received PCA-compressed data
%
% Mesh Update:
%   - updateMesh: Sets up and updates the platform mesh with position and orientation
%
% Data Visualization Functions:       
%   - plotEllipsoids: Uncertainty ellipsoids plotting
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

        % Auxiliary variables
        uavEigenvaluesPosToTransEVCI   % Vector of significant eigenvalues related to positions
        uavEigenvaluesVelToTransEVCI  %Vector of significant eigenvalues related to velocities
        uavCovarianceMatrixPosToTransEVCI    %Coefficients matrix related to positions to be transmitted
        uavCovarianceMatrixVelToTransEVCI   % Coefficients matrix related to positions to be transmitted
        uavPosReductionThreshold    % EVCI positions reduction threshold  
        uavVelReductionThreshold    % EVCI velocities reduction threshold
        uavEstimationModel      % Variable telling which estimation model is utilized P or PV
        uavVelocityOffset       % Variable specifying the number of coordinates in the
                                % state vector before the velocity components

        % Logging Variables
        evciReductionMetrics % Cell array storing the number of significant components during EVCI
    end

    methods
        %%%%%%%%%% Constructor %%%%%%%%%%%%
        function self = Drone(simulationScene, uavFlightData, swarm, swarmParameters)
            % Initialize properties based on input parameters
            self.uavFlightData = uavFlightData;
            self.uavIndex = str2double(regexp(uavFlightData.Name, '\d+', 'match'));
            self.uavSI = (self.uavIndex - 1) * 3 + 1; % Set UAV coordinates' starting index in the state vector
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

            % Determine which estimation model will be utilized
            self.uavEstimationModel = self.swarmParameters.estimationModel;

            % Initialize state vector and covariance matrix for estimation
            if strcmp(self.uavEstimationModel,'P')

                self.uavStateVector = zeros(self.swarmParameters.nbAgents * 3, 1);
                self.uavCovarianceMatrix = eye(self.swarmParameters.nbAgents * 3) * 1000;
                self.uavVelocityOffset = 0;

                % Set initial position in state vector
                self.uavStateVector(self.uavSI:self.uavSI+2) = self.uavTruePosition';

            elseif strcmp(self.uavEstimationModel,'PV')

                self.uavStateVector = zeros(self.swarmParameters.nbAgents * 6, 1);
                self.uavCovarianceMatrix = eye(self.swarmParameters.nbAgents * 6) * 1000;
                self.uavVelocityOffset = self.swarmParameters.nbAgents * 3;

                % Set initial position in state vector
                self.uavStateVector(self.uavSI:self.uavSI+2) = self.uavTruePosition';
                self.uavStateVector(self.uavVelocityOffset+self.uavSI:self.uavVelocityOffset+self.uavSI+2) = self.uavTrueVelocity';
            end

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

        %% Function which modify gpsSensor model in order to apply noise
        function gpsAddNoise(self)
            % self.uavPlatform.Sensors(1).SensorModel.DecayFactor = 0;
            self.uavPlatform.Sensors(1).SensorModel.HorizontalPositionAccuracy = 20;
        end

        %% Function which modify gpsSensor model in delete noise and
        %  restore original settings
        function gpsDeleteNoise(self)
            % self.uavPlatform.Sensors(1).SensorModel.DecayFactor = 0.999;
            self.uavPlatform.Sensors(1).SensorModel.HorizontalPositionAccuracy = 1.6;

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
            self.uavTruePosition(3) = - self.uavTruePosition(3);
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

            [Phi,Q] = calculateSwarmModel(self);

            %%%%%%%%%% Prediction %%%%%%%%%%%%
            uavPredictedStateVector = Phi * stateVector;
            uavPredictedCovarianceMatrix = Phi * covarianceMatrix * Phi' + Q;

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
                self.prepareDataToBeSentEVCI();
            end
        end

        %% Function which calculates estimation variables (Swarm model)
        %  Output: Phi - transistion matrix, Q - covariance matrix of
        %  discrete disturbances
        function [Phi,Q] = calculateSwarmModel(self)
            % Determine basic quantities
            S = 10;
            F = eye(length(self.uavStateVector));
            T = 1/self.swarm.swarmSimulationScene.UpdateRate;

            Q = 10 * eye(length(self.uavStateVector));

            switch self.uavEstimationModel
                case 'P'
                    Phi = F;
                    Q = S * T * eye(length(self.uavStateVector));
                case 'PV'
                    Phi = eye(length(self.uavStateVector));
                    Phi(1:self.uavVelocityOffset, self.uavVelocityOffset+1:end) = T * eye(self.uavVelocityOffset);

                    Q11 = (S * T^3 / 3) * eye(self.uavVelocityOffset);
                    Q12 = (S * T^2 / 2) * eye(self.uavVelocityOffset);
                    Q21 = Q12;
                    Q22 = (S * T) * eye(self.uavVelocityOffset);

                    % Assemble the full Q matrix
                    Q = [Q11, Q12; Q21, Q22];
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
            if strcmp(self.uavEstimationModel,'PV')
                H = [H zeros(measurementLength, nbAgents * 3)];
            end
        end

        %% Function which conducts PCA compression and prepares data to be sent to another nodes
        %  Output: uavCovarianceMatrixPosToTransEVCI, uavEigenvaluesPosToTransEVCI
        %          uavCovarianceMatrixVelToTransEVCI, uavEigenvaluesVelToTransEVCI
        function prepareDataToBeSentEVCI(self)
            switch self.uavEstimationModel
                case 'P'
                    % Compress data
                    [self.uavCovarianceMatrixPosToTransEVCI, self.uavEigenvaluesPosToTransEVCI] = ...
                        self.pcaCompression(self.uavCovarianceMatrixEVCI,self.uavCovarianceMatrixEVCI,...
                        self.swarm.swarmParameters.evciPosReductionThreshold,"Positions");
                    % Log the data reduction metrics for this step
                    self.evciReductionMetrics{end+1} = length(self.uavEigenvaluesPosToTransEVCI);
                case 'PV'
                    % Compress data
                    stateVectorPositions = self.uavStateVectorEVCI(1:self.uavVelocityOffset);

                    covarianceMatrixPositions = ...
                        self.uavCovarianceMatrixEVCI(1:self.uavVelocityOffset,1:self.uavVelocityOffset);

                    stateVectorVelocities = self.uavStateVectorEVCI(self.uavVelocityOffset+1:end);

                    covarianceMatrixVelocities = ...
                        self.uavCovarianceMatrixEVCI(self.uavVelocityOffset+1:end,self.uavVelocityOffset+1:end);

                    [self.uavCovarianceMatrixPosToTransEVCI, self.uavEigenvaluesPosToTransEVCI] = ...
                        self.pcaCompression(stateVectorPositions,covarianceMatrixPositions,...
                        self.swarm.swarmParameters.evciPosReductionThreshold,"Positions");

                    [self.uavCovarianceMatrixVelToTransEVCI, self.uavEigenvaluesVelToTransEVCI] = ...
                        self.pcaCompression(stateVectorVelocities,covarianceMatrixVelocities,...
                        self.swarm.swarmParameters.evciVelReductionThreshold,"Velocitites");

                    % Log the data reduction metrics for this step
                    self.evciReductionMetrics{end+1} = length(self.uavEigenvaluesPosToTransEVCI) +...
                        length(self.uavEigenvaluesVelToTransEVCI);
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
                [fusedState, fusedCovariance] = self.covarianceIntersection(neighborsData,'CI');
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

                switch self.uavEstimationModel
                    case 'P'
                        % Initialize cell arrays to store states and covariances
                        neighborsData.stateVectors = cell(1, length(neighborIndices) + 1);
                        neighborsData.covarianceMatrices = cell(1, length(neighborIndices) + 1);
                        neighborsData.weights = cell(1, length(neighborIndices) + 1);

                        % Add the state and covariance of the UAV itself
                        neighborsData.stateVectors{self.uavIndex} = self.uavStateVectorEVCI;
                        neighborsData.covarianceMatrices{self.uavIndex} = self.uavCovarianceMatrixEVCI;
                        neighborsData.weights{self.uavIndex} = swarmWeights(self.uavIndex);

                        for neighborIndex = neighborIndices
                            % Get the neighbor UAV's data (simulated as received data)
                            neighborDrone = self.swarm.UAVs(neighborIndex); % Access the neighbor UAV
                            dataToBeFused = true;

                            % Simulate transmission and reception
                            receivedMatrixPositions = neighborDrone.uavCovarianceMatrixPosToTransEVCI;

                            % Reconstruct or directly use received data
                            numDiscardedComponents = length(self.uavStateVector)-length(neighborDrone.uavEigenvaluesPosToTransEVCI);
                            PapproxNeighbor = self.reconstructCovarianceMatrix(receivedMatrixPositions, numDiscardedComponents);
                            if ~isempty(PapproxNeighbor)
                                neighborsData.stateVectors{neighborIndex} = self.swarm.UAVs(neighborIndex).uavStateVectorEVCI;
                                neighborsData.covarianceMatrices{neighborIndex} = PapproxNeighbor;
                                neighborsData.weights{neighborIndex} = swarmWeights(neighborIndex);
                            end
                        end
                    case 'PV'
                        % Initialize cell arrays to store states and covariances
                        neighborsData.stateVectors = cell(1, length(neighborIndices) + 1);
                        neighborsData.covarianceMatrices = cell(1, length(neighborIndices) + 1);
                        neighborsData.covarianceMatricesPos = cell(1, length(neighborIndices) + 1);
                        neighborsData.covarianceMatricesVel = cell(1, length(neighborIndices) + 1);
                        neighborsData.weights = cell(1, length(neighborIndices) + 1);

                        % Add the state and covariance of the UAV itself
                        neighborsData.stateVectors{self.uavIndex} = self.uavStateVectorEVCI;
                        neighborsData.covarianceMatrices{self.uavIndex} = self.uavCovarianceMatrixEVCI;
                        neighborsData.covarianceMatricesPos{self.uavIndex} = ...
                            self.uavCovarianceMatrixEVCI(1:self.uavVelocityOffset,1:self.uavVelocityOffset);
                        neighborsData.covarianceMatricesVel{self.uavIndex} = ...
                            self.uavCovarianceMatrixEVCI(self.uavVelocityOffset+1:end,self.uavVelocityOffset+1:end);
                        neighborsData.weights{self.uavIndex} = swarmWeights(self.uavIndex);

                        for neighborIndex = neighborIndices
                            % Get the neighbor UAV's data (simulated as received data)
                            neighborDrone = self.swarm.UAVs(neighborIndex); % Access the neighbor UAV
                            dataToBeFused = true;
                            
                            % Simulate transmission and reception
                            receivedMatrixPositions = neighborDrone.uavCovarianceMatrixPosToTransEVCI;
                            receivedMatrixVelocities = neighborDrone.uavCovarianceMatrixVelToTransEVCI;

                            % Reconstruct or directly use received data
                            numDiscardedComponentsPositions = self.uavVelocityOffset-length(neighborDrone.uavEigenvaluesPosToTransEVCI);
                            numDiscardedComponentsVelocities = self.uavVelocityOffset-length(neighborDrone.uavEigenvaluesVelToTransEVCI);

                            PapproxNeighborPositions = self.reconstructCovarianceMatrix(receivedMatrixPositions, numDiscardedComponentsPositions);
                            PapproxNeighborVelocities = self.reconstructCovarianceMatrix(receivedMatrixVelocities, numDiscardedComponentsVelocities);

                            PapproxNeighbor = [PapproxNeighborPositions zeros(self.uavVelocityOffset); zeros(self.uavVelocityOffset) PapproxNeighborVelocities];
                            if ~isempty(PapproxNeighborPositions)
                                neighborsData.stateVectors{neighborIndex} = self.swarm.UAVs(neighborIndex).uavStateVectorEVCI;
                                neighborsData.covarianceMatrices{neighborIndex} = PapproxNeighbor;
                                neighborsData.covarianceMatricesPos{neighborIndex} = PapproxNeighborPositions;
                                neighborsData.covarianceMatricesVel{neighborIndex} = PapproxNeighborVelocities;
                                neighborsData.weights{neighborIndex} = swarmWeights(neighborIndex);
                            end
                        end
                end
                % Perform Covariance Intersection or other fusion with valid data
                if dataToBeFused == true
                    [fusedState, fusedCovariance] = self.covarianceIntersection(neighborsData,'EVCI');
                end
            end
        end

        %% Funtion which applies Covariance Intersection to fuse multiple state estimates
        %  Input: neighborsData - struct with fields 'states' and 'covariances' from neighboring UAVs
        %  Output: fusedState, fusedCovraiance
        function [fusedState, fusedCovariance] = covarianceIntersection(self, neighborsData, fusionAlgorithm)

            neighborIndices = self.swarm.swarmInnerConnections{self.uavIndex};
            dronesToBeFused = [self.uavIndex neighborIndices];

            % Initial fused state and covariance
            fusedState = zeros(size(neighborsData.stateVectors{self.uavIndex}));
            fusedCovariance = zeros(length(neighborsData.stateVectors{self.uavIndex}));

            if strcmp(fusionAlgorithm,'CI') || strcmp(self.uavEstimationModel,'P')

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

            elseif strcmp(fusionAlgorithm,'EVCI') && strcmp(self.uavEstimationModel,'PV')

                % Initialize temporary fusion variables
                    fusedCovPos = zeros(size(neighborsData.covarianceMatricesPos{self.uavIndex}));
                    fusedCovVel = zeros(size(neighborsData.covarianceMatricesVel{self.uavIndex}));
                    fusedCovInvPos = zeros(size(fusedCovPos));
                    fusedCovInvVel = zeros(size(fusedCovVel));

                    for fusedDroneIndex = dronesToBeFused
                        fusedCovInvPos = fusedCovInvPos + neighborsData.weights{fusedDroneIndex} * neighborsData.covarianceMatricesPos{fusedDroneIndex}^(-1);
                        fusedCovInvVel = fusedCovInvVel + neighborsData.weights{fusedDroneIndex} * neighborsData.covarianceMatricesVel{fusedDroneIndex}^(-1);
                    end
                    % Invert to get the fused covariance matrix
                    fusedCovPos = inv(fusedCovInvPos);
                    fusedCovVel = inv(fusedCovInvVel);

                    fusedCovariance = [fusedCovPos zeros(self.uavVelocityOffset); zeros(self.uavVelocityOffset) fusedCovVel];

                    for fusedDroneIndex = dronesToBeFused
                        fusedState =  fusedState + neighborsData.weights{fusedDroneIndex} * neighborsData.covarianceMatrices{fusedDroneIndex}^(-1) * neighborsData.stateVectors{fusedDroneIndex};
                    end
                    fusedState = fusedCovariance * fusedState;
            end
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
        function [transmittedMatrix, significantEigenvalues] = pcaCompression(self,stateVectorHalf,covarianceMatrixToDecompose, ...
                reductionThreshold,stateVectorPart)

            % Ensure that covariance matrix is semi
            covarianceMatrix = nearestSPD(covarianceMatrixToDecompose);

            disp(stateVectorPart);
            % Print the eigenvalues for debugging
            [eigenvectors,eigenvalues] = eig(covarianceMatrix,'vector');
            % disp('Eigenvalues before PCA:');
            % disp(eigenvalues);
            % Perform PCA on the covariance matrix
            [pcaCoefficients,pcaEigenvalues,pcaExplained] = pcacov(covarianceMatrix);
            
            self.determineReductionThreshold();
            % Determine the number of components to reject
            % numRejected = size(pcaExplained)/2;
            % numRejected = sum(pcaEigenvalues > self.uavPosReductionThreshold);

            if strcmp(stateVectorPart,"Positions")
                % Plot uncertainty ellipsoids
                % plotEllipsoids(self,stateVectorHalf,eigenvectors,eigenvalues)
                % self.plot_eigenstructure(pcaCoefficients,pcaEigenvalues)
                % self.plot_eigenvectors(pcaCoefficients,pcaEigenvalues)
                % numRejected = sum(pcaEigenvalues > self.uavPosReductionThreshold);
                numRejected = 0;
            else
                % numRejected = sum(pcaEigenvalues > self.uavVelReductionThreshold);
                numRejected = 0;
            end

            % Select the significant components
            significantComponents = pcaCoefficients(:,(1+numRejected):size(pcaCoefficients,2));
            significantEigenvalues = pcaEigenvalues((1+numRejected):length(pcaEigenvalues));

            % Prepare the matrix to be transmitted
            transmittedMatrix = significantComponents * sqrt(diag(significantEigenvalues));
        end

        %% Function resposnible for determining reduction threshold in the EVCI algorithm (under development)
        % Output: uavPosReductionThreshold, uavVelReductionThreshold
        function determineReductionThreshold(self)
            self.uavPosReductionThreshold =  max(sqrt(self.uwbMeasurements));

            stateVectorVelocities = self.uavStateVectorEVCI(self.uavVelocityOffset+1:end);
            self.uavVelReductionThreshold = max(stateVectorVelocities);
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

        %% Uncertainty ellipsoids plotting
        %  Input:   stateVector, eigenvectors, eigenvalues
        function plotEllipsoids(~,stateVector, eigenvectors, eigenvalues)
            % Number of UAVs
            num_uavs = length(stateVector) / 3;

            % Confidence interval (adjust as needed)
            confInterval = 2.796; % 95% confidence interval

            % Generate points on a unit sphere
            [x, y, z] = sphere(20);
            spherePoints = [x(:)'; y(:)'; z(:)'];

            % Plot each UAV's position and uncertainty ellipsoid
            for i = 1:num_uavs
                % Extract UAV position
                uavPos = stateVector((i-1)*3+1 : i*3);

                % Extract corresponding eigenvectors and eigenvalues
                eiVec = eigenvectors((i-1)*3+1 : i*3, (i-1)*3+1 : i*3);
                eiVal = eigenvalues((i-1)*3+1 : i*3);

                % Calculate ellipsoid radii
                radii = sqrt(eiVal) * confInterval;

                % Generate ellipsoid points
                ellipsoid_points = uavPos + eiVec * diag(radii) * spherePoints;

                % Reshape for plotting
                ellipsoid_x = reshape(ellipsoid_points(1,:), size(x));
                ellipsoid_y = reshape(ellipsoid_points(2,:), size(y));
                ellipsoid_z = reshape(ellipsoid_points(3,:), size(z));

                % Plot ellipsoid
                surf(ellipsoid_x, ellipsoid_y, ellipsoid_z, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
                hold on;

                % Plot UAV position
                plot3(uavPos(1), uavPos(2), uavPos(3), 'r.', 'MarkerSize', 20);
            end

            axis equal;
            grid on;
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            title('UAV Swarm with Uncertainty Ellipsoids');
        end

        function plot_eigenstructure(~,eigenvectors, eigenvalues)
            % Error checking
            if size(eigenvectors, 1) ~= size(eigenvectors, 2)
                error('Eigenvectors matrix must be square');
            end
            if length(eigenvalues) ~= size(eigenvectors, 1)
                error('Length of eigenvalues must match size of eigenvectors matrix');
            end
            if mod(length(eigenvalues), 3) ~= 0
                error('Length of eigenvalues must be a multiple of 3');
            end

            % Create new figure
            figure;

            % Number of sets of 3D vectors
            num_sets = length(eigenvalues) / 3;

            % Define colors for different sets
            colors = hsv(num_sets); % Different color for each set

            % Plot origin
            plot3(0, 0, 0, 'k.', 'MarkerSize', 20);
            hold on;

            % For each set of 3 eigenvectors
            for i = 1:num_sets
                % Extract current set of eigenvectors and eigenvalues
                idx = (i-1)*3 + 1 : i*3;
                current_evecs = eigenvectors(idx, idx);
                current_evals = eigenvalues(idx);

                % Sort eigenvalues and eigenvectors by eigenvalue magnitude
                [sorted_evals, sort_idx] = sort(abs(current_evals), 'descend');
                sorted_evecs = current_evecs(:, sort_idx);

                % Plot each eigenvector scaled by its eigenvalue
                for j = 1:3
                    % Scale eigenvector by square root of eigenvalue for better visualization
                    scaled_evec = sorted_evecs(:,j) * sqrt(abs(sorted_evals(j)));

                    % Plot vector
                    quiver3(0, 0, 0, ...
                        scaled_evec(1), scaled_evec(2), scaled_evec(3), ...
                        0, ... % no automatic scaling
                        'Color', colors(i,:), ...
                        'LineWidth', 2, ...
                        'MaxHeadSize', 0.5);

                    % Add text label with eigenvalue
                    text(scaled_evec(1), scaled_evec(2), scaled_evec(3), ...
                        sprintf('λ_{%d,%d}=%.2f', i, j, sorted_evals(j)), ...
                        'Color', colors(i,:));
                end

                % Plot coordinate plane projections (optional)
                % XY plane
                plot3([sorted_evecs(1,1), sorted_evecs(1,2)], ...
                    [sorted_evecs(2,1), sorted_evecs(2,2)], ...
                    [0, 0], '--', 'Color', colors(i,:));
                % XZ plane
                plot3([sorted_evecs(1,1), sorted_evecs(1,3)], ...
                    [0, 0], ...
                    [sorted_evecs(3,1), sorted_evecs(3,3)], '--', 'Color', colors(i,:));
                % YZ plane
                plot3([0, 0], ...
                    [sorted_evecs(2,2), sorted_evecs(2,3)], ...
                    [sorted_evecs(3,2), sorted_evecs(3,3)], '--', 'Color', colors(i,:));
            end

            % Set plot properties
            grid on;
            axis equal;
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            title('Eigenvectors Scaled by Eigenvalues');

            % Add legend
            legend_entries = cell(num_sets, 1);
            for i = 1:num_sets
                legend_entries{i} = sprintf('Set %d', i);
            end
            legend(legend_entries);

            % Set view angle for better 3D visualization
            view(45, 30);

            % Make axis lines thicker
            set(gca, 'LineWidth', 1.5);
        end

        function plot_eigenvectors(~,eigenvectors, eigenvalues)
            % PLOT_EIGENVECTORS Visualizes the eigenvectors and eigenvalues in a 3D plot.
            %
            % Parameters:
            %   eigenvectors - A matrix of eigenvectors (NxN).
            %   eigenvalues  - A vector of eigenvalues (Nx1).
            %
            % The eigenvectors are plotted from the same origin with their length scaled
            % by the corresponding eigenvalue.

            % Check dimensions
            [num_dims, num_vectors] = size(eigenvectors);
            if num_dims ~= num_vectors || length(eigenvalues) ~= num_dims
                error('Eigenvector matrix must be NxN and eigenvalues must be Nx1.');
            end

            % Create figure
            figure;
            hold on;
            grid on;

            % Plot the origin point
            origin = zeros(1, num_dims);

            % Loop through each eigenvector
            for i = 1:num_dims
                % Scale the eigenvector by the corresponding eigenvalue
                scaled_vector = eigenvectors(:, i) * sqrt(eigenvalues(i));

                % Plot the eigenvector as an arrow
                quiver3(origin(1), origin(2), origin(3), scaled_vector(1), scaled_vector(2), scaled_vector(3), ...
                    'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
            end

            % Set plot labels and title
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            title('Eigenvectors and Eigenvalues Visualization');

            % Set axes limits for better visualization
            max_eigenval = max(sqrt(eigenvalues));
            axis([-max_eigenval max_eigenval -max_eigenval max_eigenval -max_eigenval max_eigenval]);

            % Set the view to 3D for better interpretation
            view(3);

            hold off;
        end
    end
end



