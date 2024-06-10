classdef Drone < handle
    properties

        % UAV identification properties and scenario variables
        simulationScene % Scenario in which UAV operates
        uavPlatform % UAV platform (scenario)
        uavFlightData % UAV platform data in scenario
        uavIndex % Number of the UAV in the swarm
        uavSI % The starting index for the UAV's coordinates (SI - Starting Index)
        swarmParameters

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

        % Kalman Filter
        ekf

        % Time Step
        timeStep
    end

    methods
        %%%%%%%%%% Constructor %%%%%%%%%%%%
        function self = Drone(simulationScene,uavFlightData,swarmParameters)
            self.uavFlightData = uavFlightData;
            self.uavIndex = str2double(regexp(uavFlightData.Name, '\d+', 'match'));
            self.uavSI = (self.uavIndex - 1) * 3 + 1;
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
        function extendedKalmanFilter(self,gpsMeasurements,uwbMeasurements)

            F = eye(length(self.uavStateVector));
            Q = 0.01 * eye(length(self.uavStateVector));
           
            %%%%%%%%%% Prediction %%%%%%%%%%%%
            uavPredictedStateVector = F * self.uavStateVector;
            uavPredictedCovarianceMatrix = F * self.uavCovarianceMatrix * F' + Q;

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

            self.uavStateVector = uavUpdatedStateVector;
            self.uavCovarianceMatrix = uavUpdatedCovarianceMatrix;
        end

        %% Function which constructs the observation matrix H for EKF
        % numUAVs: Number of UAVs
        % measurementLength: Total length of the measurement vector
        % predictedState: Predicted state vector
        function H = constructObservationMatrix(~, numUAVs, measurementLength, predictedState)

            H = sparse(measurementLength, numUAVs * 3); % Initialize H as a sparse matrix

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
    end
end
