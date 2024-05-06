classdef Drone < handle
    properties

        % UAV identification properties and scenario variables
        simulationScene % Scenario in which UAV operates
        uavPlatform % UAV platform (scenario)
        uavFlightData % UAV platform data in scenario
        uavIndex % Number of the UAV in the swarm

        % UAV State
        uavMotionVector % UAV motion vector with 16 elements 
                        % [x y z vx vy vz ax ay az q1 q2 q3 q4 wx wy wz]

        uavPosition % 3D position [X, Y, Z]
        uavVelocity % 3D velocity [Vx, Vy, Vz]
        uavOrientation % 3D orientation [4 elements quaternion vector)
        
        uavLLAVector % UAV motion vector
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

        nbSamples

        % Sensor Models
        GPS
        GPSsensor
        insGnssSensor
        uwbSensor

        % Kalman Filter
        ekf

        % Time Step
        timeStep
    end

    methods
        %%%%%%%%%% Constructor %%%%%%%%%%%%
        function self = Drone(simulationScene,uavFlightData)
            self.uavFlightData = uavFlightData;
            
            % Initialize UAV platform
            self.uavPlatform = uavPlatform(uavFlightData.Name,simulationScene,...
                ReferenceFrame=uavFlightData.ReferenceFrame, ...
                Trajectory=uavFlightData.Trajectory);

            % Initialize motion
            [self.uavMotionVector,self.uavLLAVector] = read(self.uavPlatform);
            self.uavPosition = self.uavMotionVector(1,1:3);
            self.uavVelocity = self.uavMotionVector(1,4:6);
            self.uavOrientation = self.uavMotionVector(1,10:13); % Quaternion vector for orientation

            self.uavLatitude = self.uavLLAVector(1);
            self.uavLongtitude = self.uavLLAVector(2);
            self.uavAltitude = self.uavLLAVector(3);

            self.transformOrientation();
            self.calculateGroundSpeed();

            % Set up platform mesh. Add a rotation to orient the mesh to the UAV body frame.
            updateMesh(self.uavPlatform,"quadrotor",{10},[1 0 0],self.uavPosition,self.uavOrientation);


        end
        %%%%%%%%%% End Constructor %%%%%%%%%%%%

        %% Function which is responsible for defininf GPS model and mounting GPS sensor on UAV
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
        %% the UAV from quaternion (uavOrientation) to yaw, pitch, roll angles
        %  Output: uavYaw, uawPitch, uavRoll
        function transformOrientation(self)
            [self.uavYaw, self.uavPitch, self.uavRoll] = ...
                quat2angle(self.uavOrientation);
        end

        %% Function which is resposnible for calculating true groudspeed of 
        %% the UAV from velocities components in xyz dimensions
        %  Output: uavGroundspeed
        function calculateGroundSpeed(self)
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
            self.uavOrientation = self.uavMotionVector(1,10:13); % Quaternion vector for orientation

            self.uavLatitude = self.uavLLAVector(1);
            self.uavLongtitude = self.uavLLAVector(2);
            self.uavAltitude = self.uavLLAVector(3);

            self.transformOrientation();
            self.calculateGroundSpeed();
        end
    end
end
