% HIVE
% ====
%
% Purpose:
% --------
% This script sets up and runs a simulation for a swarm of UAVs (Unmanned Aerial Vehicles) using the Swarm and Drone classes. 
% It loads scenario data, initializes the environment, and performs various operations such as sensor updates, state estimation,
% data fusion, and visualization in a loop that advances the simulation scene.

% Initial Setup:
% --------------
clc; clear; close all;

addpath(pwd,"Scenarios/");  % Adding current directory to MATLAB's search path for file access

% Loading Scenario:
% -----------------
scenarioData = loadScenario("Scenario6UAV.mat");  % Loading scenario data from *.mat file

% Initialize Swarm
Swarm = Swarm(scenarioData);  % Creating a Swarm object with the loaded scenario data

% Create Environment
% sceneAxes = createEnvironment(Swarm.swarmSimulationScene);  % Creating environment axes for 3D visualization
setup(Swarm.swarmSimulationScene);  % Setting up the simulation scene with platforms, sensors, and initial conditions

% Simulation Loop:
% ----------------
while advance(Swarm.swarmSimulationScene)  % Loop continues as long as the simulation scene can advance
    
    % Sensor Updates:
    % ---------------
    % Updates all sensor readings based on the latest states of all platforms in the scenario
    % This includes updating the positions, velocities, and orientations of the UAVs.
    updateSensors(Swarm.swarmSimulationScene);

    % Navigation Data Update:
    % -----------------------
    % Assigns the latest values to the navigational variables of the UAVs in the swarm.
    % This step ensures each UAV has updated information about its position, velocity, and orientation.
    Swarm.updateNavData();

    % True Position Update:
    % ---------------------
    % Updates the Swarm's true position vector [Latitude, Longitude, Altitude] for each UAV and inspects their vicinity to identify neighbors.
    % This helps in maintaining accurate true state information and identifying UAVs in close proximity for data fusion.
    Swarm.updateTruePositions();

    % State Estimation using EKF:
    % ---------------------------
    % Conducts state estimation for each UAV using the Extended Kalman Filter (EKF) based on GPS and UWB sensor measurements.
    % This step refines the UAVs' state estimates by fusing noisy sensor data into more accurate state information.
    Swarm.extendedKalmanFilter();

    % Covariance Intersection (CI) Data Fusion:
    % -----------------------------------------
    % Fuses state estimates with those of neighboring UAVs using Covariance Intersection (CI) to enhance estimation accuracy.
    % CI is a method to combine multiple state estimates in a way that accounts for their uncertainties.
    Swarm.fuseWithNeighborsCI();

    % Eigenvalue-based Covariance Intersection (EVCI) Data Fusion:
    % ------------------------------------------------------------
    % Fuses state estimates with those of neighboring UAVs using Eigenvalue-based Covariance Intersection (EVCI).
    % EVCI leverages Principal Component Analysis (PCA) to reduce the dimensionality of the covariance matrices before fusion.
    Swarm.fuseWithNeighborsEVCI();

    % Data Logging:
    % -------------
    % Logs UAV data for further analysis, including positions, velocities, and states.
    % This is crucial for performance analysis and debugging after the simulation ends.
    Swarm.logUAVData();

    % 3D Visualization:
    % -----------------
    % Visualizes the scenario in 3D, updating the figure window with the current states of all UAVs in the swarm.
    % 'Parent' specifies the parent axes for the visualization.
    % show3D(Swarm.swarmSimulationScene, 'Parent', sceneAxes);  
    % drawnow limitrate;  % Efficiently updates the figure window with a limited refresh rate

    % Swarm Inner Connections:
    % ------------------------
    % Checks and updates the inner connections within the swarm to ensure all UAVs are appropriately linked.
    % This is important for maintaining communication and coordination among UAVs.
    Swarm.swarmInnerConnections;
end

% Post-Simulation Analysis:
% -------------------------
% After the simulation loop ends, perform analysis on the collected data.

Swarm.calculateMetrics();  % Calculate metrics related to the swarm's performance, such as accuracy and efficiency
Swarm.plotRMSE();  % Plot the Root Mean Squared Error (RMSE) metrics to evaluate estimation accuracy
Swarm.plotSwarmEstimations(3);  % Plot the swarm estimations for the specified UAV index (3 in this case)

% End of Script
% =============
