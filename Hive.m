clc; clear; close all;  % Clearing command window, workspace, and closing all figures

addpath(pwd);  % Adding current directory to MATLAB's search path

scenarioData = loadScenario("Scenario4UAV.mat");  % Loading scenario data from Scenario4UAV.mat file

%% Parameters

% Initialize Swarm
Swarm = Swarm(scenarioData);  % Creating a Swarm object with scenario data

% Create environment
sceneAxes = createEnvironment(Swarm.simulationScene);  % Creating environment axes for visualization

setup(Swarm.simulationScene);  % Setting up the simulation scene

while advance(Swarm.simulationScene)  % While advancing the simulation scene

    % Updates all sensor readings based on latest states of all platforms
    % in the scenario (built-in function)
    updateSensors(Swarm.simulationScene);
           
    % Assign current values to the navigational variables of UAVs
    Swarm.updateNavData();

    % Update Swarm's true position vector [Lat Long Alt] and conduct
    % vicinity inspection (find neighbors)
    Swarm.updateTruePositions();

    % Conduct estimation in every UAV using EKF filter - GPS and UWB
    % measurements are carried out and passed as input
    Swarm.extendedKalmanFilter();

    % Fuse state estimates with neighboring UAVs using Covariance Intersection (CI)
    Swarm.fuseWithNeighborsCI();

    % Fuse state estimates with neighboring UAVs using Eigenvalue-based Covariance Intersection (EVCI)
    Swarm.fuseWithNeighborsEVCI();

    % Log UAV data for further analysis
    Swarm.logUAVData();

    % Visualize the scenario
    show3D(Swarm.simulationScene, 'Parent', sceneAxes);  % Displaying 3D visualization of the scenario
    drawnow limitrate  % Update the figure window

    Swarm.swarmInnerConnections;  % Check and update swarm inner connections
end

Swarm.calculateMetrics();  % Calculate metrics related to the swarm's performance
Swarm.plotRMSE();  % Plot Root Mean Squared Error (RMSE) metrics
Swarm.plotSwarmEstimations(3);  % Plot swarm estimations for the specified UAV index
