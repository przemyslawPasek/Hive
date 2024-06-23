clc; clear; close all;

scenarioData = loadScenario("Scenario_4UAV.mat");

%% Parameteres

% Initialize Swarm
Swarm = Swarm(scenarioData);

% Mount GPS sensor on each UAV
Swarm.gpsMount();

% Create environment
sceneAxes = createEnvironment(Swarm.simulationScene);

setup(Swarm.simulationScene)

while advance(Swarm.simulationScene)

    % Updates all sensor readings based on latest states of all platforms
    % in the scenario (buil-in function)
    updateSensors(Swarm.simulationScene);
           
    % Assign current values to the navigational variables of UAVs
    Swarm.updateNavData();

    % Update Swarm's true postition vector [Lat Long Alt] and conduct
    % vicinity ispection (find neighbors)
    Swarm.updateTruePositions();

    % Conduct estimation in every UAV using EKF filter - GPS and UWB
    % measurements are carried out and passed as input
    Swarm.extendedKalmanFilter();

    Swarm.fuseWithNeighborsCI();

    Swarm.fuseWithNeighborsEVCI();

    Swarm.logUAVData();

    if Swarm.checkMotionEnded
        break;
    end

    % Visualize the scenario
    show3D(Swarm.simulationScene,Parent=sceneAxes);
    drawnow limitrate
    Swarm.swarmInnerConnections
end
Swarm.calculateMetrics();
Swarm.plotRMSE();
Swarm.plotATE();
Swarm.plotRPE();
Swarm.plotNIS();
