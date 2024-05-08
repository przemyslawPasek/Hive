clc; clear; close all;

scenarioData = loadScenario("ScenarioTest.mat");

%% Parameteres


% Initialize Swarm
Swarm = Swarm(scenarioData);

% Mount GPS sensor on each UAV
Swarm.mountGPS();

% Create environment
sceneAxes = createEnvironment(Swarm.simulationScene);

setup(Swarm.simulationScene)

while advance(Swarm.simulationScene)

    % Update sensor readings
    Swarm.readGPS();

    updateSensors(Swarm.simulationScene);
    Swarm.updateNavData();

    % Visualize the scenario
    show3D(Swarm.simulationScene,Parent=sceneAxes);
    drawnow limitrate

    if Swarm.checkMotionEnded
        break;
    end

end
