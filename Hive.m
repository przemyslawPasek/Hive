clc; clear; close all;

scenarioData = loadScenario("ScenarioTest.mat");

%% Parameteres


% Initialize Swarm
Swarm = Swarm(scenarioData);

% Mount GPS sensor on each UAV
Swarm.mountGPS();

% Create environment
createEnvironment(Swarm.simulationScene);
ax = show3D(Swarm.simulationScene);
setup(Swarm.simulationScene)
while advance(Swarm.simulationScene)

    % Update sensor readings
    Swarm.readGPS();

    updateSensors(Swarm.simulationScene);
    Swarm.updateNavData();

    % Visualize the scenario
    show3D(Swarm.simulationScene,"Parent",ax,"FastUpdate",true);
    drawnow limitrate

    if Swarm.checkMotionEnded
        break;
    end

end
