function sceneAxes = createEnvironment(simulationScene)
% Create environment for simulation

[maxX, maxY, ~] = findFrameSize(simulationScene);

if maxX <= 100 
    polygonSizeX = 100; 
    axesSizeX = 110;
else
    polygonSizeX = 1000; 
    axesSizeX = 1100;
end

if maxY <= 100 
    polygonSizeY = 100;
    axesSizeY = 110;
else
    polygonSizeY = 1000;
    axesSizeY = 1100;
end

polygonSizeZ = 0;
axesSizeZ = 100;

addMesh(simulationScene,"Polygon", ...
    {[-polygonSizeY -polygonSizeX; polygonSizeY -polygonSizeX; ...
    polygonSizeY polygonSizeX; -polygonSizeY polygonSizeX],...
    [-1 polygonSizeZ]},[0.6 0.6 0.6]);

sceneAxes = show3D(simulationScene); 

xlim([-axesSizeY axesSizeY])
ylim([-axesSizeX axesSizeX])
zlim([-1 axesSizeZ])



end

function [maxX, maxY, maxZ] = findFrameSize(simulationScene)
% Initialize arrays to store maximum values for each column across all arrays
maxX = -inf;
maxY = -inf;
maxZ = -inf;

% Loop through each array
for i = 1:size(simulationScene.Platforms,2)

    % Find the maximum values in waypoints of each UAV
    maxX = max(maxX, max(simulationScene.Platforms(i).Trajectory.Waypoints(:, 1)));
    maxY = max(maxY, max(simulationScene.Platforms(i).Trajectory.Waypoints(:, 2)));
    maxZ = max(maxZ, max(simulationScene.Platforms(i).Trajectory.Waypoints(:, 3)));
end

end