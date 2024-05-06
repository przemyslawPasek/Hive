function scenarioData = loadScenario(scenarioName)
% Load scenario from Matlab Scenario Designer

load(scenarioName,'Scenario');

scenarioData = Scenario;

end
