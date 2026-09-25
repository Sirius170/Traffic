%waitingTime = zeros(numel(cars), 1);

% OpenTrafficLab starter: fixed-time four-way junction (not optimization).
projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot, fullfile(projectRoot,'Testing'));

% Editable settings:
inflow = [500 500 500 500]; % vehicles/hour, entry roads 1..4
turnWeights = [20 60 20];   % weights for the three routes at each entry

%%---使用 SA 找最佳號誌---
[bestPhase, bestScore, bestScoreHistory] = simulatedAnnealing();
phaseSeconds = bestPhase;
fprintf('\n=== Best SA Solution ===\n');
fprintf('Best phase = [%d %d %d %d]\n', bestPhase );
fprintf('Best waiting time = %.2f seconds\n', bestScore);

simulationSeconds = 100;
playbackSpeed = 5; % target simulated seconds per real second

assert(numel(inflow)==4 && all(isfinite(inflow)) && all(inflow>=0));
assert(numel(phaseSeconds)==4 && all(isfinite(phaseSeconds)) && all(phaseSeconds>0));
assert(numel(turnWeights)==3 && all(turnWeights>=0) && sum(turnWeights)>0);
s = createFourWayJunctionScenario();
s.StopTime = simulationSeconds;
s.SampleTime = 0.05;
net = createFourWayJunctionNetwork(s);
[cars, entryRoad] = createVehiclesForFourWayJunction(s,net,inflow,turnWeights);
waitingTime = zeros(numel(cars),1);
waitingByRoad = zeros(4,1); %south, west, north, east 
vehicleCountByRoad = zeros(4,1);

fprintf('Total %d cars\n', numel(cars));
fprintf('The first car velocity:%.2f m/s\n', norm(cars(1).Velocity));
%numel = Numbers of elements
traffic = trafficControl.TrafficLight(net(9:end), ...
    'Cliques',[1 1 1 2 2 2 3 3 3 4 4 4], ...
    'Cycle',[0 cumsum(phaseSeconds)]);
%disp(traffic.Cliques)
%disp(traffic.Cycle)

plot(s);
ax = gca;
fig = ancestor(ax,'figure');
xlim(ax, ax.XLim + [-5 5]);
ylim(ax, ax.YLim + [-5 5]);
view(ax,2);
initializeOTLScenario(s);
wallClock = tic;

%Let the simulation time keep running forward.
while isgraphics(fig) && advance(s)
    fprintf('Simulation Time = %.2f\n', s.SimulationTime);
    %disp(traffic.IsOpen);
    for k=1:numel(cars)
        speed = norm(cars(k).Velocity);
        if speed<0.1
            waitingTime(k) = waitingTime(k)+s.SampleTime;
            road = entryRoad(k); %Where is the ?? car coming from?
            waitingByRoad(road) = waitingByRoad(road) + s.SampleTime;
            vehicleCountByRoad(road) = vehicleCountByRoad(road) + 1;
        end
    end   
    traffic.plotOpenPaths(ax);
    title(ax,sprintf('Four-way junction | %.1f / %.1f s',s.SimulationTime,s.StopTime));
    drawnow limitrate;
    pause(max(0,s.SimulationTime/playbackSpeed-toc(wallClock)));
end
fprintf('Simulation stopped at %.2f seconds.\n',s.SimulationTime);
fprintf('The sum of waiting time  = %.2f second.\n', sum(waitingTime));
fprintf('The mean of waiting time = %.2f second.\n', mean(waitingTime));

%disp(waitingTime);

fprintf('\nWaiting times for each direction: \n');
fprintf('South = %.2f second.\n', waitingByRoad(1));
fprintf('West  = %.2f second.\n' , waitingByRoad(2));
fprintf('North = %.2f second.\n', waitingByRoad(3));
fprintf('East  = %.2f second.\n\n' , waitingByRoad(4));

averageWaitintByRoad = waitingByRoad ./ vehicleCountByRoad; 
fprintf('South = %.2f second/cars.\n', averageWaitintByRoad(1));
fprintf('West  = %.2f second/cars.\n' , averageWaitintByRoad(2));
fprintf('North = %.2f second/cars.\n', averageWaitintByRoad(3));
fprintf('East  = %.2f second/cars.\n' , averageWaitintByRoad(4));

fprintf('\nInformation from various sources/areas\n');
for road = 1:4
    fprintf('Direction: %d: %d cars, Total waiting: %d seconds, Average: %.2f seconds/cars\n', road, vehicleCountByRoad(road), waitingByRoad(road), averageWaitintByRoad(road));
end