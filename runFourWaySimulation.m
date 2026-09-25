function [totalWaiting, waitingByRoad] = runFourWaySimulation(phaseSeconds)

% =========================================================
% runFourWaySimulation
%
% 輸入：
%   phaseSeconds = [South West North East] 的綠燈時間
%
% 輸出：
%   totalWaiting = 四個方向總等待時間
%   waitingByRoad = 各方向等待時間
% =========================================================

% ---------- 基本設定 ----------
inflow = [500 500 500 500];
turnWeights = [20 60 20];
simulationSeconds = 100;

% ---------- 建立場景 ----------
projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot, fullfile(projectRoot,'Testing'));

s = createFourWayJunctionScenario();
s.StopTime = simulationSeconds;
s.SampleTime = 0.05;

net = createFourWayJunctionNetwork(s);

[cars, entryRoad] = createVehiclesForFourWayJunction( ...
    s, net, inflow, turnWeights);

% ---------- 等待時間 ----------
waitingTime = zeros(numel(cars),1);
waitingByRoad = zeros(4,1);

% ---------- 建立號誌 ----------
traffic = trafficControl.TrafficLight( ...
    net(9:end), ...
    'Cliques',[1 1 1 2 2 2 3 3 3 4 4 4], ...
    'Cycle',[0 cumsum(phaseSeconds)]);

% ---------- 初始化 ----------
initializeOTLScenario(s);

% =========================================================
% 模擬
% =========================================================

while advance(s)

    for k = 1:numel(cars)

        speed = norm(cars(k).Velocity);

        if speed < 0.1

            waitingTime(k) = ...
                waitingTime(k) + s.SampleTime;

            road = entryRoad(k);

            waitingByRoad(road) = ...
                waitingByRoad(road) + s.SampleTime;

        end

    end

end

% ---------- 最終結果 ----------
totalWaiting = sum(waitingTime);

end