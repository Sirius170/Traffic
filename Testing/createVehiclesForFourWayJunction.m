% create many car
function [cars, motorcycles, carEntryRoad, motorcycleEntryRoad] = createVehiclesForFourWayJunction(s,net,InjectionRate,TurnRatio, motorcycleShare)

% Copyright 2020 - 2020 The MathWorks, Inc.

rng(0); %random seed
numEntryRoads = 4; %four road
numTurns = 3; %Each entry has three possible operations.


% 如果沒有提供 motorcycleShare
% 預設機車比例為 50%
if nargin < 5
    motorcycleShare = 0.5;
end

%檢查輸入
[s,net,InjectionRate,TurnRatio] = checkInputs(s,net,InjectionRate,TurnRatio); 

%Prepare an "empty collection of vehicles."
cars = driving.scenario.Vehicle.empty;
motorcycles = driving.scenario.Vehicle.empty;

%分別記錄汽車與機車從哪個方向進入
carEntryRoad = zeros(0, 1);
motorcycleEntryRoad = zeros(0, 1);

for i=1:numEntryRoads
    %When the car entry? >> Poisson arrival process
    entryTimes = generatePoissonEntryTimes(s.SimulationTime,s.StopTime,InjectionRate(i));
    
    for entryTime =entryTimes
        %where tha car go?
        % j = 1 → 左轉
        % j = 2 → 直行
        % j = 3 → 右轉
        j  = discretize(rand, [0 cumsum(TurnRatio)]); 
        
        %決定是汽車還是機車
        isMotorcycle = rand< motorcycleShare;


        %%-----------決定路徑------------------
        %汽車/機車:使用原本路徑
        %左轉機車: 使用兩段式左轉
        if isMotorcycle && j==1
            path = getMotorcycleLeftPath(i, net);
        else
            %Where you come from → where you pass through → where you finally go.
            path = [net(i), net(i).ConnectsTo(j), net(i).ConnectsTo(j).ConnectsTo(1)];
        end

        %計算車輛初始位置與方向
        pos = path(1).getRoadCenterFromStation(0);
        [station,direction,offset]=path(1).getStationDistance(pos(1:2));
        
        %===car===
        if ~isMotorcycle
            car = vehicle(s,'Position',pos,'EntryTime',entryTime,'Velocity',[10,0,0]);
            car.ForwardVector = [direction,0];
            DrivingStrategy(car,'NextNode',path);
            cars(end+1)=car;
            carEntryRoad(end+1) = i;
        
        %===motorcycle===
        else
            motorcycle = vehicle(s,'Position',pos,'EntryTime',entryTime,'Velocity',[10,0,0]);
            motorcycle.ForwardVector = [direction,0];
            strategy = MotorcycleStrategy(motorcycle,'NextNode',path);
           
            %如果 j=1. 表示這台機車要左轉
            if j==1
                strategy.IsTwoStageLeft = true;
                strategy.TurnStage = 1;
            end

            motorcycles(end+1)=motorcycle;
            motorcycleEntryRoad(end+1) = i; 
        end    
    end
end
end

function entryTimes = generatePoissonEntryTimes(tMin,tMax,mu)

t=tMin;
minHeadway = 1;
entryTimes = [];
while t<tMax
    headway = (-log(rand)*3600/mu);
    headway = max(minHeadway,headway);
    t = t+headway;
    if t<tMax
        entryTimes(end+1)=t;
    end
end

end

function [s,net,InjectionRate,TurnRatio] = checkInputs(s,net,InjectionRate,TurnRatio) 
    if isinf(s.StopTime)
        error('Simulation Stop Time cannot be infinite')
    end
    
    if length(InjectionRate)==1
        InjectionRate = repmat(InjectionRate,1,4);
    elseif length(InjectionRate)~=4
        error('Incorrect number of Injection Times')
    end
    
    TurnRatio = TurnRatio./sum(TurnRatio);
end


%%===建立兩段式左轉路徑===
function path = getMotorcycleLeftPath(i, net)
    switch i
        case 1
            % South -> North -> West
            path = [net(1), net(10), net(17), net(6)];
        case 2
            % West -> East -> North
            path = [net(2), net(13), net(20), net(7)];
        case 3
            % North -> South -> East
            path = [net(3), net(16), net(11), net(8)];
        case 4 
            % East -> West -> North
            path = [net(4), net(19), net(12), net(5)];
    end      
end