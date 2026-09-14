% create many car
function [cars, entryRoad] = createVehiclesForFourWayJunction(s,net,InjectionRate,TurnRatio)

% Copyright 2020 - 2020 The MathWorks, Inc.

rng(0); %random seed
numEntryRoads = 4; %four road
numTurns = 3; %Each entry has three possible operations.

[s,net,InjectionRate,TurnRatio] = checkInputs(s,net,InjectionRate,TurnRatio); 

%Prepare an "empty collection of vehicles."
cars = driving.scenario.Vehicle.empty;
entryRoad = zeros(0, 1);

for i=1:numEntryRoads
    %When the car entry? >> Poisson arrival process
    entryTimes = generatePoissonEntryTimes(s.SimulationTime,s.StopTime,InjectionRate(i));
    
    for entryTime =entryTimes
        j  = discretize(rand, [0 cumsum(TurnRatio)]); %where tha car go?
        
        %Where you come from → where you pass through → where you finally go.
        path = [net(i), net(i).ConnectsTo(j), net(i).ConnectsTo(j).ConnectsTo(1)];
        pos = path(1).getRoadCenterFromStation(0);
        [station,direction,offset]=path(1).getStationDistance(pos(1:2));
        car = vehicle(s,'Position',pos,'EntryTime',entryTime,'Velocity',[10,0,0]);
        car.ForwardVector = [direction,0];
        ms = DrivingStrategy(car,'NextNode',path);
        cars(end+1)=car;
        entryRoad(end+1) = i; 
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


