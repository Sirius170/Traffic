function s = createFourWayJunctionScenario()
% Copyright 2020 - 2020 The MathWorks, Inc.

% 讀取集中式路口設定
cfg = targetIntersectionConfig();

% 道路長度
roadLengths = cfg.roadLengths;

IntCenter = (cfg.intersectionPointA+cfg.intersectionPointB)/2;
e1 = cfg.intersectionPointA-IntCenter;
e1 = -e1./norm(e1);

e2 = cfg.directionPoint-IntCenter;
e2 = e2./norm(e2);

e3 = [0,0,1];
translation = -IntCenter;

s = drivingScenario;

%% Lane marking
marking = [
    laneMarking('Unmarked')
    laneMarking('Unmarked')
    laneMarking('Solid', 'Width', cfg.laneMarkingWidths )
    laneMarking('Dashed', 'Width', cfg.laneMarkingWidths )
    laneMarking('Solid', 'Width', cfg.laneMarkingWidths )
    laneMarking('Unmarked')
    laneMarking('Unmarked')];

% 依集中式設定建立車道規格 
% Width 是車道寬度，不是標線寬度。

laneSpecification = lanespec(cfg.laneSpecification, 'Width', cfg.laneWidths, 'Marking', marking);

%% Turn
roadCenters = cfg.roadCenters;
roadCenters = roadCenters+translation;

roadCenters1 = dot(roadCenters,repmat(e1,[6,1]),2);
roadCenters2 = dot(roadCenters,repmat(e2,[6,1]),2);
roadCenters3 = dot(roadCenters,repmat(e3,[6,1]),2);


%% Roads 
%South Road(1)
locRoadCenters = [roadCenters1(1),roadCenters2(1),roadCenters3(1);roadCenters1(1)-roadLengths(1),roadCenters2(1),roadCenters3(1)];
road(s,locRoadCenters,'Lanes', laneSpecification);

%East Road(2)
locRoadCenters = [roadCenters1(end),roadCenters2(end),roadCenters3(end);roadCenters1(end),roadCenters2(end)+roadLengths(2),roadCenters3(end)];
road(s,locRoadCenters,'Lanes', laneSpecification);

%North Road(3)
locRoadCenters = [-roadCenters1(1),roadCenters2(1),roadCenters3(1);-roadCenters1(1)+roadLengths(1),roadCenters2(1),roadCenters3(1)];
road(s,locRoadCenters,'Lanes', laneSpecification);

%West Road(4)
locRoadCenters = [roadCenters1(end),-roadCenters2(end),roadCenters3(end);roadCenters1(end),-roadCenters2(end)-roadLengths(2),roadCenters3(end)];
road(s,locRoadCenters,'Lanes', laneSpecification);

%South to West(5)
locRoadCenters = [roadCenters1,roadCenters2,roadCenters3];
road(s,locRoadCenters,'Lanes', laneSpecification);

%South to North(6)
locRoadCenters=[roadCenters1(1),roadCenters2(1),roadCenters3(1);-roadCenters1(1),roadCenters2(1),roadCenters3(1)];
road(s,locRoadCenters,'Lanes', laneSpecification);

%South to East(7)
locRoadCenters = [roadCenters1,-roadCenters2,roadCenters3];
road(s,locRoadCenters,'Lanes', laneSpecification);


%North to West(8)
locRoadCenters = [-roadCenters1,roadCenters2,roadCenters3];
road(s,locRoadCenters,'Lanes', laneSpecification);


%East to West(9)
locRoadCenters=[roadCenters1(end),roadCenters2(end),roadCenters3(end);roadCenters1(end),-roadCenters2(end),roadCenters3(end)];
road(s,locRoadCenters,'Lanes', laneSpecification);


%North to East(10)
locRoadCenters = [-roadCenters1,-roadCenters2,roadCenters3];
road(s,locRoadCenters,'Lanes', laneSpecification);




