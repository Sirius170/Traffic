function s = createFourWayJunctionScenario()
% Copyright 2020 - 2020 The MathWorks, Inc.

% 讀取集中式路口設定
cfg = targetIntersectionConfig();

% 檢查集中式設定，避免車道數量與寬度設定不一致
assert(numel(cfg.laneSpecification) == 2, ...
    'laneSpecification 必須包含兩個方向的車道數量。');
assert(all(cfg.laneSpecification == cfg.lanesPerDirection),...
    'laneSpecification 與 lanesPerDirection 設定不一致。');
assert(numel(cfg.laneWidths) == sum(cfg.laneSpecification),...
    'laneWidths 的數量必須與雙向總車道數一致。');
assert(all(cfg.laneWidths > 0),...
    '所有車道寬度都必須大於 0。');


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
% =========================================================
% 車道標線設定
% 1. 道路最外側邊界使用白色實線
% 2. 同方向可變換車道之間使用白色虛線
% 3. 保留雙向各 4 車道、每車道 3.25 m
%
% Road 1、3：南北向道路，對應新生南路。
% Road 2、4：東西向道路，對應忠孝東路。
% =========================================================

% 車道標線寬度
marhingWidth = cfg.laneMarkingWidths;

% 新生南路: 雙向各 4 車道，共 8 車道
xinshengMarking = [
    laneMarking('Unmarked')
    laneMarking('Solid', 'Width', cfg.laneMarkingWidths )
    laneMarking('Dashed', 'Width', cfg.laneMarkingWidths )
    laneMarking('Dashed', 'Width', cfg.laneMarkingWidths )
    laneMarking('Solid', 'Width', cfg.laneMarkingWidths )
    laneMarking('Dashed', 'Width', cfg.laneMarkingWidths )
    laneMarking('Dashed', 'Width', cfg.laneMarkingWidths )
    laneMarking('Solid', 'Width', cfg.laneMarkingWidths )
    laneMarking('Unmarked')];

% 忠孝東路: 雙向各 4 車道，共 8 車道
zhongxiaoMarking = [
    laneMarking('Unmarked')
    laneMarking('Solid', 'Width', cfg.laneMarkingWidths )
    laneMarking('Dashed', 'Width', cfg.laneMarkingWidths )
    laneMarking('Dashed', 'Width', cfg.laneMarkingWidths )
    laneMarking('Solid', 'Width', cfg.laneMarkingWidths )
    laneMarking('Dashed', 'Width', cfg.laneMarkingWidths )
    laneMarking('Dashed', 'Width', cfg.laneMarkingWidths )
    laneMarking('Solid', 'Width', cfg.laneMarkingWidths )
    laneMarking('Unmarked')];

% 建立道路車道規格 (Width 是車道寬度)
xinshengLaneSpecification = lanespec(cfg.laneSpecification, 'Width', cfg.laneWidths, 'Marking', xinshengMarking);
zhongxiaoLaneSpecification = lanespec(cfg.laneSpecification, 'Width', cfg.laneWidths, 'Marking', zhongxiaoMarking);

% 路口內的連接路段暫時沿用原本的標線設定。
% 這裡沿用既有規格，確保後續修改範圍可控。
laneSpecification = lanespec(cfg.laneSpecification, 'Width', cfg.laneWidths, 'Marking', ...
    [
    laneMarking('Unmarked')
    laneMarking('Dashed', 'Width', cfg.laneMarkingWidths )
    laneMarking('Dashed', 'Width', cfg.laneMarkingWidths )
    laneMarking('Dashed', 'Width', cfg.laneMarkingWidths )
    laneMarking('Dashed', 'Width', cfg.laneMarkingWidths )
    laneMarking('Dashed', 'Width', cfg.laneMarkingWidths )
    laneMarking('Dashed', 'Width', cfg.laneMarkingWidths )
    laneMarking('Solid', 'Width', cfg.laneMarkingWidths )
    laneMarking('Unmarked')]);

%% Turn
roadCenters = cfg.roadCenters;
roadCenters = roadCenters + translation;

roadCenters1 = dot(roadCenters,repmat(e1,[6,1]),2);
roadCenters2 = dot(roadCenters,repmat(e2,[6,1]),2);
roadCenters3 = dot(roadCenters,repmat(e3,[6,1]),2);

%% Intersection geometry reference
% =========================================================
% 路口幾何參考座標
%
% 先建立統一的路口中心與尺寸參數，供後續停止線、
% 行人穿越道及路口標線定位使用。
%
% 本階段不建立額外道路，也不改動既有 Road 1～10。
% =========================================================

% 現有轉換後的道路中心線，保留供後續分析。
cfg.intersection.roadCentersLocal = ...
    [roadCenters1, roadCenters2, roadCenters3];

% 記錄道路中心線在局部座標中的範圍。
cfg.intersection.localMin = min( ...
    cfg.intersection.roadCentersLocal, [], 1);

cfg.intersection.localMax = max( ...
    cfg.intersection.roadCentersLocal, [], 1);

% 暫不預設停止線距離。
% 必須先確認四個入口的中心線與道路邊界，
% 才能計算停止線與行人穿越道的位置。
cfg.intersection.geometryVerified = false;


%% Roads 
%South Road(1): 新生南路方向
locRoadCenters = [roadCenters1(1),roadCenters2(1),roadCenters3(1);roadCenters1(1)-roadLengths(1),roadCenters2(1),roadCenters3(1)];
road(s,locRoadCenters,'Lanes', xinshengLaneSpecification);

%East Road(2):  忠孝西路方向
locRoadCenters = [roadCenters1(end),roadCenters2(end),roadCenters3(end);roadCenters1(end),roadCenters2(end)+roadLengths(2),roadCenters3(end)];
road(s,locRoadCenters,'Lanes', zhongxiaoLaneSpecification);

%North Road(3): 新生南路方向
locRoadCenters = [-roadCenters1(1),roadCenters2(1),roadCenters3(1);-roadCenters1(1)+roadLengths(1),roadCenters2(1),roadCenters3(1)];
road(s,locRoadCenters,'Lanes', xinshengLaneSpecification);

%West Road(4):  忠孝西路方向
locRoadCenters = [roadCenters1(end),-roadCenters2(end),roadCenters3(end);roadCenters1(end),-roadCenters2(end)-roadLengths(2),roadCenters3(end)];
road(s,locRoadCenters,'Lanes', zhongxiaoLaneSpecification);

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




