function cfg = targetIntersectionConfig()
% =========================================================
% 專題目標路口的集中式設定檔
%
% 用途：
%   1. 集中管理道路與車道的幾何參數。
%   2. 避免同一個數值散落在多個程式檔案。
%   3. 方便組員修改及比較不同路口設計。
% =========================================================

%% 四個方向的長度
%  順序沿用原始場景的 Road1 ~ Road4
cfg.roadLengths = [100,100,100,100];

%% 基本道路車道設定
cfg.lanesPerDirection = 4;
cfg.totalLanesPerRoad = 2 * cfg.lanesPerDirection;
cfg.laneSpecification = [4 4];

% 暫定每條車道寬度(由中央分隔線往外)
cfg.defaultLaneWidth = 3.25;

% 目前先記錄每個方向的車道寬度，不直接覆蓋原本設定。
cfg.laneWidths = ...
    repmat(cfg.defaultLaneWidth, 1, cfg.totalLanesPerRoad);

% 標線寬度
cfg.laneMarkingWidths = 0.13;

%% 路口幾何設定
%  路口範圍與標線尺寸
cfg.intersection.crosswalkWidth = 4.0; % 行人穿越道深度
cfg.intersection.stoplineOffset = 2.0; % 停止線與穿越道的間距
cfg.intersection.stopLineWidth = 0.30; % 停止線寬度
cfg.intersection.laneMarkingWidth = cfg.laneMarkingWidths;

% 中央分隔島: 先假設尺寸，尚不改變道路幾何
cfg.intersection.medianWidth = 0.50;

% 待轉格：先假設尺寸，尚不改變道路幾何
cfg.intersection.motorcycleWaitingBoxWidth  = 3.0;
cfg.intersection.motorcycleWaitingBoxLength  = 5.0;


% 道路與車道基本尺寸
cfg.intersection.laneWidth = cfg.defaultLaneWidth;
cfg.intersection.lanesPerDirection = cfg.lanesPerDirection;
cfg.intersection.roadWidthPerDirection = ...
    cfg.intersection.laneWidth * cfg.intersection.lanesPerDirection;

% 雙向道路總寬度:目前尚未額外扣除中央分隔島
cfg.intersection.totalRoadWidth = ...
    2 * cfg.intersection.roadWidthPerDirection;

%% 定位座標
cfg.intersectionPointA = [-401.4941 2185.809 0];
cfg.intersectionPointB = [-381.8 2208.1 0];
cfg.directionPoint     = [-402.2 2206.4 0];

%% 轉向道路的中心點
cfg.roadCenters = [
    -401.4941 2185.809 0;
    -400.5184 2186.908 0;
    -398.6231 2189.23 0 ;
    -399.4963 2203.927 0;
    -401.6534 2206.009 0;
    -402.7523 2206.984 0];

%% =========================================================
% 道路車道用途設定
% 順序：從中央分隔線往路邊排列。
% =========================================================

% 新生南路一段
cfg.xinshengRoad.name = "新生南路一段";
cfg.xinshengRoad.laneCount = 4;
cfg.xinshengRoad.laneType = {"禁行機車", "禁行機車", "公車專用", "一般車道"};

% 忠孝東路三段
cfg.zhongxiaoRoad.name = "忠孝東路三段";
cfg.zhongxiaoRoad.laneCount = 4;
cfg.zhongxiaoRoad.laneType = {"禁行機車", "一般車道", "一般車道", "一般車道"};

% 各方向車道寬度，單位：公尺
cfg.laneWidthsPerDirection = ...
    repmat(cfg.defaultLaneWidth, 1, cfg.lanesPerDirection);
cfg.laneWidthsBothDirection = ...
    repmat(cfg.defaultLaneWidth, 1, cfg.totalLanesPerRoad);

% 確認車道用途數量與設定的車道數一致
assert(numel(cfg.xinshengRoad.laneType) == cfg.xinshengRoad.laneCount),
    '新生南路一段的車道用途數量不正確';
assert(numel(cfg.zhongxiaoRoad.laneType) == cfg.zhongxiaoRoad.laneCount),
    '忠孝東路三段的車道用途數量不正確';
end