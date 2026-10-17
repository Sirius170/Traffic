%-------------------------------------------
% 汽車原本怎麼開，機車先全部沿用。
% 只有遇到「左轉」時，機車才需要多一層特殊處理。 
%-------------------------------------------

classdef MotorcycleStrategy < DrivingStrategy
    properties
        % 0 = 一般行駛
        % 1 = 兩段式左轉第一階段
        % 2 = 已經到待轉區，正在等待下一個可以左轉的號誌。
        % 3 = 兩段式左轉第二階段
        TurnStage = 0;

        % 是否為兩段式左轉機車
        IsTwoStageLeft = 0;

        % 第一段結束後，要等待的 Node
        WaitingNode = Node.empty;

        % 紀錄開始等待的時間
        % 當機車進入 Stage2 時紀錄
        WaitStartTime = NaN;

        % 在待轉區總等待時間(秒)
        TwoStageWaitTime = 0;

        % Debug 設定
        DebugEnabled = true;      % 顯示階段切換、號誌狀態、路線摘要
        DebugRoadMapping = false; % 額外顯示道路座標與 Mapping，專門查方向問題

        % 避免每個模擬時間步重複輸出
        LastDebugStage = -1;
        LastDebugSignalState = NaN;

    end


    methods
        function obj = MotorcycleStrategy(actor, varargin)
            % 呼叫父類別 DrivingStrategy 的建構子
            obj@DrivingStrategy(actor, varargin{:});

            % 初始狀態
            obj.TurnStage = 0;  
            obj.IsTwoStageLeft = false;
            obj.WaitingNode = Node.empty;
            obj.WaitStartTime = NaN;
            obj.TwoStageWaitTime = 0;
        end


        %% 每當模擬時間往前走一次，就呼叫這個 move()，讓這台車更新一次狀態。
        function running = move(obj, SimulationTime)
            dt = obj.Scenario.SampleTime;
            tNow = SimulationTime - dt;
            tNext = SimulationTime;
            car = obj.EgoActor;


            %% 還沒進入道路，或者已經離開道路
            if tNow >= car.ExitTime || tNow < car.EntryTime
                % 不顯示這輛車
                car.IsVisible = false;
                
                % 這個 Strategy 本身沒有發生需要停止模擬的錯誤，可以繼續跑。
                running = true;
                return
            end

            

            %% Check if the vehicle has entered
            if (tNow - car.EntryTime > 0 && tNow - car.EntryTime < dt)
                [s, leader] = getTrailingVehicleStation(obj.NextNode(1));

                if isempty(leader)
                    injectVehicle(obj, tNow, obj.Speed);
                else
                    delVel = leader.MotionStrategy.Speed - obj.Speed;
                    spacing = s - leader.Length;

                    [~,v_b,v_a] = drivingBehavior.gippsDriverModel(...
                        spacing, obj.Speed, delVel);

                    if v_b > 1
                        speed = min(v_b, v_a);
                        injectVehicle(obj, tNow, speed);

                    else
                        car.EntryTime = car.EntryTime + dt;
                        running = true;
                        return
                    end
                end
            end

            
            %% 取得上一個時間點的狀態
            obj.Position = getPosition(obj,tNow);
            obj.Speed = getSpeed(obj,tNow);

            % 車子目前走到那裡
            obj.Station = getStationDistance(obj,tNow);

            % getNode: 現在在哪一段道路？
            obj.Node = getNode(obj,tNow);

            % User Defined States: 使用者自己定義的狀態
            obj.UDStates = getUDStates(obj,tNow);


            %------------------------------------------------------------------
            % 兩段式左轉 Stage2: 待轉區等待
            %------------------------------------------------------------------
            
            if obj.IsTwoStageLeft && obj.TurnStage == 2
                
                % 取得第二階段號誌狀態
                signalState = getNextNodeState(obj);
                
                % 只有階段改變或號誌狀態改變時，才輸出訊息
                if obj.DebugEnabled && ...
                        (obj.LastDebugStage ~= obj.TurnStage || ...
                         obj.LastDebugSignalState ~= signalState)
                
                    fprintf('\n[二段式左轉 Debug]\n');
                    fprintf('時間：%.2f s | Stage：%d | 號誌：%d | 速度：%.2f\n', ...
                        tNext, obj.TurnStage, signalState, obj.Speed);
                
                    % 目前節點
                    if ~isempty(obj.Node)
                        fprintf('目前節點：RoadID=%d, Lane=%d\n', ...
                            obj.Node.RoadSegment.RoadID, obj.Node.Lane);
                    else
                        fprintf('目前節點：EMPTY\n');
                    end
                
                    % 待轉節點
                    if ~isempty(obj.WaitingNode)
                        fprintf('待轉節點：RoadID=%d, Lane=%d\n', ...
                            obj.WaitingNode.RoadSegment.RoadID, ...
                            obj.WaitingNode.Lane);
                
                        fprintf('是否位於待轉節點：%d\n', ...
                            ~isempty(obj.Node) && obj.Node == obj.WaitingNode);
                    end
                
                    % 預計前往的節點
                    if isempty(obj.NextNode)
                        fprintf('下一節點：EMPTY\n');
                    else
                        for k = 1:numel(obj.NextNode)
                            node = obj.NextNode(k);
                
                            if isempty(node.TrafficController)
                                stateText = '無號誌控制器';
                            else
                                stateText = sprintf('號誌狀態=%d', ...
                                    node.getNodeState());
                            end
                
                            fprintf('NextNode(%d)：RoadID=%d, Lane=%d, %s\n', ...
                                k, node.RoadSegment.RoadID, ...
                                node.Lane, stateText);
                        end
                    end
                
                    fprintf('----------------------------------------\n');
                
                    % 記錄這次輸出的狀態
                    obj.LastDebugStage = obj.TurnStage;
                    obj.LastDebugSignalState = signalState;
                end
                
                %% 檢查下一個 Node 是否開啟
                if getNextNodeState(obj)

                    % 計算機車在待轉區等了多久
                    if ~isnan(obj.WaitStartTime)
                        obj.TwoStageWaitTime = tNext - obj.WaitStartTime;
                    else
                        % 若 WaitStartTime 沒有被正確設定
                        obj.TwoStageWaitTime = 0;
                    end

                    fprintf('【二段式左轉】Stage 2 -> Stage 3，第二段號誌允許通行\n');
                    fprintf('待轉等待時間 = %.2f seconds\n', obj.TwoStageWaitTime);
                    
                    % 第二階段號誌開始
                    obj.TurnStage = 3;

                    % 保留第二階段的等待時間紀錄
                    obj.WaitStartTime = NaN;

                    % 進入第二階段左轉的 Node
                    goToNextNode(obj, tNext);

                    if isempty(obj.Node)
                        fprintf('【Stage 3 錯誤】切換後沒有下一個 Node\n');
                        running = false;
                        return
                    end

                    % 重新取得車道方向與偏移
                    [station, direction, offset] = getLaneInformation(obj);

                    %------------------------------------------------------------------
                    % Stage 3 DEBUG 
                    %------------------------------------------------------------------
                        
                    if obj.DebugEnabled
                        fprintf(['[Stage 2 -> 3] 時間=%.2f s | ' ...
                            'RoadID=%d | Lane=%d | Station=%.2f/%.2f | ' ...
                            'Speed=%.2f | NextNode數量=%d\n'], ...
                            tNext, ...
                            obj.Node.RoadSegment.RoadID, ...
                            obj.Node.Lane, ...
                            station, ...
                            getSegmentLength(obj), ...
                            obj.Speed, ...
                            numel(obj.NextNode));
                    end
                    
                    % 只有需要追查道路方向時才印出詳細座標
                    if obj.DebugRoadMapping
                        node = obj.Node;
                    
                        fprintf('[Road Mapping 詳細資訊]\n');
                        fprintf('RoadID=%d, Lane=%d\n', ...
                            node.RoadSegment.RoadID, node.Lane);
                    
                        disp('Mapping 第一列 [station x y z dx dy]：');
                        disp(node.Mapping(1,:));
                    
                        disp('Mapping 最後一列 [station x y z dx dy]：');
                        disp(node.Mapping(end,:));
                    end

                else
                    %------------------------------------------------------------------
                    % 第二階段號誌尚未開放，維持在待轉區等待
                    %
                    % 1. 不進入下一個 Node 
                    % 2. 保持 Stage 2 
                    % 3. Speed = 0 
                    % 4. 繼續停在待轉區
                    %------------------------------------------------------------------

                    obj.Speed = 0;
                    obj.Acceleration = 0;

                    station = getSegmentLength(obj);

                    obj.Position = ...
                        getRoadCenterFromStation(obj.Node, station);
                    [station, direction, offset] = getLaneInformation(obj);

                    obj.Station = station;

                    if obj.StaticLaneKeeping
                        obj.orientEgoActor(direction, offset);
                    end

                    addData(obj, tNext);
                    running = true;
                    return
                end
            end


            %% Environment dependent variables
            [obj.Leader, obj.LeaderSpacing] = getLeader(obj,tNow);

            %如果前面沒有找到車，那我就是目前這一段道路上的「領頭車」
            if isempty(obj.Leader)
                obj.IsLeader = true;
            end

            %% Get driving mode
            obj.Mode = determineDrivingMode(obj,tNow);

            %% Get driving inputs
            inputs = determineDrivingInputs(obj,tNow);

            obj.Acceleration = inputs(1);
            obj.AngularAcceleration = inputs(2);

            %% Integrate position and velocity
            obj.Position = obj.Position + dt * car.Velocity;
            obj.Speed = obj.Speed + dt*obj.Acceleration;


            % 暫時追蹤 Stage 3 的實際行駛方向
            if obj.DebugEnabled && obj.IsTwoStageLeft && obj.TurnStage == 3
                [debugStation, debugDirection, ~] = getLaneInformation(obj);

                fprintf(['[Stage 3 移動] t=%.2f | RoadID=%d | Lane=%d | ' ...
                    'Station=%.2f | Position=[%.2f, %.2f] | ' ...
                    'Direction=[%.3f, %.3f] | Speed=%.2f\n'], ...
                    tNext, ...
                    obj.Node.RoadSegment.RoadID, ...
                    obj.Node.Lane, ...
                    debugStation, ...
                    obj.Position(1), obj.Position(2), ...
                    debugDirection(1), debugDirection(2), ...
                    obj.Speed);
            end

            %% Check whether vehicle reaches the end of current Node
            [station, direction, offset] = getLaneInformation(obj);

            if station > getSegmentLength(obj)
                %----------------------------------------------------
                % 兩段式左轉 Stage1
                %
                % 如果這是一台兩段式左轉機車，而且現在正在第一段，
                % 那麼第一段走完後，不要切換 Node，
                % 而是把狀態改成等待，速度設成 0，
                % 並把車停在目前 Node 的終點。
                %----------------------------------------------------

                if obj.IsTwoStageLeft && obj.TurnStage == 1 &&...
                        ~isempty(obj.WaitingNode) && obj.Node == obj.WaitingNode

                    fprintf('【二段式左轉】到達待轉區，Stage 1 -> Stage 2\n');
                    fprintf('目前 Speed = %.2f\n', obj.Speed);                    
                                    
                    % 第一階段完成，進入待轉等待
                    obj.TurnStage = 2;

                    % 紀錄開始等待的時間
                    obj.WaitStartTime = tNext;

                    % 停車
                    obj.Speed = 0;
                    obj.Acceleration = 0;

                    % 停在目前 Node 的終點
                    station = getSegmentLength(obj);

                    % 重新取得終點位置
                    obj.Position = ...
                        getRoadCenterFromStation(obj.Node, station);

                    % 重新取得車道方向與偏移
                    [station, direction, offset] = getLaneInformation(obj);
                
                else
                    %----------------------------------------------------
                    % 一般車輛原本的行為/第二段左轉
                    %----------------------------------------------------
                    goToNextNode(obj, tNext);

                    if isempty(obj.Node)
                        % 車輛完成整條路徑
                        running = false;
                        return
                    else
                        [station, direction, offset] = ...
                            getLaneInformation(obj);
                    end
                end
            end

            %% Update state dependent variables
            obj.Station = station;
            updateUDStates(obj, tNow);

            %% Keep vehicle aligned with lane
            if obj.StaticLaneKeeping
                % 根據道路的方向與偏移量，把車子的方向和位置對齊道路。
                obj.orientEgoActor(direction, offset);
            end

            %% Store Data
            addData(obj, tNext); %把這個時間點車子的狀態記錄下來
            running = true;      %這台車的模擬還沒有結束，下一個時間步還要繼續處理它

        end
    end
    
    %% 覆寫父類別的下一節點號誌判斷
    methods(Access = protected)
        function state = getNextNodeState(obj)
            % 沿用 DrivingStrategy 原本的判斷
            state = getNextNodeState@DrivingStrategy(obj);

            % 兩段式左轉機車進入第二段後，
            % 如果下一個節點沒有號誌控制器，
            % 就不應該把它當成紅燈。 
            if obj.IsTwoStageLeft && ...
                    obj.TurnStage == 3 &&...
                    ~isempty(obj.NextNode) && ...
                    isempty(obj.NextNode(1).TrafficController)
                state = 1;
            end
        end
    end
end