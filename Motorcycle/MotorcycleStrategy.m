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

        WaitingNode = Node.empty;
    end


    methods
        function obj = MotorcycleStrategy(actor, varargin)
            % 呼叫父類別 DrivingStrategy 的建構子
            obj@DrivingStrategy(actor, varargin{:});

            % 初始狀態
            obj.TurnStage = 0;  % 尚未進入兩段式左轉
            obj.IsTwoStageLeft = false;
            obj.WaitingNode = Node.empty;
        end


        % 每當模擬時間往前走一次，就呼叫這個 move()，讓這台車更新一次狀態。
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


            %% 兩段式左轉 Stage2: 待轉區等待
            if obj.IsTwoStageLeft && obj.TurnStage == 2
                % 檢查下一個 Node 是否開啟
                if getNextNodeState(obj)
                    % 第二階段號誌開放
                    obj.TurnStage = 3;

                    % 進入第二階段左轉的 Node
                    goToNextNode(obj, tNext);

                    if isempty(obj.Node)
                        running = false;
                        return
                    end

                    % 重新取得車道方向與偏移
                    [station, direction, offset] = getLaneInformation(obj);

                else
                    % 號誌尚未開放，維持在待轉區等待
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
end