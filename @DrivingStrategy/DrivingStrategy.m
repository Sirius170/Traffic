classdef DrivingStrategy < driving.scenario.MotionStrategy...
        & driving.scenario.mixin.PropertiesInitializableInConstructor
%DrivingStrategy Driving Strategy
%   DrivingStrategy is used to control vehicles in an OppenTrafficLab
%   simulation. The driving logic consists of car following behavior,
%   suitably modified to react to open or closed lanes ahead.
%   To implemnent a custom driving strategy, use this class as a parent
%   class and override the relevant methods
%   
%   strategy = DrivingStrategy(vehicle) creates a nominal driving strategy
%   for the actor vehicle
%   
%   strategy = DrivingStrategy(...,Name,Value) creates a nominal driving strategy
%   for the actor vehicle
% 
%   Overview
%   DrivingStrategy controls the vehicles in its move method, which is
%   called every time the advance method of the drivingScenario being
%   simulated is called. The move method moves the vehicle to the requested
%   time. It does so by setting all states and state dependent variables to
%   the immediatly previous time step, computing the control input for that
%   set of states, and integrating the states forward in time by one time
%   step. The move method perform the following steps
%   
%   1. Check if vehicle has entered the network or is currently in the network. If not break anf continue
%   2. Set the vehicle's states nadd state dependent variables to the
%   correct their value at the correct time step (i.e one time step before the target time the vehicle is being moved to)
%   3. Call determineDrivingMode
%   4. Call determineDrivingInput
%   5. Integrate states from to the requested time
%   6. Update state dependent variables based on the new value for the
%   states
%   
%   DrivingStrategy properties:
%   
%   NextNode - list of Nodes the vehicle will traverse in its path
%   UDStates - user defined states
%   Mode - flag indicating driving mode (i.e. 'CarFollowing', 'ApproachingRedLight')
%   Data - data structure for saving time series info.
%   
%       Parameters:
%   
%   DesiredSpeed - 
%   ReactionTime - 
%   SpeedBounds - 
%   AccBounds - 
%   
%       States:
%   
%   Position - 
%   Velocity - 
%   ForwardVector - 
%   
%       State dependent variable:
%   
%   Station - 
%   Speed - 
%   Node - 
%   
%   DrivingStrategy methods:
%   
%       Public Access methods:
%
%   getStationDistance - 
%   getSpeed -    
%   getPositon -  
%   getUDStates - 
%
%       User defined methods: 
%   
%   updateUDStates -  
%   initializeUDStates -  
%
%       Driving Logic methods: 
%   
%   determineDrivingMode -  
%   determineDrivingInputs -  
%   carFollowing - 
%
%   
%   Note: DrivingStrategy inherits from a MATLAB class meant for internal
%   use. It has been tested in MATLAB 2020b, and may not work in future or
%   earlier releases
%
% Copyright 2020 - 2020 The MathWorks, Inc.
%% Properties
    properties
       % R2025b actorPoses/getReferencePosition require this property.
       % This strategy updates EgoActor.Position using its rear-axle origin.
       ReferencePoint = "rear-axle";
       %Scenario drivingScenario object the 
       Scenario
       %Data structure to store the time series data of vehicle's states.
       %    Data is a structure where DrivingStrategy store the state information. 
       %    Set the StoreData property to 'true' to store data at all
       %    time steps. The default is to only store the current and previous time
       %    step, because this is needed for simulation.
       Data = struct('Time',[],...
                     'Station',[],...
                     'Speed',[],...
                     'Node',Node.empty,...
                     'UDStates',[],...
                     'Position',[],...
                     'Yaw',[]);
                 
       %NextNode a vector of type Node containing the vehicle's planned path
       NextNode = Node.empty
       
       %UDStates a vector of doubles used for user defined states.
       %    UDStates and the methods initializeUDStates and updateUDStates are
       %    placeholders for states users might want to define in child
       %    classes of DrivingStrategy, and their dynamics. 
       %    initializeUDStates is called when a vehicle enters the network
       %    and updateUDstates is called at the end of each time step, when
       %    all other states have been updated.
       UDStates = [nan] 
       
       %Mode string flag indicating driving logic
       %    Mode is a flag of type string that is set at each timestep by the method
       %    determineDrivingMode. The possible values in the nominal
       %    DrivingStrategy class are 'CarFollowing','ApproachingGreenLight',
       %    and 'ApproachingRedLight'.
       Mode = '';
       
       % Parameters for car following
       
       DesiredSpeed = 10; % [m/s]
       ReactionTime = 0.8; % [s]
       SpeedBounds = [0,15];
       AccBounds = [-5,3];
       
       % CarFollowingModel is a flag property of type string indicating
       % which car following model to use. Options: 'IDM','Gipps'
       CarFollowingModel = 'Gipps'; 
       
       % Flags
       
       %StoreData flag to indicate wether or not to store the time series data of the states
       %    Set to true to store time series data in the Data structure
       StoreData = true;
       
       %StaticLaneKeeping flag to indicate wether or not the lateral
       %dynamics are enabled
       %    Set to true to have the vehicle follow the center of the lane
       %    without worrying about control the sterring of the vehicle.
       %    The lateral control of the vehicle has not yet been tested.
       StaticLaneKeeping = true;
    end
    
    properties (Dependent,Access = protected)
        % Vehicle States
        Position(3,1) double {mustBeReal, mustBeFinite}
        Velocity(3,1) double {mustBeReal, mustBeFinite}
        ForwardVector(3,1)double {mustBeReal, mustBeFinite}
        Speed
    end
    
    properties (Access = protected)
        % Input States
        Acceleration(1,1) double {mustBeReal, mustBeFinite} = 0
        AngularAcceleration(1,1) double {mustBeReal, mustBeFinite} = 0
    end
    properties (Access = protected)
        % Position Dependent Variables
        Node = Node.empty;
        Station = [];
        % Environment Dependent Variables
        IsLeader = false;
        Leader = driving.scenario.Vehicle.empty;
        LeaderSpacing = nan;
    end
    
    %% Constructor
    methods
        function obj = DrivingStrategy(egoActor,varargin)
            obj@driving.scenario.MotionStrategy(egoActor);
            obj@driving.scenario.mixin.PropertiesInitializableInConstructor(varargin{:});
            egoActor.MotionStrategy = obj;
            obj.Scenario = egoActor.Scenario;
            obj.Speed = obj.DesiredSpeed;
            egoActor.IsVisible = false;
        end
    end
    %% Set and Get methods
    methods
        % Position
        function set.Position(obj,pos)
             obj.EgoActor.Position = pos;
        end
        function pos = get.Position(obj)
            pos = nan(length(obj),3);
            for idx = 1:length(obj)
                pos(idx,:) = obj(idx).EgoActor.Position;
            end
        end
        % Velocity
        function set.Velocity(obj,vel)
            obj.EgoActor.Velocity = vel;
        end
        function vel = get.Velocity(obj)
            vel = nan(length(obj),3);
            for idx = 1:length(obj)
                vel(idx,:) = obj(idx).EgoActor.Velocity;
            end
        end
        % Forward Vector
        function set.ForwardVector(obj,dir)
            if all(size(dir)==[3,1])
                dir=dir';
            end
            obj.EgoActor.ForwardVector = dir;
        end
        function dir = get.ForwardVector(obj)
            dir = nan(length(obj),3);
            for idx = 1:length(obj)
                dir(idx,:) = obj(idx).EgoActor.ForwardVector;
            end
        end
        % Speed
        function s = get.Speed(obj)
            s = obj.ForwardVector*obj.Velocity';
        end
        function set.Speed(obj,speed)
            obj.EgoActor.Velocity = obj.EgoActor.ForwardVector*speed;
        end
        
        % Node
        function set.Node(obj,node)
            if node == obj.Node
                return
            end
            
            if ~isempty(obj.Node)
                removeVehicle(obj.Node,obj.EgoActor);
                if ~isempty(node)
                    addVehicle(node,obj.EgoActor);
                end
                obj.Node = node;
            else
                if ~isempty(node)
                    addVehicle(node,obj.EgoActor);
                end
                obj.Node = node;
            end
                
            
        end
        
    end
    %% Public Access Methods
    methods (Access = public)
        function s = getStationDistance(obj,time) 
            if nargin<2 %If no time is given assume current sim time
                time = obj(1).Scenario.SimulationTime;
            end
            dt = obj(1).Scenario.SampleTime;
            for idx = 1:length(obj)
                tIdx = round((time-obj(idx).Data.Time(1))/dt)+1;
                if tIdx>0 && tIdx<length(obj(idx).Data.Time)+1
                    s(idx) = obj(idx).Data.Station(tIdx);
                else
                    error('The requested time is not on record')
                end
            end
        end
        function node = getNode(obj,time)
            if nargin<2 %If no time is given assusme current sim time
                time = obj(1).Scenario.SimulationTime;
            end
            dt = obj(1).Scenario.SampleTime;
            for idx = 1:length(obj)
                tIdx = round((time-obj(idx).Data.Time(1))/dt)+1;
                if tIdx>0 && tIdx<length(obj(idx).Data.Time)+1
                    node(idx) = obj(idx).Data.Node(tIdx);
                else
                    error('The requested time is not on record')
                end
            end
        end
        function v = getSpeed(obj,time)
            if nargin<2 %If no time is given assusme current sim time
                time = obj(1).Scenario.SimulationTime;
            end
            dt = obj(1).Scenario.SampleTime;
            for idx = 1:length(obj)
                tIdx = round((time-obj(idx).Data.Time(1))/dt)+1;
                if tIdx>0 && tIdx<length(obj(idx).Data.Time)+1
                    v(idx) = obj(idx).Data.Speed(tIdx);
                else
                    error('The requested time is not on record')
                end
            end
        end
        function pos = getPosition(obj,time)
            if nargin<2 %If no time is given assusme current sim time
                time = obj(1).Scenario.SimulationTime;
            end
            dt = obj(1).Scenario.SampleTime;
            for idx = 1:length(obj)
                tIdx = round((time-obj(idx).Data.Time(1))/dt)+1;
                if tIdx>0 && tIdx<length(obj(idx).Data.Time)+1
                    pos(idx,:) = obj(idx).Data.Position(tIdx,:);
                else
                    error('The requested time is not on record')
                end
            end
        end
        function states = getUDStates(obj,time)
            if nargin<2 %If no time is given assusme current sim time
                time = obj(1).Scenario.SimulationTime;
            end
            dt = obj(1).Scenario.SampleTime;
            for idx = 1:length(obj)
                tIdx = round((time-obj(idx).Data.Time(1))/dt)+1;
                if tIdx>0 && tIdx<length(obj(idx).Data.Time)+1
                    states(idx,:)  = obj(idx).Data.UDStates(tIdx,:);
                else
                    error('The requested time is not on record')
                end
            end
        end
    end
    %% Update State and Environment Dependent Variables

    methods (Access = protected)
        function updateUDStates(obj,t)
            obj.UDStates = obj.UDStates;
        end
        function initializeUDStates(obj,t)
            obj.UDStates = obj.UDStates;
        end
    end
    
    %% Helper methods
    methods (Access = protected)
              
        function [s,direction,offset]=getLaneInformation(obj)
            [s,direction,offset]=obj.Node.getStationDistance(obj.Position(1:2));
        end
        
        function length = getSegmentLength(obj)
            length = obj.Node.getRoadSegmentLength();
        end
        
        function goToNextNode(obj,SimulationTime)
            if ~isempty(obj.NextNode)
                obj.Node = obj.NextNode(1);
                obj.NextNode(1)=[];
            else
                obj.EgoActor.ExitTime = SimulationTime;
                obj.EgoActor.IsVisible = false;
                obj.Node = Node.empty;
            end
        end
        
        function injectVehicle(obj,t,speed)
            % Set the new node
            goToNextNode(obj)
            % Set the position and velocity
            obj.Position = getRoadCenterFromStation(obj.Node,0);
            if nargin>2
            obj.Speed = speed;
            end
            obj.Station = 0;
            initializeUDStates(obj,t);
            addData(obj,t)
            obj.EgoActor.IsVisible=true;   
            
        end
        
        function addData(obj,t)
            if ~isempty(obj.Data.Time)
                if t~=obj.Data.Time(end)+obj.Scenario.SampleTime
                    error('Can only add data to the following time step')
                end
            end
            
            obj.Data.Time(end+1) = t;
            obj.Data.Station(end+1) = obj.Station;
            obj.Data.Node(end+1) = obj.Node;
            obj.Data.Speed(end+1) = obj.Speed;
            obj.Data.Position(end+1,:) = obj.Position;
            obj.Data.Yaw(end+1) = obj.EgoActor.Yaw;
            obj.Data.UDStates(end+1,:) = obj.UDStates;
            
            % Delete first entry if store data flag is false
            if ~obj.StoreData && length(obj.Data.Time)>2
                obj.Data.Time(1) = [];
                obj.Data.Station(1) = [];
                obj.Data.Node(1) = [];
                obj.Data.Speed(1) = [];
                obj.Data.Yaw(1) = [];
                obj.Data.Position(1,:) = [];
                obj.Data.UDStates(1,:) = [];
                
                
            end
        end
        
        function orientEgoActor(obj,direction,offset)
            if nargin == 1
                [s,direction,offset]= obj.getLaneInformation();
            end
            obj.ForwardVector = [direction,0];
            obj.Position = obj.Position + [offset,0];
        end
        
        function [leader,leaderSpacing] = getLeader(obj,tNow)

            % 取得目前道路上的所有車輛
            actors = getVehiclesInSegment(obj);
        
            % 沒有其他車輛
            if isempty(actors)
                leader = driving.scenario.Vehicle.empty;
                leaderSpacing = nan;
                return;
            end
    
            % ---------------------------------------------------------
            % 不使用：
            %
            % drivers = [actors.MotionStrategy];
            %
            % 因為現在可能同時存在：
            % DrivingStrategy
            % MotorcycleStrategy
            %
            % 改成逐台取得 Strategy
            % ---------------------------------------------------------
        
            numActors = numel(actors);
    
            stations = zeros(1,numActors);
        
            % 先指定一個不存在的索引
            selfIdx = 0;
        
            for k = 1:numActors
        
                driver = actors(k).MotionStrategy;
        
                % 記錄每台車在道路上的位置
                stations(k) = driver.Station;
        
                % 找到自己
                if driver == obj
                    selfIdx = k;
                end
        
            end
    
            % 如果找不到自己
            if selfIdx == 0
                leader = driving.scenario.Vehicle.empty;
                leaderSpacing = nan;
                return;
            end
        
            % ---------------------------------------------------------
            % 計算其他車輛與自己的距離
            % ---------------------------------------------------------
        
            deltaStations = stations - stations(selfIdx);
        
            % 自己以及後方車輛不算 Leader
            deltaStations(deltaStations < 0.1) = inf;
        
            % 找最近的前車
            [leaderSpacing,idx] = min(deltaStations);
        
            % ---------------------------------------------------------
            % 沒有前車
            % ---------------------------------------------------------
        
            if isinf(leaderSpacing)
        
                leaderSpacing = nan;
                leader = driving.scenario.Vehicle.empty;
        
                % 如果前方還有下一個 Node
                if ~isempty(obj.NextNode)
        
                    [sLeader,leader] = ...
                        obj.NextNode(1).getTrailingVehicleStation(tNow);
        
                    if ~isempty(leader)
        
                        leaderSpacing = ...
                            sLeader ...
                            - leader.Length ...
                            + (obj.getSegmentLength() - stations(selfIdx));
        
                    end
                end
        
            % ---------------------------------------------------------
            % 有前車
            % ---------------------------------------------------------
        
            else
        
                leader = actors(idx);
        
                leaderSpacing = ...
                    leaderSpacing - leader.Length;
        
            end
    
        end
        
        function actors = getVehiclesInSegment(obj)
            actors = obj.Node.getActiveVehicles();
        end
        
        function state = getNextNodeState(obj)
            % =========================================================
            % 判斷下一個 Node 是否允許進入
            %
            % 有號誌控制器  ：依照實際號誌狀態判斷。
            % 沒有號誌控制器：視為允許通行。
            %
            % 路口內的轉向 Node 可能連接到沒有號誌的出口 Node。
            % 不可把「沒有號誌」直接視為紅燈，否則車輛會停在路口內。
            % =========================================================
            state = true;

            if ~isempty(obj.NextNode)
                nextNode = obj.NextNode(1);

                if ~isempty(obj.NextNode(1).TrafficController)
                    state = obj.NextNode(1).getNodeState();
                end
            end
        end
    end
    %% Nominal Driving Logic
    methods
        function mode = determineDrivingMode(obj,tNow)
            % This function decides which mode(s) the driver is following,
            % and the creates and outputs the function handle to that mode
            
            segLength = obj.getSegmentLength();
            leader = obj.Leader;
            leaderSpacing = obj.LeaderSpacing;
            distToEnd = segLength-obj.Station;
            state = obj.getNextNodeState();
            isLeader = isempty(leader);
            
            % =========================================================
            % 核心原則：
            % 1. 前車是否通過，不決定本車是否有通行權。
            % 2. 紅燈時，尚未進入路口的車輛必須進入紅燈減速模式。
            % 3. 已經進入受控路口的車輛，不應因號誌改變而停在路口內。
            % =========================================================
            
            % Determine the mode
            isInsideControlleredNode = ~isempty(obj.Node.TrafficController);
            
            if state
                % 下一個 Node 允許通行
                if isLeader
                    mode = 'ApproachingGreenLight';
                else
                    mode = 'CarFollowing';
                end
            
            elseif isInsideControlleredNode
                % 本車已在受控路口內，避免因號誌轉紅而停在路口中央
                if isLeader
                    mode = 'ApproachingGreenLight';
                else
                    mode = 'CarFollowing';
                end

            else
                % 尚在路口外且下一個 Node 為紅燈：
                % 不論前車是否已經通過，本車都必須自行準備停車。
                mode = 'ApproachingRedLight';
            end
        end
        
        function inputs = determineDrivingInputs(obj,tNow)
            dt = obj.Scenario.SampleTime;
            segLength = obj.getSegmentLength();
            leader = obj.Leader;
            speed = max(obj.Speed, 0);

            % 前車限制與停止線限制分開計算
            accLeader = inf;
            accStopLine = inf;

            % =========================================================
            % 限制一：前車間距
            % 前車是否通過路口，不影響本車對前車的安全距離判斷。
            % =========================================================
            if ~isempty(leader) && isfinite(obj.LeaderSpacing) 
                leaderSpeed = max(leader.MotionStrategy.Speed, 0);
                leaderGap   = max(obj.LeaderSpacing, 0);
                speedDiff   = leaderSpeed - speed;

                if strcmp(obj.CarFollowingModel, 'Gipps')
                    accLeader = drivingBehavior.gippsDriverModel( ...
                        leaderGap, speed, speedDiff);
                else
                    accLeader = carFollowing(obj, leaderGap, speed, speedDiff);
                end
            end

            % =========================================================
            % 限制二：紅燈停止線
            % 僅當本車下一個 Node 受號誌控制且目前為紅燈時啟用。
            % =========================================================
            redLight = false;

            if ~isempty(obj.NextNode)
                nextNode = obj.NextNode(1);

                if ~isempty(nextNode.TrafficController)
                    redLight = ~nextNode.getNodeState();
                end
            end

            if redLight && isempty(obj.Node.TrafficController)
                % 停留點保留車身長度及 2m 緩衝距離
                stopOffset = obj.EgoActor.Length/2+2.0;
                stopDistance = max(segLength - obj.Station - stopOffset, 0);

                if strcmp(obj.CarFollowingModel, 'Gipps')
                    accStopLine = drivingBehavior.gippsDriverModel( ...
                        max(stopDistance, 0.1), speed, -speed);
                else
                    accStopLine = carFollowing( ...
                        obj, max(stopDistance, 0.1), speed, -speed);
                end
                % =====================================================
                % 預測下一個時間步的位置
                %
                % sNext = s + v*dt + 0.5*a*dt^2
                %
                % 若預測會越過停止點，反推本時間步所需的減速度。
                % 這個檢查在位置更新前執行，不再等車輛越線後回拉。
                % =====================================================
                predictedTravel = speed*dt + 0.5*accStopLine*dt^2;

                if predictedTravel >= stopDistance
                    requiredAcc = 2*(stopDistance - speed*dt)/(dt^2);
                    accStopLine = min(accStopLine, requiredAcc);
                end
            end

            % =========================================================
            % 取兩個獨立限制中較嚴格的加速度
            % 加速度越小，代表減速越強。
            % =========================================================
            acc = min(accLeader, accStopLine);
            
            % 沒有前車限制或紅燈限制時，維持原本的行為模式
            if isinf(acc)
                switch obj.Mode
                    case 'ApproachingGreenLight'
                        acc = 0;
                    otherwise
                        acc = obj.Acceleration;
                end
            end

            % 一般加速度與速度限制
            accBounds = (obj.SpeedBounds - speed)/dt;

            accBounds = [max(accBounds(1), obj.AccBounds(1)),...
                         min(accBounds(2), obj.AccBounds(2))];
            acc = min(max(accBounds(1), acc), accBounds(2));
           
            % 緊急越線預測優先於一般舒適減速度限制。
            % 若 requiredAcc 低於一般減速度下限，這裡仍保留該限制；
            % 下一步的模擬結果需要特別檢查是否已接近物理上來不及停車。
            if redLight && isempty(obj.Node.TrafficController)
                if predictedTravel >= stopDistance
                    acc = min(acc,requiredAcc);
                end
            end

            inputs = [acc,0];
   
        end
        
        function acc = carFollowing(obj,spacing,speed,speedDiff)
            if strcmp(obj.CarFollowingModel,'IDM')
                acc = drivingBehavior.intelligentDriverModel(spacing,speed,speedDiff);
            end
            
            if strcmp(obj.CarFollowingModel,'Gipps')
                if mod(obj.Scenario.SimulationTime,obj.ReactionTime)<obj.Scenario.SampleTime
                    acc = drivingBehavior.gippsDriverModel(spacing,speed,speedDiff);
                else
                    acc = obj.Acceleration;
                end
                if ~isreal(acc)
                    acc = 0;
                end
            end 
        end
    end
    %% Inherited Abstract Methods (move, restart)
    methods
        
        function running = move(obj,SimulationTime)
            
            dt =  obj.Scenario.SampleTime;
            tNow = SimulationTime - dt;
            tNext = SimulationTime;
            car = obj.EgoActor;
            
            %% Check vehicle's entry time, and manage injection
            
            if tNow>=car.ExitTime || tNow<car.EntryTime
                car.IsVisible = false;
                running = true;%false;
                return
            end
            
            % Check if the vehicle's entry time has just been reached. If so, confirm there is
            % space in the road for it to enter. Otherwise, push entry time
            % back
            
            if (tNow - car.EntryTime>0)&&(tNow - car.EntryTime<dt)
                
                [s,leader] = getTrailingVehicleStation(obj.NextNode(1));
                if isempty(leader)
                    injectVehicle(obj,tNow,obj.Speed);
                else
                    delVel = leader.MotionStrategy.Speed-obj.Speed;
                    spacing = s-leader.Length;
                    [~,v_b,v_a]=drivingBehavior.gippsDriverModel(spacing,obj.Speed,delVel);
                    if v_b>1
                        speed = min(v_b,v_a);
                        injectVehicle(obj,tNow,speed);
                    else
                        car.EntryTime = car.EntryTime+dt;
                        running = true;
                        return;
                    end
                end
                %%
                %c = discretize(s-5,[-inf,5,10,inf]);
%                 switch c
%                     case 1 %
%                         car.EntryTime = car.EntryTime+dt;
%                         running = true;
%                         return;
%                     case 2
%                         if ~isempty(leader)
%                             speed = leader.MotionStrategy.getSpeed(tNow);
%                         else
%                             speed=obj.Speed;
%                         end
%                         injectVehicle(obj,tNow);
%                     case 3
%                         injectVehicle(obj,tNow,obj.Speed);
%                         
%                 end
            end
            
            %% Set vehicle's state and variables to tNow
            % State
            obj.Position = getPosition(obj,tNow);
            obj.Speed = getSpeed(obj,tNow);
            % State dependent variables
            obj.Station = getStationDistance(obj,tNow);
            obj.Node = getNode(obj,tNow);
            obj.UDStates = getUDStates(obj,tNow);
            
            % Environment Dependent Variables
            [obj.Leader,obj.LeaderSpacing]=getLeader(obj,tNow);
            if isempty(obj.Leader)
                obj.IsLeader = true;
            end
            %% Determine Driving Mode
            obj.Mode = determineDrivingMode(obj,tNow);
            
            %% Get Inputs
            inputs = determineDrivingInputs(obj,tNow);
            
            obj.Acceleration = inputs(1);
            obj.AngularAcceleration = inputs(2);
            
            %% Integrate

            % =========================================================
            % 先更新速度，再以新舊速度的平均值更新位置。
            % 讓位置積分與加速度計算保持一致。          
            % =========================================================            
            oldVelocity = car.Velocity;
            oldSpeed = max(obj.Speed, 0);
            
            newSpeed = max(0, oldSpeed + dt * obj.Acceleration);
            
            obj.Speed = newSpeed;
            newVelocity = car.Velocity;

            obj.Position = obj.Position + 0.5*dt*(oldVelocity+newVelocity);
                
            % Forward Vector
            %obj.ForwardVector = ... to be implemented 
            
            %% Update state dependent variables
            % Get new station distance, node and lane information
            [station, direction, offset] = getLaneInformation(obj);

            segLength = getSegmentLength(obj);

            if station > getSegmentLength(obj) % Check if the veh entered a new node
                goToNextNode(obj,tNext)
                if isempty(obj.Node)
                    % The vehicle finished its path
                    running = false;
                    return
                else
                    % Get new station andlane information
                    [station, direction, offset] = getLaneInformation(obj);
                end
                
            end
            obj.Station = station;
            updateUDStates(obj,tNow);
            
            if obj.StaticLaneKeeping %Orient veh along lane if lateral control is deactivated
                obj.orientEgoActor(direction,offset);
            end
            
            addData(obj,tNext);
            running = true;
        end
        
        function restart(obj)
            % Needs to be programmed
            
        end
    end
end

