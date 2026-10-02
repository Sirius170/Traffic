classdef MotorcycleStrategy < DrivingStrategy
    properties
        TurnStage = 0 %一般行駛
    end

    %=================================================
    %汽車原本怎麼開，機車先全部沿用。
    %只有遇到「左轉」時，機車才需要多一層特殊處理。    
    %=================================================


    methods
        function obj = MotorcycleStrategy(actor, varargin)
            obj@DrivingStrategy(actor, varargin{:});

            % 0 = 一般行駛
            % 1 = 兩段式左轉第一階段
            % 2 = 已經到待轉區，正在等待下一個可以左轉的號誌。
            % 3 = 兩段式左轉第二階段
            
            obj.TurnStage = 0;
        end
    end

end