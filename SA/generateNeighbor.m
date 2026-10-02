function newPhase = generateNeighbor(currentPhase, stream)

    % currentPhase:
    % [South West North East]
    
    minGreen = 5;
    maxGreen = 45;
    
    %%不斷嘗試，直到找到合法的新方案
    while true
        newPhase = currentPhase;
        
        %隨機選兩個不同方向
        i = randi(stream, 4);
        j = randi(stream, 4);
        
        while j == i
            j = randi(stream, 4);
        end
        
        %每次調整1~3秒
        delta = randi(stream, [1 3]);
        
        %確認 i 方向不能低於最小綠燈
        if newPhase(i)-delta >=minGreen && newPhase(i)+delta <= maxGreen
            newPhase(i) = newPhase(i) - delta;
            newPhase(j) = newPhase(j) + delta;
        
            break;
        end
    end
end

