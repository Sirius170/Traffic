function [bestPhase, bestScore, bestScoreHistory] = simulatedAnnealing()

%% SA 參數設定
currentPhase = [30 10 10 10]; %初始號誌方案
T = 100;      %初始溫度
alpha = 0.95; %降溫速度
maxIter = 30; %測試跑幾次

%--- SA 專用隨機數---
saStream = RandStream('mt19937ar', 'Seed', 1);

%--計算出使方案的等待時間--
currentScore = runFourWaySimulation(currentPhase);

%--一開始的方案就是目前最佳--
bestPhase = currentPhase;
bestScore = currentScore;

%---紀錄每次迭代的最佳等待時間---
bestScoreHistory = zeros(maxIter, 1);

fprintf('\n===SA Start===\n');
fprintf('Initial phase = [%d %d %d %d]\n', currentPhase);
fprintf('Initial waiting time = %.2f seconds.\n\n', currentScore);


%% SA 主迴圈
for iter = 1:maxIter
    
    %---產生鄰近方案---
    candidatePhase = generateNeighbor(currentPhase, saStream);
    
    %---模擬候選方案---
    candidateScore = runFourWaySimulation(candidatePhase);
    
    %---判斷是否接受---
    accept = acceptSolution(currentScore, candidateScore, T, saStream);

    if accept
        currentPhase = candidatePhase;
        currentScore = candidateScore;
    end
    
    %---更新最佳解---
    if currentScore < bestScore
        bestPhase = currentPhase;
        bestScore = currentScore;
    end
    
    %---顯示這次結果---
    fprintf(['Iteration %d | ' ...
             'Phase = [%d %d %d %d] | ' ...
             'Waiting = %.2f | ' ...
             'T = %.2f\n'], ...
             iter, ...
             currentPhase(1), ...
             currentPhase(2), ...
             currentPhase(3), ...
             currentPhase(4), ...
             currentScore, ...
             T);

    %---紀錄目前最佳結果---
    bestScoreHistory(iter) = bestScore;
    
    %---降溫---
    T = T * alpha;

end

fprintf('\n===SA Result===\n');
fprintf('Best phase = [%d %d %d %d]\n', bestPhase);
fprintf('Best waiting time = %.2f seconds.\n', bestScore);

%%---SA 收斂曲線---
figure;
plot(1:maxIter, bestScoreHistory, '-o');
xlabel('Iteration');
ylabel('Best Total Waitng Time (seconds)');
title('SA Convergence');
grid on;

end

