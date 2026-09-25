function accept = acceptSolution(currentScore, candidateScore, T, stream)

delta = candidateScore - currentScore;

%如果候選解比較好，直接接受
if delta <= 0
    accept = true;

else
    %候選解比較差，仍有機會接受
    probability = exp(-delta/T);
    accept = rand(stream) < probability;
end

end