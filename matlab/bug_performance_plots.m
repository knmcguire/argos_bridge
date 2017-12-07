close all, clear all, clc


load results/results_12-06-2017_20-39.mat

bug_names = {'wf', 'com_bug', 'bug_2','alg_1', 'alg_2', 'i_bug', 'blind_bug'};

fitness = zeros(length(results.environment),length(bug_names))
for it = 1:length(results.environment)
    
    for itk = 1:length(bug_names)
        
    bug_temp = results.environment(it).bug;
    index = find(strcmp({bug_temp.bug_name},bug_names{itk}))
    if(~isempty(index))
    fitness(it,itk) = results.environment(it).bug(index).fitness
    end
    
    end
    
end
    
    
figure,boxplot(fitness)
