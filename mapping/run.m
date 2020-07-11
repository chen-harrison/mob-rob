function ogm = run(task_num, plot)

addpath([cd, filesep, 'utils'])
addpath([cd, filesep, 'data'])

if ~exist('task_num','var') || isempty(task_num)
    task_num = 1;
end
if ~exist('plot','var') || isempty(plot)
    plot = true;
end

if task_num == 1 || task_num == 2
    load('sample_Intel_dataset.mat');
elseif task_num == 3 || task_num == 4
    load('sample_Intel_dataset_semantic.mat');
end


switch task_num
    % Task 1
    case 1
        ogm = occupancy_grid_map_CSM(robotPose, laserScan);
        ogm.build_ogm;
        
        if plot
            plot_mean(ogm, 'CSM Mean', 'ogm_intel_CSM_mean.png');
            plot_variance(ogm, 'CSM Variance', 'ogm_intel_CSM_variance.png');
        end
    % Task 2
    case 2
        ogm = occupancy_grid_map_continuous_CSM(robotPose, laserScan);
        ogm.build_ogm;
        
        if plot
            plot_mean(ogm, 'Continuous CSM Mean', 'ogm_intel_continuous_CSM_mean.png');
            plot_variance(ogm, 'Continuous CSM Variance', 'ogm_intel_continuous_CSM_variance.png');
        end
    % Task 3
    case 3
        ogm = semantic_grid_map_S_CSM(robotPose, laserScan);
        ogm.build_ogm;
        
        if plot
            plot_semantic(ogm, 'S-CSM Mean', 'ogm_intel_S_CSM_mean.png');
            plot_variance(ogm, 'S-CSM Variance', 'ogm_intel_S_CSM_variance.png');
        end
    % Task 4
    case 4
        ogm = semantic_grid_map_continuous_S_CSM(robotPose, laserScan);
        ogm.build_ogm;
        
        if plot
            plot_semantic(ogm, 'Continuous S-CSM Mean', 'ogm_intel_continuous_S_CSM_mean.png');
            plot_variance(ogm, 'Continuous S-CSM Variance', 'ogm_intel_continuous_S_CSM_variance.png');
        end
end
