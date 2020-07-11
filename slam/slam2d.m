import gtsam.*

%% read into .g2o file
[vertices, edges] = readg2o_2D("input_INTEL_g2o.g2o");

%% batch solution
fg_batch = NonlinearFactorGraph;
priorMean = Pose2(vertices(2,1), vertices(3,1), vertices(4,1));
priorNoise = noiseModel.Diagonal.Sigmas([0.3;0.3;0.1]);
fg_batch.add(PriorFactorPose2(0, priorMean, priorNoise));

for i = 1:size(edges, 2)
    info = buildInfo(3, edges(6:11,i));
    noise = noiseModel.Gaussian.Covariance(inv(info));
    fg_batch.add(BetweenFactorPose2(edges(1,i), edges(2,i),...
                                       Pose2(edges(3,i), edges(4,i), edges(5,i)),...
                                       noise));
end

initial_batch = Values;
for i = 1:size(vertices, 2)
    initial_batch.insert(vertices(1,i), Pose2(vertices(2,i),...
                                                      vertices(3,i),...
                                                      vertices(4,i)));
end

gn = GaussNewtonOptimizer(fg_batch, initial_batch);
result_batch = gn.optimizeSafely();
hold on
plot2DTrajectory(initial_batch, '-r');
plot2DTrajectory(result_batch, '-b');
axis equal
title('Task 1B: Batch Solution')
hold off

%% incremental solution
isam = gtsam.ISAM2();

fg_inc = NonlinearFactorGraph;
fg_inc.add(PriorFactorPose2(0, priorMean, priorNoise));
initial_inc = Values;
initial_inc.insert(vertices(1,1), Pose2(vertices(2,1),...
                                                vertices(3,1),...
                                                vertices(4,1)));
isam.update(fg_inc, initial_inc);
result_inc = isam.calculateEstimate();
fg_inc = NonlinearFactorGraph;
initial_inc = Values;

for i = 2:size(vertices, 2)
    key = i - 1;
    
    for j = 1:size(edges, 2)
        if edges(2,j) == key
            info = buildInfo(3, edges(6:11,j));
            noise = noiseModel.Gaussian.Covariance(inv(info));
            fg_inc.add(BetweenFactorPose2(edges(1,j), edges(2,j),...
                                             Pose2(edges(3,j), edges(4,j), edges(5,j)),...
                                             noise));
            if edges(1,j) == key - 1
                odom = Pose2(edges(3,j), edges(4,j), edges(5,j));
            end
        end
    end
      
    initial_inc.insert(key, result_inc.at(key-1).compose(odom));
    
    isam.update(fg_inc, initial_inc);
    result_inc = isam.calculateEstimate();
    fg_inc = NonlinearFactorGraph;
    initial_inc = Values;
end

figure
hold on
plot2DTrajectory(initial_batch, '-r')
plot2DTrajectory(result_inc, '-b');
axis equal;
title('Task 1C: Incremental Solution')
hold off

%% functions
function [vertices, edges] = readg2o_2D(filename)
fileID = fopen(filename, 'r');

vertex_end = 0;
while(true)
    line = char(fgetl(fileID));
    if line(1) == 'E'
        break;
    else
        vertex_end = vertex_end + 1;
    end
end

frewind(fileID);
formatSpec_V = '%*s %d %f %f %f';
vertices = fscanf(fileID, formatSpec_V, [4 vertex_end]);
formatSpec_E = '%*s %d %d %f %f %f %f %f %f %f %f %f';
edges = fscanf(fileID, formatSpec_E, [11 Inf]);
end

function info = buildInfo(n, vals)
info = zeros(n);
for i = 1:n
    info(i,i:end) = vals(1:(n - i + 1))';
    vals(1:(n - i + 1)) = [];
end
info = (info + info') - eye(n).*diag(info);
end