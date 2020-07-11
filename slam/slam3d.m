import gtsam.*

%% read into .g2o file
[vertices, edges] = readg2o_3D("parking-garage.g2o");

%% batch solution
fg_batch = NonlinearFactorGraph;
t_init = Point3(vertices(2,1), vertices(3,1), vertices(4,1));
R_init = Rot3(quat2Rot(vertices(5:8,1)));
priorMean = Pose3(R_init, t_init);
priorNoise = noiseModel.Diagonal.Sigmas([0.3;0.3;0.3;0.1;0.1;0.1]);
fg_batch.add(PriorFactorPose3(0, priorMean, priorNoise));

for i = 1:size(edges, 2)
    t = Point3(edges(3,i), edges(4,i), edges(5,i));
    R = Rot3(quat2Rot(edges(6:9,i)));
    info = buildInfo(6, edges(10:30,i));
    noise = noiseModel.Gaussian.Covariance(inv(info));
    fg_batch.add(BetweenFactorPose3(edges(1,i), edges(2,i), Pose3(R, t), noise));
end

initial_batch = Values;
for i = 1:size(vertices, 2)
    t = Point3(vertices(2,i), vertices(3,i), vertices(4,i));
    R = Rot3(quat2Rot(vertices(5:8,i)));
    initial_batch.insert(vertices(1,i), Pose3(R, t));
end

optimizer = GaussNewtonOptimizer(fg_batch, initial_batch);
result = optimizer.optimizeSafely();
hold on
plot3DTrajectory(initial_batch, '-r');
plot3DTrajectory(result, '-b');
axis equal
title('Task 2B: Batch Solution')
hold off

%% incremental solution
isam = gtsam.ISAM2();

fg_inc = NonlinearFactorGraph;
fg_inc.add(PriorFactorPose3(0, priorMean, priorNoise));
initial_inc = Values;
initial_inc.insert(vertices(1,1), Pose3(R_init, t_init));
isam.update(fg_inc, initial_inc);
result_inc = isam.calculateEstimate();
fg_inc = NonlinearFactorGraph;
initial_inc = Values;

for i = 2:size(vertices, 2)
    key = i - 1;
    
    for j = 1:size(edges, 2)
        if edges(2,j) == key
            t = Point3(edges(3,j), edges(4,j), edges(5,j));
            R = Rot3(quat2Rot(edges(6:9,j)));
            info = buildInfo(6, edges(10:30,j));
            noise = noiseModel.Gaussian.Covariance(inv(info));
            fg_inc.add(BetweenFactorPose3(edges(1,j), edges(2,j), Pose3(R, t), noise));
            
            if edges(1,j) == key - 1
                t_odom = t;
                R_odom = R;
            end
        end
    end
    
    odom = Pose3(R_odom, t_odom);     
    initial_inc.insert(key, result_inc.at(key-1).compose(odom));
    
    isam.update(fg_inc, initial_inc);
    result_inc = isam.calculateEstimate();
    fg_inc = NonlinearFactorGraph;
    initial_inc = Values;
end

figure
hold on
plot3DTrajectory(initial_batch, '-r')
plot3DTrajectory(result_inc, '-b');
axis equal;
title('Task 2C: Incremental Solution')
hold off

%% functions
function [vertices, edges] = readg2o_3D(filename)
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
formatSpec_V = "%*s %d %f %f %f %f %f %f %f";
vertices = fscanf(fileID, formatSpec_V, [8 vertex_end]);
formatSpec_E = strcat('%*s %d %d ', repmat(' %f', 1, 28));
edges = fscanf(fileID, formatSpec_E, [30 Inf]);
end

function R = quat2Rot(q)
qx = q(1);
qy = q(2);
qz = q(3);
qw = q(4);

R = [1 - 2*(qy^2 + qz^2), 2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw);...
     2*(qx*qy + qz*qw),   1 - 2*(qx^2 + qz^2), 2*(qy*qz - qx*qw);...
     2*(qx*qz - qy*qw),   2*(qy*qz + qx*qw),   1 - 2*(qx^2 + qy^2)];
end

function info = buildInfo(n, vals)
info = zeros(n);
for i = 1:n
    info(i,i:end) = vals(1:(n - i + 1))';
    vals(1:(n - i + 1)) = [];
end
info = (info + info') - eye(n).*diag(info);
end