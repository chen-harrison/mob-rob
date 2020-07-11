classdef semantic_grid_map_S_CSM < handle
    properties
        % map dimensions
        range_x = [-15, 20];
        range_y = [-25, 10];
        % sensor parameters
        z_max = 30;                 % max range in meters
        n_beams = 133;              % number of beams
        % grid map paremeters
        grid_size = 0.135;
        alpha = 2 * 0.135;          % 2 * grid_size
        beta = 2 * pi/133;          % 2 * pi/n_beams
        nn = 16;                    % number of nearest neighbor search
        map;                        % map!
        pose;                       % pose data
        scan;                       % laser scan data
        m_i = [];                   % cell i
        % semantic
        num_classes = 6;
        % -----------------------------------------------
        % To Do: 
        % prior initialization
        % -----------------------------------------------
    end
    
    methods
        function obj = semantic_grid_map_S_CSM(pose, scan)
            % class constructor
            % construct map points, i.e., grid centroids.
            x = obj.range_x(1):obj.grid_size:obj.range_x(2);
            y = obj.range_y(1):obj.grid_size:obj.range_y(2);
            [X,Y] = meshgrid(x,y);
            t = [X(:), Y(:)];
            % a simple KDtree data structure for map coordinates.
            obj.map.occMap = KDTreeSearcher(t);
            obj.map.size = size(t,1);
            
            % -----------------------------------------------
            % To Do: 
            % map parameter initialization such as map.alpha
            % -----------------------------------------------
            init_prior = 0.001;
            obj.map.alpha = repmat(init_prior, obj.map.size, obj.num_classes + 1);
            
            % set robot pose and laser scan data
            obj.pose = pose;
            obj.pose.mdl = KDTreeSearcher([pose.x, pose.y]);
            obj.scan = scan;
        end
        
        function build_ogm(obj)
            % build occupancy grid map using the binary Bayes filter.
            % we first loop over all map cells, then for each cell, we find
            % N nearest neighbor poses to build the map. Note that this is
            % more efficient than looping over all poses and all map cells
            % for each pose which should be the case in online
            % (incremental) data processing.
            for i = 1:obj.map.size
                m = obj.map.occMap.X(i,:);
                idxs = knnsearch(obj.pose.mdl, m, 'K', obj.nn);
                if ~isempty(idxs)
                    for k = idxs
                        % pose k
                        pose_k = [obj.pose.x(k),obj.pose.y(k), obj.pose.h(k)];
                        if obj.is_in_perceptual_field(m, pose_k)
                            % laser scan at kth state; convert from
                            % cartesian to polar coordinates
                            [bearing, range] = cart2pol(obj.scan{k}(1,:), obj.scan{k}(2,:));
                            z = [range' bearing' obj.scan{k}(3,:)'];
                            
                            % -----------------------------------------------
                            % To Do: 
                            % update the semantic sensor model in cell i
                            % -----------------------------------------------
                            obj.S_CSM(z, i);
                        end
                    end
                end
                
                % -----------------------------------------------
                % To Do: 
                % update mean and variance for each cell i
                % -----------------------------------------------
                obj.map.alpha_sum = sum(obj.map.alpha, 2);
                obj.map.mean = obj.map.alpha ./ obj.map.alpha_sum;
                obj.map.alpha_max = max(obj.map.alpha, [], 2);
                obj.map.variance = obj.map.alpha_max .* ...
                    (obj.map.alpha_sum - obj.map.alpha_max) ./ ...
                    ((obj.map.alpha_sum).^2 .* (obj.map.alpha_sum + 1));
            end
        end
        
        function inside = is_in_perceptual_field(obj, m, p)
            % check if the map cell m is within the perception field of the
            % robot located at pose p.
            inside = false;
            d = m - p(1:2);
            obj.m_i.range = sqrt(sum(d.^2));
            obj.m_i.phi = wrapToPi(atan2(d(2),d(1)) - p(3));
            % check if the range is within the feasible interval
            if (0 < obj.m_i.range) && (obj.m_i.range < obj.z_max)
                % here sensor covers -pi to pi!
                if (-pi < obj.m_i.phi) && (obj.m_i.phi < pi)
                    inside = true;
                end
            end
        end
        
        function S_CSM(obj, z, i)
            % -----------------------------------------------
            % To Do: 
            % implement the semantic counting sensor model
            % -----------------------------------------------
            bearing_diff = abs(wrapToPi(z(:,2) - obj.m_i.phi));
            [bearing_min, k] = min(bearing_diff);
            
            if obj.m_i.range > min(obj.z_max, z(k,1) + obj.alpha/2) || bearing_min > obj.beta/2
                % do nothing
            elseif z(k,1) < obj.z_max && abs(obj.m_i.range - z(k,1)) < obj.alpha/2 
                obj.map.alpha(i,z(k,3)) = obj.map.alpha(i,z(k,3)) + 1;
            elseif obj.m_i.range <  z(k,1) && z(k,1) < obj.z_max
                obj.map.alpha(i,7) = obj.map.alpha(i,7) + 1;
            end
        end
    end
end