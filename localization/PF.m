classdef PF < handle
    properties
        gfun;               %Motion model function
        hfun;               %Measurement Model Function
        Q;                  %Sensor Noise
        M;                  %Motion Model Noise (dynamical and function of input)
        n;                  %Number of Particles
        particles;          %Pose of particle
        particle_weight;    %Particle Weight
        mu;
        Sigma;
    end
    
    methods
        function obj = PF(sys, init)
            % motion model
            obj.gfun = sys.gfun;
            % measurement model
            obj.hfun = sys.hfun;
            % motion noise covariance
            obj.M = sys.M;
            % measurement noise covariance
            obj.Q = sys.Q;
            % PF parameters
            obj.mu = init.mu;
            obj.Sigma = init.Sigma;
            obj.n = init.n;
            obj.particles = init.particles;
            obj.particle_weight = init.particle_weight;
        end
        function prediction(obj, u)
            for i = 1:obj.n
                obj.particles(:,i) = obj.gfun(obj.particles(:,i), chol(obj.M(u),'lower')*randn(3,1) + u);
            end
        end
        
        function correction(obj, z)
            global FIELDINFO;
            landmark_x = FIELDINFO.MARKER_X_POS(z(3));
            landmark_y = FIELDINFO.MARKER_Y_POS(z(3));
                
            %Calculate measurement and difference in measurements
            %Use mvnpdf to get weight of corresponding measurement
            z_diff = zeros(2, obj.n);
            weight_scale = zeros(obj.n, 1);
            for i = 1:obj.n
                z_diff(:,i) = z(1:2) - obj.hfun(landmark_x, landmark_y, obj.particles(:,i));
                z_diff(1,i) = wrapToPi(z_diff(1,i));
                weight_scale(i) = mvnpdf(z_diff(:,i), zeros(2,1), obj.Q);
            end

            %Update Weights
            obj.particle_weight = obj.particle_weight .* weight_scale ./ sum(obj.particle_weight .* weight_scale);
            
            n_eff = 1 / sum(obj.particle_weight.^2);
            % if n_eff < obj.n/5
            %    resample(obj);
            % end
            resample(obj);
            meanAndVariance(obj);
        end 
         
        function resample(obj)
            newSamples = zeros(size(obj.particles));
            newWeight = zeros(size(obj.particle_weight));
            W = cumsum(obj.particle_weight);
            r = rand/obj.n;
            count = 1;
            for j = 1:obj.n
                u = r+(j-1)/obj.n;
                while u > W(count)
                    count = count+1;
                end
                newSamples(:,j) = obj.particles(:,count);
                newWeight(j) = 1/obj.n;
            end
            obj.particles = newSamples;
            obj.particle_weight = newWeight;
        end
        
        function meanAndVariance(obj)
            obj.mu = mean(obj.particles, 2); 
            % orientation is a bit more tricky.
            sinSum = 0;
            cosSum = 0;
            for s = 1:obj.n
                cosSum = cosSum + cos(obj.particles(3,s));
                sinSum = sinSum + sin(obj.particles(3,s));
            end
            obj.mu(3) = atan2(sinSum, cosSum);     
            % Compute covariance.
            zeroMean = obj.particles - repmat(obj.mu, 1, obj.n);
            for s = 1:obj.n
                zeroMean(3,s) = wrapTo2Pi(zeroMean(3,s));
            end
            
            obj.Sigma = zeroMean * zeroMean' / obj.n;
        end
    end
end