classdef UKF < handle
    properties
        mu;             % Pose Mean
        Sigma;          % Pose Covariance
        gfun;           % Motion Model Function
        hfun;           % Measurement Model Function
        M;              % Motion model noise(dynamical and function of input)
        Q;              % Sensor Noise
        kappa_g;        
        mu_pred;
        Sigma_pred;
        n;
        X;
        w;
        Y;
        Z;
    end
    
    methods
        function obj = UKF(sys, init)
            % motion model
            obj.gfun = sys.gfun;
            % measurement model
            obj.hfun = sys.hfun;
            % motion noise covariance
            obj.M = sys.M;
            % measurement noise covariance
            obj.Q = sys.Q;
            obj.kappa_g = init.kappa_g;
            % initial mean and covariance
            obj.mu = init.mu;
            obj.Sigma = init.Sigma;
        end
        
        function prediction(obj, u)
            % tune kappa_g value
            obj.kappa_g = 5;
            
            %Create Sigma Points
            mu_a = [obj.mu; zeros(3,1); zeros(2,1)];
            Sigma_a = diag([diag(obj.Sigma); diag(obj.M(u)); diag(obj.Q)]);
            sigma_point(obj, mu_a, Sigma_a, obj.kappa_g);
            
            %Propagate Sigma Points through Non-linear map
            for i = 1:size(obj.X, 2)
                obj.X(1:3,i) = obj.gfun(obj.X(1:3,i), u + obj.X(4:6,i));
            end
            obj.mu_pred = sum(obj.w' .* obj.X(1:3,:), 2);
            
            %Update Covariance
            obj.Sigma_pred = zeros(3);
            for i = 1:size(obj.X, 2)
                delta_x = obj.X(1:3,i) - obj.mu_pred;
                delta_x(3) = wrapToPi(delta_x(3));
                obj.Sigma_pred = obj.Sigma_pred + obj.w(i)*delta_x*delta_x';
            end
        end
        
        function correction(obj, z)
            global FIELDINFO;
            landmark_x = FIELDINFO.MARKER_X_POS(z(3));
            landmark_y = FIELDINFO.MARKER_Y_POS(z(3));
            
            %Calculate Innovation
            obj.Z = zeros(2, size(obj.X, 2));
            for i = 1:size(obj.Z, 2)
                obj.Z(:,i) = obj.hfun(landmark_x, landmark_y, obj.X(1:3,i)) + obj.X(7:8,i);
                obj.Z(1,i) = wrapToPi(obj.Z(1,i));
            end
            z_hat = sum(obj.w' .* obj.Z, 2);
            z_hat(1) = wrapToPi(z_hat(1));
            innovation = z(1:2) - z_hat;
            innovation(1) = wrapToPi(innovation(1));
            
            %Calculate Kalman Gain
            S = zeros(2);
            Sigma_xz = zeros(3,2);
            for i = 1:size(obj.Z, 2)
                delta_z = obj.Z(:,i) - z_hat;
                delta_z(1) = wrapToPi(delta_z(1));
                delta_x = obj.X(1:3,i) - obj.mu_pred;
                delta_x(3) = wrapToPi(delta_x(3));
                S = S + obj.w(i)*delta_z*delta_z';
                Sigma_xz = Sigma_xz + obj.w(i)*delta_x*delta_z';
            end
            K = Sigma_xz * inv(S);
            
            % Correction
            obj.mu = obj.mu_pred + K*innovation;
            obj.Sigma = obj.Sigma_pred - K*S*K';
            
        end
        
        function sigma_point(obj, mean, cov, kappa)
            obj.n = numel(mean);
            L = sqrt(obj.n + kappa) * chol(cov,'lower');
            obj.Y = mean(:,ones(1, numel(mean)));
            obj.X = [mean,obj.Y + L, obj.Y - L];
            obj.w = zeros(2 * obj.n + 1,1);
            obj.w(1) = kappa / (obj.n + kappa);
            obj.w(2:end) = 0.5 / (obj.n + kappa);
        end
    end
end