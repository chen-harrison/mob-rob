classdef EKF < handle
    properties
        mu;             % Pose Mean
        Sigma;          % Pose Covariance
        gfun;           % Discrete Dynamical equations of motion
        hfun;           % Measurement model equations
        Gfun;           % Motion Model Jacobian with respect to state
        Vfun;           % Motion Model Jacobian with respect to control inputs
        Hfun;           % Measurement Model Jacobian
        M;              % Noise Covariance in input measruements(This is dynamical, changes with the input values)
        Q;              % Sensor Noise Covariance
        mu_pred;        % Predicted Mean after prediction step
        Sigma_pred;     % Predicted Covarince after prediction step
    end
    
    methods
        function obj = EKF(sys, init)
            % motion model
            obj.gfun = sys.gfun;
            % measurement model
            obj.hfun = sys.hfun;
            % Jacobian of motion model
            obj.Gfun = init.Gfun;
            obj.Vfun = init.Vfun;
            % Jocabian of measurement model
            obj.Hfun = init.Hfun;
            % motion noise covariance
            obj.M = sys.M;
            % measurement noise covariance
            obj.Q = sys.Q;
            % initial mean and covariance
            obj.mu = init.mu;
            obj.Sigma = init.Sigma;
        end
        
        function prediction(obj, u)
            %Evaluate G with mean and input
            G = obj.Gfun(obj.mu, u);
            
            %Evaluate V with mean and input
            V = obj.Vfun(obj.mu, u);
            
            %Propagate mean through non-linear dynamics
            obj.mu_pred = obj.gfun(obj.mu, u);
            
            %Update covariance with G,V and M(u)
            obj.Sigma_pred = G*obj.Sigma*G' + V*obj.M(u)*V';
        end
        
        function correction(obj, z)
            global FIELDINFO;
            landmark_x = FIELDINFO.MARKER_X_POS(z(3));
            landmark_y = FIELDINFO.MARKER_Y_POS(z(3));
            
            % Compute expected observation and Jacobian
            z_hat = obj.hfun(landmark_x, landmark_y, obj.mu_pred);            
            H = obj.Hfun(landmark_x, landmark_y, obj.mu_pred, z_hat);
            
            % Innovation / residual covariance
            S = H*obj.Sigma_pred*H' + obj.Q;
            resid = z(1:2) - z_hat;
            resid(1) = wrapToPi(resid(1));
            
            % Kalman gain
            K = obj.Sigma_pred*H'*inv(S);
            
            % Correction
            obj.mu = obj.mu_pred + K*resid;
            obj.Sigma = (eye(3) - K*H)*obj.Sigma_pred;
        end
    end
end

