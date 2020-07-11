classdef InEKF < handle   
    properties
        mu;                 % Pose Mean
        Sigma;              % Pose Sigma
        gfun;               % Motion model function
        mu_pred;             % Mean after prediction step
        Sigma_pred;          % Sigma after prediction step
        mu_cart;
        sigma_cart;
    end
    
    methods
        function obj = InEKF(sys, init)
            obj.gfun = sys.gfun;
            obj.mu = init.mu;
            obj.Sigma = init.Sigma;
        end
        
        function prediction(obj, u)
        %Formulate Adjoint function to be used in propagation
        AdjX = [obj.mu(1:2,1:2), [obj.mu(2,3); -obj.mu(1,3)]; 0, 0, 1];

        %Convert motion command into lie algebra element to pass in to
        %propagation
        X_cart = [obj.mu(1,3); obj.mu(2,3); atan2(obj.mu(2,1), obj.mu(1,1))];
        Xsim_cart = obj.gfun(X_cart, u);
        
        Xsim = posemat(obj, Xsim_cart);
        twist_hat = logm(obj.mu \ Xsim);
        
        propagation(obj, twist_hat, AdjX);
        end
        
        function propagation(obj, u, AdjX)
            % SE(2) propagation model; the input is u \in se(2) plus noise
            % propagate mean
            obj.mu_pred = obj.mu * expm(u);
            
            % propagate covariance
            Q = diag([0.1^2, 0.1^2, 0.1^2]);
            obj.Sigma_pred = obj.Sigma + AdjX*Q*AdjX';    
        end
        
        function correction(obj, Y, Y2, landmark_ids)
            global FIELDINFO;        
            landmark_x = FIELDINFO.MARKER_X_POS(landmark_ids(1));
            landmark_y = FIELDINFO.MARKER_Y_POS(landmark_ids(1));       
            landmark_x2 = FIELDINFO.MARKER_X_POS(landmark_ids(2));
            landmark_y2 = FIELDINFO.MARKER_Y_POS(landmark_ids(2));
            
%             obj.mu = obj.mu_pred;
            H1 = [-1, 0, landmark_y;
                  0, -1, -landmark_x];
            H2 = [-1, 0, landmark_y2;
                  0, -1, -landmark_x2];
            H = [H1; H2];
            
            N = obj.mu_pred * diag([100, 100, 0]) * obj.mu_pred';
            N = N(1:2,1:2);
            N_large = blkdiag(N, N);
            S = H*obj.Sigma_pred*H' + N_large;
            L = obj.Sigma_pred*H'*inv(S);
            
            eta1 = (obj.mu_pred*Y' - [landmark_x; landmark_y; 1]);
            eta2 = (obj.mu_pred*Y2' - [landmark_x2; landmark_y2; 1]);
            eta = [eta1(1:2,:); eta2(1:2,:)];
            twist = L*eta;
            twist_hat = [0, -twist(3), twist(1);
                         twist(3),  0, twist(2);
                         0,         0,        0];
            obj.mu = expm(twist_hat) * obj.mu_pred;
            obj.Sigma = (eye(3) - L*H)*obj.Sigma_pred*(eye(3) - L*H)' + L*N_large*L';
        end
        
        function H = posemat(obj, state)
            x = state(1);
            y = state(2);
            h = state(3);
            % construct a SE(2) matrix element
            H = [...
                cos(h) -sin(h) x;
                sin(h)  cos(h) y;
                     0       0 1];
        end
    end
end
