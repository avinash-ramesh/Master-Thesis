close all;
clear all;
format compact;
format shorte;
load("workspaces/SLAM.mat");

%% EKF SLAM - [x, vx , y, vy]

global dead_reckoning;
global n_states;
global landmark_index_map;
landmark_index_map = containers.Map;
dead_reckoning = false;
consistency_checks = ~dead_reckoning;
filter_consistency = [];
T = 0.25;
timestep = 1;

n_states = 4;
x = 0;
vx = u(1,timestep)*cosd(u(2,timestep));
y = 0;
vy = u(1,timestep)*sind(u(2,timestep));
X_pre = [x, vx, y, vy]'; % State vector X = [x, vx , y, vy], Robot starts from (x,y) = (0,0)
P_pre = blkdiag(0,25,0,25); 

ax = gca;
set(gcf, 'Position', get(0, 'Screensize'));
hold on;

v = VideoWriter('synthetic_implementation\videos\synthetic_ekf_slam_x_vx_y_vy_without_controls.avi');
v.FrameRate = 30;
open(v);

Z_ = struct;
Z_.id = Z{timestep}.id';
Z_.zpos = Z{timestep}.zpos;
if dead_reckoning
    X = X_pre;
    P = P_pre;
    Y = nan;
    S = nan;
else
%     Z_ = rotate_measurements_to_global_frame(Z_, u(2,timestep));
    [X, P, Y, S] = update(X_pre, P_pre, T, Z_);
end

filter_consistency(timestep).X = X;
filter_consistency(timestep).P = P;
filter_consistency(timestep).Y = Y;
filter_consistency(timestep).S = S;
filter_consistency(timestep).ground_truth = nan;

X_for_plot = [];
X_for_plot = [X_for_plot, [X(1);X(3)]];
robot_pose = plot(X_for_plot(1,1),X_for_plot(2,1),'rx');
delete(findobj(gcf,'Tag','landmarks'));
landmark_pos = plot(X(5:2:end),X(6:2:end),'b*','Tag','landmarks');
drawnow;
A = getframe(gcf);
writeVideo(v, A);

hold on;
for timestep = 2 : size(Z,1)
    Z_ = struct;
    Z_.id = Z{timestep}.id';
    Z_.zpos = Z{timestep}.zpos;
    if size(Z_.id,2) == 0 % Account for no measurements in the Z variable
        T = T + 0.25;
    else
%         heading = u(2, timestep-1);
%         control_input = [u(1, timestep-1) * cosd(heading);
%                          u(1, timestep-1) * sind(heading)];
        control_input = [u(1,timestep-1);
                         u(2,timestep-1)];
        
        [X_pre, P_pre] = predict(X, P, T, control_input);
        if dead_reckoning
            X = X_pre;
            P = P_pre;
            Y = nan;
            S = nan;
        else
%             Z_ = rotate_measurements_to_global_frame(Z_, heading);
            [X, P, Y, S] = update(X_pre, P_pre, T, Z_); 
        end
        
        filter_consistency(timestep).X = X;
        filter_consistency(timestep).P = P;
        filter_consistency(timestep).Y = Y;
        filter_consistency(timestep).S = S;
        filter_consistency(timestep).ground_truth = nan;
        
        X_for_plot = [X_for_plot, [X(1);X(3)]];
        robot_pose = plot(X_for_plot(1,end),X_for_plot(2,end),'rx');
        delete(findobj(gcf,'Tag','landmarks'));
        landmark_pos = plot(X(5:2:end),X(6:2:end),'b*','Tag','landmarks');
        drawnow;
        A = getframe(gcf);
        writeVideo(v, A);
        T = 0.25;
    end
end
%plot(X_for_plot(1,:),X_for_plot(2,:),'r-x');
legend([robot_pose, landmark_pos],["Robot's position", "Landmarks"], 'Location','northwest');
drawnow;
hold off;
close(v);
if consistency_checks
    save("synthetic_implementation\workspaces\filter_consistency_synthetic_ekf_x_vx_y_vy_without_controls.mat", "filter_consistency");
end

%% Prediction
function [X_pre, P_pre] = predict(X, P, T, u)
    global n_states;
    
%     A = blkdiag([1,T; 0,0],[1,T; 0,0]); % State transition matrix A
%     B = [T,0;1,0;0,T;0,1]; % Control Matrix B
%     sigma_v = 4;
%     sigma_theta = 10;
%     V = [T*cos(u(2)), -T*u(1)*sin(u(2));
%          cos(u(2))  , -u(1)*sin(u(2))   ;
%          T*cos(u(2)), T*u(1)*cos(u(2)) ;
%          sin(u(2))  , u(1)*cos(u(2))  ];
%      
%     Q = V * blkdiag(sigma_v, sigma_theta) * V';
% 
%     for i = 1: (size(X,1) - n_states)/2
%        A = blkdiag(A,1,1);
%        Q = blkdiag(Q,0,0); 
%        B = [B; zeros(2)];
%     end        
%     X_pre = A * X + B * u; % Predict State
%     P_pre = A * P * A' + Q; % Predict State Covariance
    
    
    A = blkdiag([1,T; 0,1],[1,T; 0,1]); % State transition matrix A
    sigma_vx = 25;
    sigma_vy = 25;
    Q = blkdiag([T^3/3, T^2/2; T^2/2, T],[T^3/3, T^2/2; T^2/2, T]) * blkdiag(sigma_vx, sigma_vx, sigma_vy, sigma_vy); % Brownian Process Noise Covariance Q
    
    for i = 1: (size(X,1) - n_states)/2
       A = blkdiag(A,1,1);
       Q = blkdiag(Q,0,0); 
%        B = [B; zeros(2)]; 
    end        
    
    X_pre = A * X ;% B * u; % Predict State
    P_pre = A * P * A' + Q; % Predict State Covariance
end

%% Update equations
function [X, P, Y, S] = update(X_pre, P_pre, T, Z)
    global landmark_index_map;   
    
    % Update step for every landmark that is observed
    X = X_pre; % This is done so that X is iteratively updated after each update step
    all_new_landmarks = true;
    for m = 1:size(Z.id,2)
        if landmark_index_map.isKey(string(Z.id(m)))
            all_new_landmarks = false;
            
            x_prime = X(landmark_index_map(string(Z.id(m)))) - X(1);
            y_prime = X(landmark_index_map(string(Z.id(m)))+1) - X(3);
            
            h = [sqrt(x_prime^2 + y_prime^2); atan2(y_prime, x_prime)];
            R = blkdiag(50,deg2rad(30)^2);
            
            measured_range = sqrt(Z.zpos(1,m)^2 + Z.zpos(2,m)^2);
            measured_bearing = atan2(Z.zpos(2,m), Z.zpos(1,m));
            Z_ = [measured_range; measured_bearing];            
            
            Y = Z_ - h;
            Y(2) = normalize_angle(Y(2));
            
            H = zeros(2,size(P_pre,1));
            H(1:2,1:4) = [-x_prime/sqrt(x_prime^2 + y_prime^2), 0, -y_prime/sqrt(x_prime^2 + y_prime^2), 0;
                 y_prime/(x_prime^2 + y_prime^2), 0, -x_prime/(x_prime^2 + y_prime^2), 0];
            H(1,landmark_index_map(string(Z.id(m))):landmark_index_map(string(Z.id(m)))+1) = [x_prime/sqrt(x_prime^2 + y_prime^2), y_prime/sqrt(x_prime^2 + y_prime^2)];
            H(2,landmark_index_map(string(Z.id(m))):landmark_index_map(string(Z.id(m)))+1) = [-y_prime/(x_prime^2 + y_prime^2), x_prime/(x_prime^2 + y_prime^2)];
            % Innovation covariance
            S = H * P_pre * H' + R;
            % Kalman Gain
            K = P_pre * H' * S^(-1);
            % Update State 
            X = X + K * Y;
            % Update State Covariance
            P = (eye(size(X,1)) - K*H) * P_pre;
            P = (P + P')/2; % This is done to retain symmetric state covariance matrix
        end
    end
    
    if all_new_landmarks
        X = X_pre;
        P = P_pre;
        K = NaN;
        R = NaN;
        Y = NaN;
        S = NaN;
    end
    
    % Add new landmarks to the correct position of the vehicle to create map
    for m = 1:size(Z.id,2)
        if ~landmark_index_map.isKey(string(Z.id(m)))
            landmark_index_map(string(Z.id(m))) = size(X,1)+1;
            X = vertcat(X, Z.zpos(1:2,m) + [X(1);X(3)]);
            P = blkdiag(P,1,1);
        end
    end
end

function Z_ = rotate_measurements_to_global_frame(Z_,orientation)
    q_rotmat = [cosd(orientation), -sind(orientation), 0;
                sind(orientation),  cosd(orientation), 0;
                0                ,  0                , 1];   % Usage of cosd and sind requires usage of orientation in degrees
    % q = quaternion(angle2quat(orientation, 0, 0, 'ZYX')); % This requires
    % orientation to be in radians
    % q_rotmat = rotmat(q,'point');
    temp = rotate_point_to_global_frame([Z_.zpos(1:2,:);zeros(1,size(Z_.zpos,2))],q_rotmat);
    Z_.zpos(1:2,:) = temp(1:2,:);
end

function point = rotate_point_to_global_frame(point, rotation_matrix)
    point(1:3,:) = rotation_matrix * point(1:3,:);
end

%Normalize phi to be between -pi and pi
function [phiNorm] = normalize_angle(phi)
    while(phi>pi)
        phi = phi - 2*pi;
    end

    while(phi<-pi)
        phi = phi + 2*pi;
    end
    phiNorm = phi;
end
