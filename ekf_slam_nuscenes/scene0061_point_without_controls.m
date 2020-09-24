%% SLAM
clc;
close all;
clear all;
scene_name = ["0655","0061","0916"];
scene_name = scene_name(2);
dead_reckoning = false;
extended_object_processing = false;
consistency_checks = true;
filter_consistency = [];
global n_states;
global landmark_index_map;
landmark_index_map = containers.Map;

load("nuscenes_implementation\workspaces\ground_truth-scene"+scene_name+".mat","ground_truth_ego_pos");
load("nuscenes_implementation\workspaces\control_input_1-scene"+scene_name+".mat","control_input");
load("nuscenes_implementation\workspaces\timestamp-scene"+scene_name+".mat","timestamp");

if ~extended_object_processing
    % For one detection per target - Point object processing
    load("nuscenes_implementation\workspaces\global_landmarks_map-scene"+scene_name+".mat","global_landmarks_map");
    load("nuscenes_implementation\workspaces\measurements-scene"+scene_name+".mat","measurements");
    load("nuscenes_implementation\workspaces\measurements_global-scene"+scene_name+".mat","measurements_global");
else
    % For multiple detections per target - Extended object processing
    load("nuscenes_implementation\workspaces\global_landmarks_map-multi_box_scene"+scene_name+".mat","global_landmarks_map");
    load("nuscenes_implementation\workspaces\measurements-multi_box-scene"+scene_name+".mat","measurements");
    load("nuscenes_implementation\workspaces\measurements_multi_box-global-scene"+scene_name+".mat","measurements_global");
end

ax = gca;
set(gcf, 'Position', get(0, 'Screensize'));
hold on;
plot_scenario(ground_truth_ego_pos, global_landmarks_map);

%% Filter Initialization
timestep = 1;
n_states = 4;

x = ground_truth_ego_pos(1,timestep);
vx = control_input(1,timestep) * cos(control_input(2,timestep));
y = ground_truth_ego_pos(2,timestep);
vy = control_input(1,timestep) * sin(control_input(2,timestep));
X_pre = [x,vx,y,vy]'; % State vector X = [x, vx , y, vy]
P_pre = blkdiag(45, 25, 45, 25); % position with 20m 3sig error, and velocity with 5m/s 1sig error.

% First Update
if dead_reckoning
    X = X_pre;
    P = P_pre;
    Y = nan;
    S = nan;
else
	[Z_, Z_plot] = get_current_measurement(timestep, measurements, measurements_global);
    % Either convert measurements from local frame to global frame or convert landmarks in state vector from global to local frame
    Z_ = rotate_measurements_to_global_frame(Z_, control_input(2,timestep));
	[X, P, K, R, Y, S] = update(X_pre, P_pre, Z_);
end

filter_consistency(timestep).X = X;
filter_consistency(timestep).P = P;
filter_consistency(timestep).Y = Y;
filter_consistency(timestep).S = S;
filter_consistency(timestep).ground_truth = ground_truth_ego_pos(:, timestep);

plot_on_global_map(timestep, X_pre, X, ground_truth_ego_pos, measurements_global, ax);
% pause(1);

prev_time = timestamp(timestep);
T = 0;



%% Predict and Update steps
for timestep = 2 : size(timestamp,2)
    T = T + (timestamp(timestep) - prev_time) * 1e-6;
    [Z_, Z_plot] = get_current_measurement(timestep, measurements, measurements_global);
    
    if size(Z_.id,2) ~= 0 % Perform filtering only when measurements in the Z variable are present
        u = [control_input(1,timestep) * cos(control_input(2,timestep));
             control_input(1,timestep) * sin(control_input(2,timestep))];
        [X_pre, P_pre, Q, B] = predict(X, P, T, u);
        if dead_reckoning
            X = X_pre;
            P = P_pre;
            Y = nan;
            S = nan;
        else
            % Either convert measurements from local frame to global frame or convert landmarks in state vector from global to local frame
            Z_ = rotate_measurements_to_global_frame(Z_, control_input(2,timestep));
            [X, P, K, R, Y, S] = update(X_pre, P_pre, Z_); 
        end
        
        filter_consistency(timestep).X = X;
        filter_consistency(timestep).P = P;
        filter_consistency(timestep).Y = Y;
        filter_consistency(timestep).S = S;
        filter_consistency(timestep).ground_truth = ground_truth_ego_pos(:, timestep);
        
        plot_on_global_map(timestep, X_pre, X, ground_truth_ego_pos, measurements_global, ax);
%         pause(1);
        T = 0;
    end
    prev_time = timestamp(timestep);
end
hold off;

if consistency_checks
    %save("nuscenes_implementation\workspaces\kf_slam_consistency1.mat","filter_consistency");
    run_consistency_checks(filter_consistency);
end



%% Helper functions
function run_consistency_checks(filter_consistency)
    displacement_error_k = [];
    states_k = [];
    states_cov_k = [];
    ground_truth = [];
    for i = 1:length(filter_consistency)
        states_k = [states_k, filter_consistency(i).X(1:4,1)];
        states_cov_k = [states_cov_k, diag(filter_consistency(i).P(1:4,1:4))];
        ground_truth = [ground_truth, filter_consistency(i).ground_truth(:,1)];
        displacement_error_k = [displacement_error_k, sqrt(sum((filter_consistency(i).X([1,3],1) - filter_consistency(i).ground_truth(:,1)).^2))];
    end

    % Check 1: Displacement (Eucledian distance) Error (minADE_k) - The pointwise L2 distances between the predicted trajectory and ground truth.  
    mean_error = mean(displacement_error_k);
    fprintf("Mean displacement error is: %f\n", mean_error);

    % Check 2: Final Displacement Error (minFDE_k) - The L2 distance between the final points of the prediction and ground truth.
    fprintf("Final displacement error is: %f\n", displacement_error_k(end));
    
    % Check 3: Miss rate at 1 meters - If pointwise L2 distance between the
    % prediction and ground truth is greater than 1 meters, we define it as a miss.
    fprintf("Miss rate at 1 meters (in percentage): %f\n", sum(displacement_error_k > 1) * 100 / length(displacement_error_k));
    
    % Check 4: Weak consistency check: Estimated error should be gaussian with variance equal to state covariance.
    estimated_error = mean(states_k([1,3],:)-ground_truth, 2);
    estimated_error_cov = var(states_k([1,3],:)-ground_truth, 0, 2);
    cov_diff = states_cov_k([1,3],end) - estimated_error_cov;
    cov_diff = blkdiag(cov_diff(1), cov_diff(2));
    ispossemidef = false;
    if issymmetric(cov_diff)
        eigen_values = eig(cov_diff);
        ispossemidef = all(eigen_values > 0);
    end
    fprintf("\nWeak consistency check: \n");
    fprintf("Zero mean estimated error: [x = %s, y = %s]\n", estimated_error(1), estimated_error(2));
    fprintf("Estimated error covariance is less than state covariance: %s\n", string(ispossemidef));
    
    
    % Check 5: Visual inspection of state covariance, the covariance should
    % decrease with time or become stationary
    figure;
    t = tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
    set(gcf, 'Position', get(0, 'Screensize'));
    hold on;
    ax = gca;
    ax.XLim = [min(states_k(1,:))-20, max(states_k(1,:))+20];
    ax.YLim = [min(states_k(3,:))-20, max(states_k(3,:))+20];
    text_to_display = {['Mean displacement error is: ', char(num2str(mean_error))],...
                       ['Final displacement error is: ', char(num2str(displacement_error_k(end)))],...
                       ['Miss rate at 1 meters: ', char(num2str(sum(displacement_error_k > 1) * 100 / length(displacement_error_k))), '%'],...
                       [' '],...
                       ['Weak consistency check: '],...
                       ['Mean estimated error: [x = ', char(num2str(estimated_error(1))),', y = ', char(num2str(estimated_error(2))),']'],...
                       ['Estimation error covariance is less than state covariance: ', char(string(ispossemidef))]};
    annotation('textbox',[.6 .25 .1 .1],'String',text_to_display,'FitBoxToText','on', 'HorizontalAlignment','left','BackgroundColor', [150/255,255/255,150/255], 'Tag','text_to_display');

    ax = nexttile;
    hold on;
    ax.XLim = [1, size(ground_truth,2)];
    error = states_k([1,3],:)-ground_truth;
    lower_boundary = -3*sqrt(states_cov_k(1,:));
    upper_boundary = 3*sqrt(states_cov_k(1,:));
    poly_shape = polyshape([1:length(upper_boundary),length(upper_boundary):-1:1],[upper_boundary, lower_boundary(end:-1:1)]);
    cov_plot = plot(poly_shape,'FaceColor','y','FaceAlpha',0.5);
    error_plot = plot(error(1,:),'r-', 'LineWidth', 2);
    plot(zeros(1,size(error(1,:),2)),'b-');
    legend([cov_plot, error_plot],["$\pm3\sigma$ boundary of state covariance","Estimation error"],'Location','northeast', 'interpreter','latex');
    title('First Order Error in Position X', 'Interpreter', 'latex');
    xlabel('Timesteps $\longrightarrow$', 'Interpreter', 'latex');
    ylabel('Position (meters) $\longrightarrow$', 'Interpreter', 'latex');
    hold off;
    
    ax = nexttile;
    hold on;
    ax.XLim = [1, size(ground_truth,2)];
    error = states_k([1,3],:)-ground_truth;
    lower_boundary = -3*sqrt(states_cov_k(3,:));
    upper_boundary = 3*sqrt(states_cov_k(3,:));
    poly_shape = polyshape([1:length(upper_boundary),length(upper_boundary):-1:1],[upper_boundary, lower_boundary(end:-1:1)]);
    cov_plot = plot(poly_shape,'FaceColor','c','FaceAlpha',0.5);
    error_plot = plot(error(2,:),'r-','LineWidth', 2);
    plot(zeros(1,size(error(1,:),2)),'b-');
    legend([cov_plot, error_plot],["$\pm3\sigma$ boundary of state covariance","Estimation error"],'Location','northeast', 'interpreter','latex');
    title('First Order Error in Position Y', 'Interpreter', 'latex');
    xlabel('Timesteps $\longrightarrow$', 'Interpreter', 'latex');
    ylabel('Position (meters) $\longrightarrow$', 'Interpreter', 'latex');
    hold off;
end

function plot_scenario(ground_truth_ego_pos, global_landmarks_map)
    % Plot ground truth trajectory of the ego car
    plot(ground_truth_ego_pos(1,:), ground_truth_ego_pos(2,:), 'b-o', 'MarkerFaceColor', 'b');

    % Plot landmarks in global coordinates
    global_landmarks_map_keys = global_landmarks_map.keys;
    for i = 1: length(global_landmarks_map.keys)
        land = global_landmarks_map(global_landmarks_map_keys{i});
        plot(land(1), land(2), "b*");
%         text(land(1), land(2), global_landmarks_map_keys{i}, "Color", "b");
    end
end

function plot_on_global_map(timestep, X_pre, X, ground_truth_ego_pos, measurements_global, ax)
    global n_states;
    legend off;
    delete(findall(ax, "Tag","current_ground_truth_position"));
	delete(findall(ax, "Tag","mean_cluster"));
	delete(findall(ax, "Tag","unclustered"));
	delete(findall(ax, "Tag","cluster_id"));
    delete(findall(ax, "Tag","landmarks"));
    
    plot(X_pre(1), X_pre(3),'c*');
    plot(X(1), X(3),'k*');
    plot(X(n_states+1:2:end), X(n_states+2:2:end),'r+','Tag','landmarks','MarkerSize',5);
	plot(ground_truth_ego_pos(1,timestep), ground_truth_ego_pos(2,timestep), "m*", "Tag","current_ground_truth_position");
	scatter(measurements_global{timestep}.zpos(1,:), measurements_global{timestep}.zpos(2,:), 'Marker','o','MarkerFaceColor' ,'g', 'MarkerEdgeColor','g', "Tag", "mean_cluster");
	scatter(measurements_global{timestep}.detections(1,:), measurements_global{timestep}.detections(2,:), 'Marker','o', 'MarkerEdgeColor','r', "Tag", "unclustered");
% 	text(measurements_global{timestep}.zpos(1,:), measurements_global{timestep}.zpos(2,:), measurements_global{timestep}.id', "Tag", "cluster_id", "Color", "g");
end

function [Z_, Z_plot] = get_current_measurement(timestep, measurements, measurements_global)
    Z_ = struct;
    Z_.id = measurements{timestep}.id';
    Z_.zpos = measurements{timestep}.zpos;
    Z_.detections = measurements{timestep}.detections;
    
    Z_plot = struct;
    Z_plot.id = measurements_global{timestep}.id';
    Z_plot.zpos = measurements_global{timestep}.zpos;
    Z_plot.detections = measurements_global{timestep}.detections;
end

function [X_pre, P_pre, Q, B] = predict(X, P, T, u)
    global n_states;
    A = blkdiag([1,T; 0,1],[1,T; 0,1]); % State transition matrix A
    B = [0,0;1,0;0,0;0,1]; % Control Matrix B
%     sigma_v = 1; % 4
%     sigma_theta = 0.001; % 10
%     V = [T*cos(u(2)), -T*u(1)*sin(u(2));
%          cos(u(2))  , -u(1)*sin(u(2))   ;
%          T*cos(u(2)), T*u(1)*cos(u(2)) ;
%          sin(u(2))  , u(1)*cos(u(2))  ];
%      
%     Q = V * blkdiag(sigma_v, sigma_theta) * V';
%     
    sigma_vx = 0.1; %100, 0.1
    sigma_vy = 0.1; %100, 0.1
    Q = blkdiag([T^3/3, T^2/2; T^2/2, T],[T^3/3, T^2/2; T^2/2, T]) * blkdiag(sigma_vx, sigma_vx, sigma_vy, sigma_vy); % Brownian Process Noise Covariance Q
    

    for i = 1: (size(X,1) - n_states)/2
       A = blkdiag(A,1,1);
       Q = blkdiag(Q,0,0); 
       B = [B; zeros(2)];
    end        
    X_pre = A * X ;%+ B * u; % Predict State
    P_pre = A * P * A' + Q; % Predict State Covariance
end

% Update equations
function [X, P, K, R, Y, S] = update(X_pre, P_pre, Z)
    global landmark_index_map;
    
    % Update step for every landmark that is observed
    X = X_pre; % This is done so that X is iteratively updated after each update step
    all_new_landmarks = true;
    for m = 1:size(Z.id,2)
        if landmark_index_map.isKey(string(Z.id(m)))
            all_new_landmarks = false;
            
            x_prime = X(landmark_index_map(string(Z.id(m)))) - X(1);
            y_prime = X(landmark_index_map(string(Z.id(m)))+1) - X(3);
            
            h = [sqrt(x_prime^2 + y_prime^2); atan2(y_prime, x_prime)]; % For bearing, either subtract the orientation of vehicle here, or rotate measurements to global frame before calling this function 
            R = blkdiag(1,0.01); %one_0916:5,5 ; multi_0916: 0.1, 0.005; one_0655:5,0.001 ; multi_0655: 0.1, 0.005
            
            measured_range = sqrt(Z.zpos(1,m)^2 + Z.zpos(2,m)^2);
            measured_bearing = atan2(Z.zpos(2,m),Z.zpos(1,m));
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
            P = blkdiag(P,25,25);
        end
    end
end

function Z_ = rotate_measurements_to_global_frame(Z_, orientation)
    q_rotmat = [cos(orientation), -sin(orientation), 0;
                sin(orientation),  cos(orientation), 0;
                0               ,  0               , 1];
%     q = quaternion(angle2quat(orientation, 0, 0, 'ZYX'));
%     q_rotmat = rotmat(q,'point');   
    temp = rotate_point_to_global_frame([Z_.zpos(1:2,:);zeros(1,size(Z_.zpos,2))],q_rotmat);
    Z_.zpos(1:2,:) = temp(1:2,:);
end
            
function point = translate_point_to_global_frame(point, translation_matrix)
    point(1:3,:) = point(1:3,:) + translation_matrix;
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

%% Backup

% % ground_truth_ego_pos : (2x40) containing position coordinates [x;y]
% % global_landmarks_map : is a map with key=landmark label and value=[x;y]
% % which is the center of the target measurements : (40x1) contains
% % measurements as a structure for each timestep. This is to be used as
% % measurements for the filter.
% 			% measurements.id        : contains label of landmarks observed
% 			% in the current timestep. measurements.zpos      : contains
% 			% mean values for [x,y,vx,vy] of all the detections inside the
% 			% target in the ego vehicle coordinate system.
% 			% measurements.detections: contains [x,y,vx,vy] of unclustered
% 			% detections in ego vehicle coordinate system.
% 				
% % measurements_global : (40x1) contains measurements as a structure for
% % each timestep. This is to be used only for plotting.
% 					% measurement_globals.id        : contains label of
% 					% landmarks observed in the current timestep.
% 					% measurement_globals.zpos      : contains [x,y,vx,vy]
% 					% of the measured target in the global coordinate
% 					% system. measurement_globals.detections: contains
% 					% [x,y,vx,vy] of unclustered detections in global
% 					% coordinate system.
% % timestamp : contains time in long data type
% % message [velocity; bearing_radians] control_input_1 : pose CAN, control_input_2 : vehicle monitor
% 
% % Usage script scene0655 - suitable for constant velocity motion model
% % scene0916 - suitable for particle filter evaluation
% scene_name = ["0655","0916"];
% scene_name = scene_name(2);
% 
% % Common workspaces
% load("nuscenes_implementation\workspaces\ground_truth-scene"+scene_name+".mat","ground_truth_ego_pos");
% load("nuscenes_implementation\workspaces\control_input_1-scene"+scene_name+".mat","control_input");
% load("nuscenes_implementation\workspaces\timestamp-scene"+scene_name+".mat","timestamp");
% % load("nuscenes_implementation\workspaces\control_input_2-scene"+scene_name+".mat","control_input");
% 
% % % For one detection per target - Point object processing
% % load("nuscenes_implementation\workspaces\global_landmarks_map-scene"+scene_name+".mat","global_landmarks_map");
% % load("nuscenes_implementation\workspaces\measurements-scene"+scene_name+".mat","measurements");
% % load("nuscenes_implementation\workspaces\measurements_global-scene"+scene_name+".mat","measurements_global");
% 
% % For multiple detections per target - Extended object processing
% % load("nuscenes_implementation\workspaces\global_landmarks_map-multi_box_scene"+scene_name+".mat","global_landmarks_map");
% % load("nuscenes_implementation\workspaces\measurements-multi_box-scene"+scene_name+".mat","measurements");
% % load("nuscenes_implementation\workspaces\measurements_multi_box-global-scene"+scene_name+".mat","measurements_global");
% 
% ax = gca;
% hold on;
% 
% % Plot ground truth trajectory of the ego car
% plot(ground_truth_ego_pos(1,:), ground_truth_ego_pos(2,:), 'b-o', 'MarkerFaceColor', 'b');
% 
% % Plot landmarks in global coordinates
% global_landmarks_map_keys = global_landmarks_map.keys;
% for i = 1: length(global_landmarks_map.keys)
% 	land = global_landmarks_map(global_landmarks_map_keys{i});
% 	plot(land(1), land(2), "b*");
% 	% text(land(1), land(2), global_landmarks_map_keys{i}, "Color", "b");
% end
% 
% % Plot detections for the current timestamp
% for i = 1: size(timestamp,2) % Could change index to start evaluatation from 2nd timestamp instead of 1st timestamp
% 	% Here comes the particle filter step
% 	
% 	% Plotting
% 	plot(ground_truth_ego_pos(1,i), ground_truth_ego_pos(2,i), "m*", "Tag","current_ground_truth_position");
% 	scatter(measurements_global{i}.zpos(1,:), measurements_global{i}.zpos(2,:), 'Marker','o','MarkerFaceColor' ,'g', 'MarkerEdgeColor','g', "Tag", "mean_cluster");
% 	scatter(measurements_global{i}.detections(1,:), measurements_global{i}.detections(2,:), 'Marker','o', 'MarkerEdgeColor','r', "Tag", "unclustered");
% 	% text(measurements_global{i}.zpos(1,:),
% 	% measurements_global{i}.zpos(2,:), measurements_global{i}.id', "Tag",
% 	% "cluster_id", "Color", "g");
% 	pause(1);
% 	delete(findall(ax, "Tag","current_ground_truth_position"));
% 	delete(findall(ax, "Tag","mean_cluster"));
% 	delete(findall(ax, "Tag","unclustered"));
% 	delete(findall(ax, "Tag", "cluster_id"));
% end



