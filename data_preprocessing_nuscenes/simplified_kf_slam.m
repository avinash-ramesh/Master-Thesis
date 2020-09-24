%% SLAM
clc;
close all;
clear all;
scene_name = ["0655","0061","0916"];
scene_name = scene_name(3);
dead_reckoning = false;
extended_object_processing = true;
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
    
%     for i = 1:size(states_k,2)
%         plot(states_k(1,i),states_k(3,i),'bo','MarkerFaceColor', 'b');
% %         delete(findall(gca, 'Tag', 'error_ellipse'));
%         vehicle_pos = states_k([1,3],i);
%         vehicle_cov = blkdiag(states_cov_k(1,i),states_cov_k(3,i));
%         print_error_ellipse(vehicle_pos, vehicle_cov, 0.95);
%         drawnow;
% %         pause(0.5);
%     end
%     plot(states_k(1,:),states_k(3,:),'bo-','MarkerFaceColor', 'b','Tag', 'estimated_trajectory');
%     hold off; 
%     l1 = findall(gca, 'Tag', 'estimated_trajectory');
%     l2 = findall(gca, 'Tag', 'error_ellipse');
%     legend([l1(1), l2(1)], ["Estimated Trajectory", "Position covariance with CI 0.95"],'Location','southeast');
    

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
    
    drawnow;
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
    A = blkdiag([1,T; 0,0],[1,T; 0,0]); % State transition matrix A
    B = [0,0;1,0;0,0;0,1]; % Control Matrix B
    sigma_vx = 200; %100, 0.1
    sigma_vy = 200; %100, 0.1
    Q = blkdiag([T^3/3, T^2/2; T^2/2, T],[T^3/3, T^2/2; T^2/2, T]) * blkdiag(sigma_vx, sigma_vx, sigma_vy, sigma_vy); % Brownian Process Noise Covariance Q
    
    for i = 1: (size(X,1) - n_states)/2
       A = blkdiag(A,1,1);
       Q = blkdiag(Q,0,0); 
       B = [B; zeros(2)]; 
    end        
    X_pre = A * X + B * u; % Predict State
    P_pre = A * P * A' + Q; % Predict State Covariance
end

% Update equations
function [X, P, K, R, Y, S] = update(X_pre, P_pre, Z)
    global n_states;
    global landmark_index_map;   
    
    % Measurement/Observation matrix H
    size_state = size(X_pre,1);
    H = [-1, 0, 0, 0, zeros(1,size_state-n_states);...
         0, 0, -1, 0, zeros(1,size_state-n_states)];
    R = blkdiag(25,25); % Assume that the detections always fall within the bounding box of 7x7 m2 area which is 3sig boundary. (5,5), (0.5,0.5)
    for tmp = 1:size(Z.id,2)
        if landmark_index_map.isKey(string(Z.id(tmp)))
            H(1,landmark_index_map(string(Z.id(tmp))))   = 1;
            H(2,landmark_index_map(string(Z.id(tmp)))+1) = 1;     
            break;
        end
    end
     
    % Use the existing landmarks for self localization
    for m = tmp+1:size(Z.id,2)
        if landmark_index_map.isKey(string(Z.id(m)))
            H_ = [-1, 0, 0, 0, zeros(1,size_state-n_states);...
                  0, 0, -1, 0, zeros(1,size_state-n_states)];
            H_(1,landmark_index_map(string(Z.id(m))))   = 1;
            H_(2,landmark_index_map(string(Z.id(m)))+1) = 1;
            H = vertcat(H,H_);
            R = blkdiag(R,25,25);
        end
    end
    
    % Innovation 
    Z_ = [];
    for m = 1:size(Z.id,2)
        if landmark_index_map.isKey(string(Z.id(m)))
            Z_ = [Z_, Z.zpos(1:2,m)];
        end
    end
	if ~isempty(Z_)
        Y = Z_(:) - H * X_pre; % Z_(:) will flatten the matrix in 1D column vector
        % Innovation covariance
        S = H * P_pre * H' + R;
        % Kalman Gain
        K = P_pre * H' * S^(-1);
        % Update State 
        X = X_pre + K * Y;
        % Update State Covariance
        P = (eye(size(X,1)) - K*H) * P_pre;
        P = (P + P')/2; % This is done to retain symmetric state covariance matrix
    else
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

function Z_ = rotate_measurements_to_global_frame(Z_,orientation)
    q_rotmat = [cos(orientation), -sin(orientation), 0;
                sin(orientation),  cos(orientation), 0;
                0               ,  0               , 1];
    % q = quaternion(angle2quat(orientation, 0, 0, 'ZYX'));
    % q_rotmat = rotmat(q,'point');   
    temp = rotate_point_to_global_frame([Z_.zpos(1:2,:);zeros(1,size(Z_.zpos,2))],q_rotmat);
    Z_.zpos(1:2,:) = temp(1:2,:);
end

function point = rotate_point_to_global_frame(point, rotation_matrix)
    point(1:3,:) = rotation_matrix * point(1:3,:);
end

%% Backup - 2
% %% SLAM
% clc;
% close all;
% clear all;
% scene_name = ["0655","0916"];
% scene_name = scene_name(2);
% dead_reckoning = false;
% global n_states;
% global landmark_index_map;
% landmark_index_map = containers.Map;
% 
% load("nuscenes_implementation\workspaces\ground_truth-scene"+scene_name+".mat","ground_truth_ego_pos");
% load("nuscenes_implementation\workspaces\control_input_1-scene"+scene_name+".mat","control_input");
% load("nuscenes_implementation\workspaces\timestamp-scene"+scene_name+".mat","timestamp");
% % For one detection per target - Point object processing
% load("nuscenes_implementation\workspaces\global_landmarks_map-scene"+scene_name+".mat","global_landmarks_map");
% load("nuscenes_implementation\workspaces\measurements-scene"+scene_name+".mat","measurements");
% load("nuscenes_implementation\workspaces\measurements_global-scene"+scene_name+".mat","measurements_global");
% 
% ax = gca;
% set(gcf, 'Position', get(0, 'Screensize'));
% hold on;
% plot_scenario(ground_truth_ego_pos, global_landmarks_map);
% 
% %% Filter Initialization
% timestep = 1;
% n_states = 4;
% 
% x = ground_truth_ego_pos(1,timestep);
% vx = control_input(1,timestep) * cos(control_input(2,timestep));
% y = ground_truth_ego_pos(2,timestep);
% vy = control_input(1,timestep) * sin(control_input(2,timestep));
% X_pre = [x,vx,y,vy]'; % State vector X = [x, vx , y, vy]
% P_pre = blkdiag(45, 25, 45, 25); % position with 20m 3sig error, and velocity with 5m/s 1sig error.
% 
% % First Update
% if dead_reckoning
%     X = X_pre;
%     P = P_pre;
% else
%     [Z_, Z_plot] = get_current_measurement(timestep, measurements, measurements_global);
% 	[X, P, K, R, Y, S] = update(X_pre, P_pre, Z_);
% end
% plot_on_global_map(timestep, X_pre, X, ground_truth_ego_pos, measurements_global, ax);
% % pause(1);
% 
% prev_time = timestamp(timestep);
% T = 0;
% 
% %% Predict and Update steps
% for timestep = 2 : size(timestamp,2)
%     T = T + (timestamp(timestep) - prev_time) * 1e-6;
%     [Z_, Z_plot] = get_current_measurement(timestep, measurements, measurements_global);
%     
%     if size(Z_.id,2) ~= 0 % Perform filtering only when measurements in the Z variable are present
%         u = [control_input(1,timestep) * cos(control_input(2,timestep));
%              control_input(1,timestep) * sin(control_input(2,timestep))];
%         [X_pre, P_pre, Q, B] = predict(X, P, T, Z_, u);
%         if dead_reckoning
%             X = X_pre;
%             P = P_pre;
%         else
%             [X, P, K, R, Y, S] = update(X_pre, P_pre, Z_); 
%         end
%         plot_on_global_map(timestep, X_pre, X, ground_truth_ego_pos, measurements_global, ax);
% %         pause(1);
%         T = 0;
%     end
%     prev_time = timestamp(timestep);
% end
% hold off;
% 
% %% Helper functions
% function plot_scenario(ground_truth_ego_pos, global_landmarks_map)
%     % Plot ground truth trajectory of the ego car
%     plot(ground_truth_ego_pos(1,:), ground_truth_ego_pos(2,:), 'b-o', 'MarkerFaceColor', 'b');
% 
%     % Plot landmarks in global coordinates
%     global_landmarks_map_keys = global_landmarks_map.keys;
%     for i = 1: length(global_landmarks_map.keys)
%         land = global_landmarks_map(global_landmarks_map_keys{i});
%         plot(land(1), land(2), "b*");
%         % text(land(1), land(2), global_landmarks_map_keys{i}, "Color", "b");
%     end
% end
% 
% function plot_on_global_map(timestep, X_pre, X, ground_truth_ego_pos, measurements_global, ax)
%     legend off;
%     delete(findall(ax, "Tag","current_ground_truth_position"));
% 	delete(findall(ax, "Tag","mean_cluster"));
% 	delete(findall(ax, "Tag","unclustered"));
% 	delete(findall(ax, "Tag", "cluster_id"));
%     
%     plot(X_pre(1), X_pre(3),'c*');
%     plot(X(1), X(3),'k*');
% 	plot(ground_truth_ego_pos(1,timestep), ground_truth_ego_pos(2,timestep), "m*", "Tag","current_ground_truth_position");
% 	scatter(measurements_global{timestep}.zpos(1,:), measurements_global{timestep}.zpos(2,:), 'Marker','o','MarkerFaceColor' ,'g', 'MarkerEdgeColor','g', "Tag", "mean_cluster");
% 	scatter(measurements_global{timestep}.detections(1,:), measurements_global{timestep}.detections(2,:), 'Marker','o', 'MarkerEdgeColor','r', "Tag", "unclustered");
% 	% text(measurements_global{i}.zpos(1,:), measurements_global{i}.zpos(2,:), measurements_global{i}.id', "Tag", "cluster_id", "Color", "g");
% end
% 
% function [Z_, Z_plot] = get_current_measurement(timestep, measurements, measurements_global)
%     Z_ = struct;
%     Z_.id = measurements{timestep}.id';
%     Z_.zpos = measurements{timestep}.zpos;
%     Z_.detections = measurements{timestep}.detections;
%     
%     Z_plot = struct;
%     Z_plot.id = measurements_global{timestep}.id';
%     Z_plot.zpos = measurements_global{timestep}.zpos;
%     Z_plot.detections = measurements_global{timestep}.detections;
% end
% 
% function [X_pre, P_pre, Q, B] = predict(X, P, T, Z, u)
%     global landmark_index_map;
%     global n_states;
%     A = blkdiag([1,0; 0,0],[1,0; 0,0]); % State transition matrix A
%     B = [T,0;1,0;0,T;0,1]; % Control Matrix B
%     sigma_vx = 1;
%     sigma_vy = 1;
%     Q = blkdiag([T^3/3, T^2/2; T^2/2, T],[T^3/3, T^2/2; T^2/2, T]) * blkdiag(sigma_vx, sigma_vx, sigma_vy, sigma_vy); % Brownian Process Noise Covariance Q
%     
%     for i = 1: (size(X,1) - n_states)/2
%        A = blkdiag(A,1,1);
%        Q = blkdiag(Q,0,0); 
%        B = [B; zeros(2)];
%     end
%     
% %     % Adjust the matrices depending on the number of landmarks observed in
% %     % the current iteration
% %     for m = 1:size(Z.id,2)
% %         if landmark_index_map.isKey(string(Z.id(m)))
% %             Q(landmark_index_map(string(Z.id(m))):landmark_index_map(string(Z.id(m)))+1, landmark_index_map(string(Z.id(m))):landmark_index_map(string(Z.id(m)))+1) = blkdiag(1,1);
% %         end
% %     end
%         
%     X_pre = A * X + B * u; % Predict State
%     P_pre = A * P * A' + Q; % Predict State Covariance
% end
% 
% % Update equations
% function [X, P, K, R, Y, S] = update(X_pre, P_pre, Z)
%     global n_states;
%     global landmark_index_map;   
%     
%     % Measurement/Observation matrix H
%     size_state = size(X_pre,1);
%     H = [-1, 0, 0, 0, zeros(1,size_state-n_states);...
%          0, 0, -1, 0, zeros(1,size_state-n_states)];
%     R = blkdiag(6,6); % Assume that the detections always fall within the bounding box of 7x7 m2 area which is 3sig boundary.
% 
%     for tmp = 1:size(Z.id,2)
%         if landmark_index_map.isKey(string(Z.id(tmp)))
%             H(1,landmark_index_map(string(Z.id(tmp))))   = 1;
%             H(2,landmark_index_map(string(Z.id(tmp)))+1) = 1;     
%             break;
%         end
%     end
%      
%     % Use the existing landmarks for self localization
%     for m = tmp+1:size(Z.id,2)
%         if landmark_index_map.isKey(string(Z.id(m)))
%             H_ = [-1, 0, 0, 0, zeros(1,size_state-n_states);...
%                 0, 0, -1, 0, zeros(1,size_state-n_states)];
%             H_(1,landmark_index_map(string(Z.id(m))))   = 1;
%             H_(2,landmark_index_map(string(Z.id(m)))+1) = 1;
%             H = vertcat(H,H_);
%             R = blkdiag(R,6,6);
%         end
%     end
%     
%     % Innovation 
%     Z_ = [];
%     for m = 1:size(Z.id,2)
%         if landmark_index_map.isKey(string(Z.id(m)))
%             Z_ = [Z_, Z.zpos(1:2,m)];
%         end
%     end
% 	if ~isempty(Z_)
%         Y = Z_(:) - H * X_pre; % Z_(:) will flatten the matrix in 1D column vector
%         % Innovation covariance
%         S = H * P_pre * H' + R;
%         % Kalman Gain
%         K = P_pre * H' * S^(-1);
%         % Update State 
%         X = X_pre + K * Y;
%         % Update State Covariance
%         P = (eye(size(X,1)) - K*H) * P_pre;
%         P = (P + P')/2; % This is done to retain symmetric state covariance matrix
%     else
%         X = X_pre;
%         P = P_pre;
%         K = NaN;
%         R = NaN;
%         Y = NaN;
%         S = NaN;
%     end
%     
%     % Add new landmarks to the correct position of the vehicle to create map
%     for m = 1:size(Z.id,2)
%         if ~landmark_index_map.isKey(string(Z.id(m)))
%             landmark_index_map(string(Z.id(m))) = size(X,1)+1;
%             X = vertcat(X, Z.zpos(1:2,m) + [X(1);X(3)]);
%             P = blkdiag(P,25,25);
%         end
%     end
% end
% 
% function run_consistency_checks(filter_consistency)
%     displacement_error_k = [];
% %     estimation_gof_k = []; % NEES Test for goodness of fit at 95% confidence or 5% significance
% %     innovation_gof_k = []; % NIS Test for goodness of fit at 95% confidence or 5% significance
%     states_k = [];
%     states_cov_k = [];
%     ground_truth = [];
% %     innov_k = [];
% %     innov_cov_k = [];
%     for i = 1:length(filter_consistency)
%         states_k = [states_k, filter_consistency(i).X(1:4,1)];
%         states_cov_k = [states_cov_k, diag(filter_consistency(i).P(1:4,1:4))];
%         ground_truth = [ground_truth, filter_consistency(i).ground_truth(:,1)];
%         displacement_error_k = [displacement_error_k, sqrt(sum((filter_consistency(i).X([1,3],1) - filter_consistency(i).ground_truth(:,1)).^2))];
% %         estimation_error_ = filter_consistency(i).X([1,3],1) - filter_consistency(i).ground_truth(:,1);
% %         estimation_error = (estimation_error_'/filter_consistency(i).P([1,3],[1,3])) * estimation_error_;
% %         estimation_gof_k = [estimation_gof_k, estimation_error]; % Group data into 4 bins, so degree of freedom is 4-1 = 3.
% %         innovation_error = (filter_consistency(i).Y'/filter_consistency(i).S) * filter_consistency(i).Y;
% %         innovation_gof_k = [innovation_gof_k, innovation_error]; 
%     end
% 
%     % Check 1: Displacement (Eucledian distance) Error (minADE_k) - The pointwise L2 distances between the predicted trajectory and ground truth.  
%     mean_error = mean(displacement_error_k);
%     fprintf("Mean displacement error is: %f\n", mean_error);
% 
%     % Check 2: Final Displacement Error (minFDE_k) - The L2 distance between the final points of the prediction and ground truth.
%     fprintf("Final displacement error is: %f\n", displacement_error_k(end));
%     
%     % Check 3: Miss rate at 1 meters - If pointwise L2 distance between the
%     % prediction and ground truth is greater than 1 meters, we define it as a miss.
%     fprintf("Miss rate at 1 meters (in percentage): %f\n", sum(displacement_error_k > 1) * 100 / length(displacement_error_k));
%     
%     % Check 4: Visual inspection of state covariance, the covariance should
%     % decrease with time or become stationary
%     figure;
%     hold on;
%     plot(states_cov_k(1,:));
%     plot(states_cov_k(2,:));
%     plot(states_cov_k(3,:));
%     plot(states_cov_k(4,:));
%     legend('Covariance in x','Covariance in vx','Covariance in y','Covariance in vy');
%     hold off;
%     
%     % Check 5: Weak consistency check: Estimated error should be gaussian with variance equal to state covariance.
%     estimated_error = mean(states_k([1,3],:)-ground_truth, 2);
%     estimated_error_cov = var(states_k([1,3],:)-ground_truth, 0, 2);
%     cov_diff = states_cov_k - blkdiag(estimated_error_cov(1), estimated_error_cov(2));
%     ispossemidef = false;
%     if issymmetric(cov_diff)
%         eigen_values = eig(cov_diff);
%         ispossemidef = all(eigen_values > 0);
%     end
%     fprintf("Weak consistency check: \n");
%     fprintf("Mean absolute estimated error: [x = %s, y = %s]\n", estimated_error(1), estimated_error(2));
%     fprintf("Estimated error covariance is less than state covariance: %b", ispossemidef);
%     
%     
%     
% %     % Check 5: Normalized Estimation Error Squared Test
% %     [h,p] = chi2gof(estimation_gof_k);
% %     fprintf("NEES test for goodness of fit (0: Normal distribution, 1: Other distribution): %f with a p value of %f at a significance level of %s\n", h, p, "5%");
% %     
% %     % Check 6: Normalized Innovation Squared Test
% %     [h,p] = chi2gof(innovation_gof_k);
% %     fprintf("NIS test for goodness of fit (0: Normal distribution, 1: Other distribution): %f with a p value of %f at a significance level of %s\n", h, p, "5%");
% end