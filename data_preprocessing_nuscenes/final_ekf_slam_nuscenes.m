clc;
close all;
clear all;

format compact;
format longe;

%% Data loading
global scene_name;
global extended_object_processing;
scene_name = ["0655","0061","0916"];
scene_name = scene_name(1);
extended_object_processing = false;
dead_reckoning = false;
consistency_checking = false;
map_scaling_factor = 10;
v = VideoWriter("nuscenes_implementation\videos\ekfslam-extended-with_controls-scene"+scene_name+".avi");
can_message = ["pose","vehicle_monitor","zoe_info","zoe_odo","zoe_wheel"];
can_message = can_message(1);
load("nuscenes_implementation\workspaces\radar_"+scene_name+".mat","radar");
if ~extended_object_processing
    load("nuscenes_implementation\workspaces\simplified_clustered_detections-one_box_carlos-all_radars-default_filters-40m-"+scene_name+".mat","simplified_cluster_detections");
else
    load("nuscenes_implementation\workspaces\simplified_clustered_detections-multi_box_carlos-all_radars-default_filters-40m-"+scene_name+".mat","simplified_cluster_detections");
end

%% Control input sources
detections = simplified_cluster_detections;
ordered_timestamps = sort(double(string(detections.keys)));
ego_canbus_aligned = generate_control_input(ordered_timestamps, can_message, scene_name);

%% Plot landmarks on Global map along with ego poses
transform_landmarks_to_global_frame;
% ### Update the moving landmarks array from this script's output

%% Formatting the measurements as required and transforming them to global frame
[Z, Z_for_plot] = generate_measurements(ordered_timestamps, detections, ego_translation, ego_orientation, extended_object_processing);

%% Global variables
global landmark_index_map;
global filter_collected_data;
global n_states;
landmark_index_map = containers.Map;
filter_collected_data.variable = containers.Map;

%% EKF SLAM
hold on;
ax = gca;

timestep = 1; % Perform a first update
n_states = 4;
u = get_current_control_input("pose", timestep, ordered_timestamps, ego_canbus_aligned);
x = ego_translation(1,timestep);
vx = u(1);
y = ego_translation(2,timestep);
vy = u(2);
X_pre = [x,vx,y,vy]'; % State vector X = [x, vx , y, vy]
P_pre = blkdiag(45, 25, 45, 25); % State Covariance matrix P % position with 20m 3sig error, and velocity with 5m/s 1sig error.
filter_collected_data.X_prior = X_pre;
filter_collected_data.P_prior = P_pre;

[Z_, Z_plot] = get_current_measurement(timestep, Z, Z_for_plot);
if dead_reckoning
	X = X_pre;
	P = P_pre;
else
    Z_ = rotate_measurements_to_global_frame(Z_, ego_canbus_aligned(string(ordered_timestamps(timestep))).bearing_radians);
	[X, P, K, R, Y, S] = update(X_pre, P_pre, Z_);
	if consistency_checking
        filter_collected_data.variable(string(timestep)) = collect_filter_data(ordered_timestamps(timestep), X_pre, P_pre, X, P, [ego_translation(1,timestep); ego_translation(2,timestep)], K, Y, S, NaN, R, Z{timestep}, Z_plot, NaN, NaN, landmark_index_map);
	end
end

bep = birdsEyePlot('Parent', ax);
if scene_name == "0061"
    camroll(-180);
else
    camroll(-90);
end
legend('off');
[predicted_pose, estimated_pose, current_ground_truth, clustered_det_plot,unclustered_det_plot, landmark_pos] = plot_on_global_map(ax, bep, timestep, X_pre, X, ego_translation, ego_orientation, radar, Z_plot, map_scaling_factor);
legend([predicted_pose, estimated_pose, current_ground_truth, ground_truth_track, static_landmarks, moving_landmarks,clustered_det_plot,unclustered_det_plot, landmark_pos],...
            ["Predicted position","Estimated position","Actual position","Ground truth track","Static landmarks","Moving landmarks","Clustered detections","Unclustered detections","Landmark estimate"],...
            'Location','northwest');
        
set(gcf, 'Position', get(0, 'Screensize'));
InSet = get(gca, 'TightInset');
set(gca, 'Position', [InSet(1:2), 1-InSet(1)-InSet(3), 1-InSet(2)-InSet(4)]);
v.FrameRate = 2;
open(v);
A = getframe(gcf);
writeVideo(v, A);

prev_time = ordered_timestamps(timestep);
T = 0; % Delta T

for timestep = 2 : size(Z,1)
    T = T + (ordered_timestamps(timestep) - prev_time) * 1e-6;
    [Z_, Z_plot] = get_current_measurement(timestep, Z, Z_for_plot);
    
    if size(Z_.id,2) ~= 0 % Perform filtering only when measurements in the Z variable are present
        u = get_current_control_input(can_message, timestep, ordered_timestamps, ego_canbus_aligned);
        [X_pre, P_pre, Q, B] = predict(X, P, T, u, can_message);
        if dead_reckoning
            X = X_pre;
            P = P_pre;
        else
            Z_ = rotate_measurements_to_global_frame(Z_, ego_canbus_aligned(string(ordered_timestamps(timestep))).bearing_radians);
            [X, P, K, R, Y, S] = update(X_pre, P_pre, Z_); 
            if consistency_checking
                filter_collected_data.variable(string(timestep)) = collect_filter_data(ordered_timestamps(timestep), X_pre, P_pre, X, P, [ego_translation(1,timestep); ego_translation(2,timestep)], K, Y, S, NaN, R, Z{timestep}, Z_plot, NaN, NaN, landmark_index_map);
            end
        end
        [predicted_pose, estimated_pose, current_ground_truth, clustered_det_plot,unclustered_det_plot, landmark_pos] = plot_on_global_map(ax, bep, timestep, X_pre, X, ego_translation, ego_orientation, radar, Z_plot, map_scaling_factor);
        legend([predicted_pose, estimated_pose, current_ground_truth, ground_truth_track, static_landmarks, moving_landmarks,clustered_det_plot,unclustered_det_plot, landmark_pos],...
            ["Predicted position","Estimated position","Actual position","Ground truth track","Static landmarks","Moving landmarks","Clustered detections","Unclustered detections", "Landmark estimate"],...
            'Location','northwest');
        A = getframe(gcf);
        writeVideo(v, A);
        T = 0;
    end
    prev_time = ordered_timestamps(timestep);
end

%hold off;
% close(v);

%% Save filter collected data
% save("nuscenes_implementation\workspaces\kf_slam_filter_collected_data_scene-"+scene_name+".mat", "filter_collected_data");
A = getframe(gcf);
writeVideo(v, A);
A = getframe(gcf);
writeVideo(v, A);
A = getframe(gcf);
writeVideo(v, A);
A = getframe(gcf);
writeVideo(v, A);
A = getframe(gcf);
writeVideo(v, A);
A = getframe(gcf);
writeVideo(v, A);
A = getframe(gcf);
writeVideo(v, A);
A = getframe(gcf);
writeVideo(v, A);
A = getframe(gcf);
writeVideo(v, A);
hold off;
close(v);


%% Functions
% Prediction
function [X_pre, P_pre, Q, B] = predict(X, P, T, u, can_message)
    global landmark_index_map;
    global n_states;
    A = blkdiag([1,T; 0,0],[1,T; 0,0]); % State transition matrix A

    if can_message == "zoe_odo"
        B = [1,0;0,0;0,1;0,0]; % Control Matrix B
    else
        B = [0,0;1,0;0,0;0,1]; % Control Matrix B
    end
	
    sigma_v = 1; % Maximum change in velocity is 5 m/s in 3sig
    sigma_theta = 0.008; % Maximum change in orientation is 45deg in 3sig
    V = [T*cos(u(2)), -T*u(1)*sin(u(2));
         cos(u(2))  , -u(1)*sin(u(2))   ;
         T*cos(u(2)), T*u(1)*cos(u(2)) ;
         sin(u(2))  , u(1)*cos(u(2))  ];
     
    Q = V * blkdiag(sigma_v, sigma_theta) * V';
    
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
            R = blkdiag(1,0.2); %one_0916:5,5 ; multi_0916: 0.1, 0.005; one_0655:5,0.001 ; multi_0655: 0.1, 0.005
            
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

function ego_canbus_aligned = generate_control_input(ordered_timestamps, can_message, scene_name)
    ego_canbus_aligned = containers.Map;
    if can_message == "pose"
        % Align the control inputs with the timestamp of keyframe - Pose CAN bus message (vel in m/s, orientation in quaternion)
        ego_canbus = jsondecode(fileread(join(["datasets\NuScenes\dataset_v1.0-mini","can_bus","scene-"+scene_name+"_pose.json"],filesep)));
        for i = 1:length(ordered_timestamps)
            [~, ind_min] = min(abs([ego_canbus.utime] - ordered_timestamps(i)));
            ego_canbus(ind_min).velocity = ego_canbus(ind_min).vel(1);
            q = quaternion(ego_canbus(ind_min).orientation');
            bearing_radians = euler(q, 'ZYX', 'point');
            ego_canbus(ind_min).bearing_radians = bearing_radians(1);
            ego_canbus_aligned(string(ordered_timestamps(i))) = ego_canbus(ind_min);
        end
    elseif can_message == "vehicle_monitor"
        % Align the control inputs with the timestamp of keyframe - Vehicle monitor CAN bus message (vehicle_speed in Km/h, steering in deg)
        ego_canbus = jsondecode(fileread(join(["datasets\NuScenes\dataset_v1.0-mini","can_bus","scene-"+scene_name+"_vehicle_monitor.json"],filesep)));
        for i = 1:length(ordered_timestamps)
            [~, ind_min] = min(abs([ego_canbus.utime] - ordered_timestamps(i)));
            ego_canbus(ind_min).velocity = ego_canbus(ind_min).vehicle_speed / 3.6 ;
            ego_canbus(ind_min).bearing_radians = deg2rad(ego_canbus(ind_min).steering);
            ego_canbus_aligned(string(ordered_timestamps(i))) = ego_canbus(ind_min);
        end
    elseif can_message == "zoe_info"
        % Align the control inputs with the timestamp of keyframe - Zoe vehicle information CAN bus message (odom_speed in Km/h, steer_corrected in deg)
        ego_canbus = jsondecode(fileread(join(["datasets\NuScenes\dataset_v1.0-mini","can_bus","scene-"+scene_name+"_zoe_veh_info.json"],filesep)));
        for i = 1:length(ordered_timestamps)
            [~, ind_min] = min(abs([ego_canbus.utime] - ordered_timestamps(i)));
            ego_canbus(ind_min).velocity = ego_canbus(ind_min).odom_speed / 3.6 ;
            ego_canbus(ind_min).bearing_radians = deg2rad(ego_canbus(ind_min).steer_corrected);
            ego_canbus_aligned(string(ordered_timestamps(i))) = ego_canbus(ind_min);
        end
    elseif can_message == "zoe_odo"
        % Align the control inputs with the timestamp of keyframe - Zoe vehicle information CAN bus message (odom in cm, steer_corrected in deg)
        % Circumference of Zoe car wheels is 0.305
        ego_canbus = jsondecode(fileread(join(["datasets\NuScenes\dataset_v1.0-mini","can_bus","scene-"+scene_name+"_zoe_veh_info.json"],filesep)));
        ego_canbus_aligned = containers.Map;
        for i = 1:length(ordered_timestamps)
            [~, ind_min] = min(abs([ego_canbus.utime] - ordered_timestamps(i)));
            ego_canbus(ind_min).position = ego_canbus(ind_min).odom / 100 ;
            ego_canbus(ind_min).bearing_radians = deg2rad(ego_canbus(ind_min).steer_corrected);
            ego_canbus_aligned(string(ordered_timestamps(i))) = ego_canbus(ind_min);
        end
    elseif can_message == "zoe_wheel"
        % Align the control inputs with the timestamp of keyframe - Zoe vehicle information CAN bus message (FL_wheel_speed and FR_wheel_speed in rounds per minute, steer_corrected in deg)
        % Conversion of wheel speed to vehicle speed in m/s by multiplying itself with 2 * pi * 0.305 / 60
        ego_canbus = jsondecode(fileread(join(["datasets\NuScenes\dataset_v1.0-mini","can_bus","scene-"+scene_name+"_zoe_veh_info.json"],filesep)));
        ego_canbus_aligned = containers.Map;
        for i = 1:length(ordered_timestamps)
            [~, ind_min] = min(abs([ego_canbus.utime] - ordered_timestamps(i)));
            FR_veloctiy = ego_canbus(ind_min).FR_wheel_speed * 2 * pi * 0.305 / 60 ;
            FL_veloctiy = ego_canbus(ind_min).FL_wheel_speed * 2 * pi * 0.305 / 60 ;
            ego_canbus(ind_min).velocity = (FL_veloctiy + FR_veloctiy) / 2;
            ego_canbus(ind_min).bearing_radians = deg2rad(ego_canbus(ind_min).steer_corrected);
            ego_canbus_aligned(string(ordered_timestamps(i))) = ego_canbus(ind_min);
        end
    end
end

function [Z, Z_for_plot] = generate_measurements(ordered_timestamps, detections, ego_translation, ego_orientation, extended_object_processing)
    global scene_name;    
    Z = {};
    Z_for_plot = {};
    for i = 1:length(ordered_timestamps)
        tmp = detections(string(ordered_timestamps(i)));
        tmp_struct = struct;
        
        tmp_struct.id = string({tmp.label})';
        if extended_object_processing
            tmp_struct.parent_id = string({tmp.parent_label_id})';
        end
        tmp_struct.zpos = [tmp.mean];
        tmp_struct.detections = [tmp.detections];
        tmp_struct.radial_velocity = [tmp.radial_velocity];
        
%         % Apply RANSAC to filter out moving targets
%         point_cloud = struct;
%         point_cloud.x = 1:length(tmp_struct.radial_velocity);
%         point_cloud.y = tmp_struct.radial_velocity;
%         [~, filter_moving_targets, ~] = ransac2d(point_cloud,length(tmp_struct.radial_velocity)*2,0.1);    
%         
%         if ~extended_object_processing
%             idx = find(ismember(tmp_struct.id, tmp_struct.id(filter_moving_targets.x))); %scene0655-["8","15","20","36","41","51","59","128"], scene0916-["17","18","24","26","27","29","34","35","36","38","41","77","78"]
%         else
%             idx = find(ismember(tmp_struct.parent_id, tmp_struct.parent_id(filter_moving_targets.x))); % These are moving objects, so do not consider them %scene0655-["8","15","20","36","41","51","59","128"], scene0916-["17","18","24","26","27","29","34","35","36","38","41","77","78"]
%         end

        filter_moving_targets = [];
        if scene_name == "0655"
            filter_moving_targets = ["8","15","20","36","41","51","59","128"];
        elseif scene_name == "0061"
            filter_moving_targets = ["1","10","102","104","109","114","117","12","168","179","181","182","19","194","195","199","2","202","203","204","21","23","26","31","32","33","34","39","4","42","48","49","5","57","59","60","62","69","71","76","8","81","82","85","86","88","9"];
        elseif scene_name == "0916"
            filter_moving_targets = ["17","18","24","26","27","29","34","35","36","38","41","77","78"];
        end
        if ~extended_object_processing
            idx = find(ismember(tmp_struct.id, filter_moving_targets)); %scene0655-["8","15","20","36","41","51","59","128"], scene0061-["1","10","102","104","109","114","117","12","168","179","181","182","19","194","195","199","2","202","203","204","21","23","26","31","32","33","34","39","4","42","48","49","5","57","59","60","62","69","71","76","8","81","82","85","86","88","9"], scene0916-["17","18","24","26","27","29","34","35","36","38","41","77","78"]
        else
            idx = find(ismember(tmp_struct.parent_id, filter_moving_targets)); % These are moving objects, so do not consider them %scene0655-["8","15","20","36","41","51","59","128"], scene0061-["1","10","102","104","109","114","117","12","168","179","181","182","19","194","195","199","2","202","203","204","21","23","26","31","32","33","34","39","4","42","48","49","5","57","59","60","62","69","71","76","8","81","82","85","86","88","9"], scene0916-["17","18","24","26","27","29","34","35","36","38","41","77","78"]
        end
        
        tmp_struct.id(idx) = [];
        if extended_object_processing
            tmp_struct.parent_id(idx) = [];
        end
        tmp_struct.zpos(:,idx) = [];
        tmp_struct.radial_velocity(idx) = [];
        tmp_struct.detections(:,idx) = [];
        
        Z = [Z; tmp_struct];

        tmp_struct.zpos(3,:) = 0;
        tmp_struct.detections(3,:) = 0;

        q = quaternion(ego_orientation(:,i)');
        q_rotmat = rotmat(q,'point');

        tmp_struct.zpos = rotate_point_to_global_frame(tmp_struct.zpos,q_rotmat);
        tmp_struct.zpos = translate_point_to_global_frame(tmp_struct.zpos,ego_translation(:,i));

        tmp_struct.detections = rotate_point_to_global_frame(tmp_struct.detections,q_rotmat);
        tmp_struct.detections = translate_point_to_global_frame(tmp_struct.detections,ego_translation(:,i));

        tmp_struct.zpos = tmp_struct.zpos(1:2,:);
        tmp_struct.detections = tmp_struct.detections(1:2,:);
        Z_for_plot = [Z_for_plot; tmp_struct];
    end
end

function [segmented_inliers, segmented_outliers, separating_line] = ransac2d(point_cloud,niterations,disttol)
    finalized_inliers = [];
    
    for i=1:niterations
        choosen_point = randi(size(point_cloud.x,2));
        inliers = [choosen_point];
        while size(inliers,2)<2
            choosen_point = randi(size(point_cloud.x,2));
            inliers = unique([inliers, choosen_point]);
        end 
        
        x1 = point_cloud.x(inliers(1));
        y1 = point_cloud.y(inliers(1));
        x2 = point_cloud.x(inliers(2));
        y2 = point_cloud.y(inliers(2));
        
        A = y1-y2;
        B = x2-x1;
        C = (x1*y2)-(x2*y1);
        
        for j=1:size(point_cloud.x,2)
            if any(ismember(inliers,j))
               continue; 
            end
            x3 = point_cloud.x(j);
            y3 = point_cloud.y(j);
            cal_distance = abs((A*x3)+(B*y3)+C)/sqrt((A^2)+(B^2));
            if cal_distance <= disttol
                inliers = unique([inliers, j]);
            end
        end
        
        if size(finalized_inliers,2) < size(inliers,2)
            finalized_inliers = inliers;
            separating_line.x = [x1 x2];
            separating_line.y = [y1 y2];
        end
    end
    segmented_inliers.x = point_cloud.x(finalized_inliers);
    segmented_inliers.y = point_cloud.y(finalized_inliers);
    
    segmented_outliers.x = [];
    segmented_outliers.y = [];
    for k=1:size(point_cloud.x,2)
        if ~any(ismember(finalized_inliers,k))
            segmented_outliers.x = [segmented_outliers.x point_cloud.x(k)];
            segmented_outliers.y = [segmented_outliers.y point_cloud.y(k)];
        end
    end
end

function collected_data = collect_filter_data(ordered_timestamp, X_pre, P_pre, X, P, ground_truth, K, Y, S, Q, R, Z, Z_plot, B, u, landmark_index_map)
    collected_data = struct;
    collected_data.timestamp = ordered_timestamp;
    collected_data.X_pre = X_pre;
    collected_data.P_pre = P_pre;
    collected_data.X = X;
    collected_data.P = P;
    collected_data.ground_truth = ground_truth;
    collected_data.K = K;
    collected_data.Y = Y;
    collected_data.S = S;
    collected_data.Q = Q;
    collected_data.R = R;
    collected_data.representative_mean = [Z_plot.zpos(1,:); Z_plot.zpos(2,:)];
    collected_data.unclustered_detections = [Z_plot.detections(1,:); Z_plot.detections(2,:)];
    collected_data.landmark_index_map = landmark_index_map;
    collected_data.Z = Z;
    collected_data.B = B;
    collected_data.u = u;
end

function [Z_, Z_plot] = get_current_measurement(timestep, Z, Z_for_plot)
    Z_ = struct;
    Z_.id = Z{timestep}.id';
    Z_.zpos = Z{timestep}.zpos;
    Z_.detections = Z{timestep}.detections;
    
    Z_plot = struct;
    Z_plot.id = Z_for_plot{timestep}.id';
    Z_plot.zpos = Z_for_plot{timestep}.zpos;
    Z_plot.detections = Z_for_plot{timestep}.detections;
end

function u = get_current_control_input(can_message, timestep, ordered_timestamps, ego_canbus_aligned)
    if can_message == "zoe_odo"
        u_pos = ego_canbus_aligned(string(ordered_timestamps(timestep))).position;
        u_angle = ego_canbus_aligned(string(ordered_timestamps(timestep))).bearing_radians;
        u = [u_pos * cos(u_angle); u_pos * sin(u_angle)];
    else
        u_vel = ego_canbus_aligned(string(ordered_timestamps(timestep))).velocity;
        u_angle = ego_canbus_aligned(string(ordered_timestamps(timestep))).bearing_radians;
        u = [u_vel * cos(u_angle); u_vel * sin(u_angle)];
    end
end

function [predicted_pose, estimated_pose, current_ground_truth, clustered_det_plot,unclustered_det_plot, landmark_pos] = plot_on_global_map(ax, bep, timestep, X_pre, X, ego_translation, ego_orientation, radar, Z_plot, map_scaling_factor)
    global extended_object_processing;
    global landmark_index_map;
    predicted_pose = plot(X_pre(1)*map_scaling_factor, X_pre(3)*map_scaling_factor,'c*');
    estimated_pose = plot(X(1)*map_scaling_factor, X(3)*map_scaling_factor,'k*');
    
    % Plot the ego's current ground truth position along with coverage
    % area, clustered detections and unclustered detections
    delete(findall(ax,'Tag','ego_current_position'));
    delete(findobj(bep.Plotters,'Tag','coverage_area'));
    delete(findall(ax,'Tag','clustered_detections'));
    delete(findall(ax,'Tag','unclustered_detections'));
    delete(findall(ax,'Tag','landmarks'));

    current_ground_truth = plot(ego_translation(1,timestep) * map_scaling_factor, ego_translation(2,timestep) * map_scaling_factor,'m*','Tag','ego_current_position');
    if ~extended_object_processing
        landmark_pos = plot(X(5:2:end).* map_scaling_factor, X(6:2:end).* map_scaling_factor,'b+','Tag','landmarks','MarkerSize',5);
    else
        indices = string(landmark_index_map.keys);
        values = landmark_index_map.values;
        split_indices = split(indices',"_");
        [num_count, index_category] = histcounts(categorical(split_indices(:,1)));
        X_plot = containers.Map;
        for i = 1: length(indices)
            temp = split(indices(i),"_");
            temp = temp(1);
            if X_plot.isKey(temp)
                X_plot(temp) = X_plot(temp) + X(values{i}:values{i}+1, 1);
            else
                X_plot(temp) = X(values{i}:values{i}+1, 1);
            end
        end
        X_plot = cell2mat(X_plot.values) ./ num_count;
        if ~isempty(X_plot)
            landmark_pos = plot(X_plot(1,:).* map_scaling_factor, X_plot(2,:).* map_scaling_factor,'b+','Tag','landmarks','MarkerSize',5);
            plot(X_plot(1,:).* map_scaling_factor, X_plot(2,:).* map_scaling_factor,'bo','Tag','landmarks','MarkerSize',5);
        end
    end
    ego_car_rot_mat = rotmat(quaternion(ego_orientation(:,timestep)'),'point');
    ego_car_trans = [ego_translation(1,timestep), ego_translation(2,timestep)];
    ego_car_yaw = eulerd(quaternion(ego_orientation(:,timestep)'),'ZYX','point'); %yaw pitch roll
    ego_car_yaw = ego_car_yaw(1);
    
    sensor_labels = {'RADAR\_FRONT\_LEFT', 'RADAR\_FRONT', 'RADAR\_FRONT\_RIGHT', 'RADAR\_BACK\_RIGHT', 'RADAR\_BACK\_LEFT'};
    sensor_colors = {'magenta','black','blue','green','cyan'};
    for a = 1:length(radar)
       cap = coverageAreaPlotter(bep,'FaceColor',sensor_colors{a},'EdgeColor','none', 'Tag','coverage_area','DisplayName', sensor_labels{a});
       position = ego_car_rot_mat * [radar{a}.SensorLocation, 1]';
       position = position(1:2)' + ego_car_trans;
       plotCoverageArea(cap, position * map_scaling_factor,...
            40 * map_scaling_factor, radar{a}.Yaw + ego_car_yaw, radar{a}.FieldOfView(1));
    end

    clustered_det_plot = scatter(ax, Z_plot.zpos(1,:) * map_scaling_factor, Z_plot.zpos(2,:) * map_scaling_factor,...
        'Marker','o','MarkerFaceColor' ,'g', 'MarkerEdgeColor','g' ,'Tag','clustered_detections');  
    unclustered_det_plot = scatter(ax, Z_plot.detections(1,:) .* map_scaling_factor, Z_plot.detections(2,:) .* map_scaling_factor,...
        'ro','Tag','unclustered_detections');  
    %print_error_ellipse([X(1); X(3)],blkdiag(P(1),P(3)));
    
    drawnow;
end

function Z_ = rotate_measurements_to_global_frame(Z_,orientation)
    q = quaternion(angle2quat(orientation, 0, 0, 'ZYX'));
    q_rotmat = rotmat(q,'point');   
    temp = rotate_point_to_global_frame([Z_.zpos(1:2,:);zeros(1,size(Z_.zpos,2))],q_rotmat);
    Z_.zpos(1:2,:) = temp(1:2,:);
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

function point = translate_point_to_global_frame(point, translation_matrix)
    point(1:3,:) = point(1:3,:) + translation_matrix;
end

function point = rotate_point_to_global_frame(point, rotation_matrix)
    point(1:3,:) = rotation_matrix * point(1:3,:);
end