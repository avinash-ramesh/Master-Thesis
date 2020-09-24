clc;
close all;
clear all;
random_num_gen = rng(100); % Initialize random number generator
format compact;
format longe;

%% Global variables
global landmark_index_map;
global filter_collected_data;
global n_states;
global global_point_landmark_map;
global scene_name;
global extended_object_processing;

%% Data loading
scene_name = ["0655","0061","0916"];
scene_name = scene_name(3);
extended_object_processing = true;
global_point_landmark_map = containers.Map;
landmark_index_map = containers.Map;
filter_collected_data.variable = containers.Map;
dead_reckoning = false;
consistency_checking = false;
map_scaling_factor = 10;
v = VideoWriter("nuscenes_implementation\videos\fastslam-extended-with_controls-scene"+scene_name+".avi");
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

%% FastSLAM
hold on;
ax = gca;

timestep = 1; % Perform a first update
n_states = 4;

N_particles = 1000;
weight_particles = ones(1,N_particles) * 1/N_particles;
u = get_current_control_input("pose", timestep, ordered_timestamps, ego_canbus_aligned);
x = ego_translation(1,timestep);
vx = u(1) * cos(u(2));
y = ego_translation(2,timestep);
vy = u(1) * sin(u(2));

% Particles generated from normal distribution
mean_state = [x, vx, y, vy]; % x, vx, y, vy
std_state = [2, 1, 2, 1]; % position with 20m 3sig error, and velocity with 5m/s 1sig error. %2, 1, 2, 1
particles_pre = create_gaussian_particles(mean_state, std_state, N_particles); % A array of Particles is created

[Z_, Z_plot] = get_current_measurement(timestep, Z, Z_for_plot);
if dead_reckoning
	particles = particles_pre;
else
    Z_ = rotate_measurements_to_global_frame(Z_, ego_canbus_aligned(string(ordered_timestamps(timestep))).bearing_radians);
	[particles, weight_particles] = update(particles_pre, weight_particles, Z_, ax);
    if effective_no(weight_particles) < N_particles/2
        indexes = systematic_resampling(weight_particles);
        [particles, weight_particles] = resample_from_index(particles, weight_particles, indexes);
    end
end

bep = birdsEyePlot('Parent', ax);
if scene_name == "0061"
    camroll(-180);
else
    camroll(-90);
end
legend('off');
[predicted_pose, estimated_pose, current_ground_truth, clustered_det_plot,unclustered_det_plot] = plot_on_global_map(ax, bep, timestep, particles_pre, particles, weight_particles, ego_translation, ego_orientation, radar, Z_plot, map_scaling_factor);
legend([predicted_pose, estimated_pose, current_ground_truth, ground_truth_track, static_landmarks, moving_landmarks,clustered_det_plot,unclustered_det_plot],...
            ["Predicted position","Estimated position","Actual position","Ground truth track","Static landmarks","Moving landmarks","Clustered detections","Unclustered detections"],...
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
        particles_pre = predict(particles, T, u, can_message);
        if dead_reckoning
            particles = particles_pre;
        else
            Z_ = rotate_measurements_to_global_frame(Z_, ego_canbus_aligned(string(ordered_timestamps(timestep))).bearing_radians);
            [particles, weight_particles] = update(particles_pre, weight_particles, Z_, ax);
            indexes = systematic_resampling(weight_particles);
            [particles, weight_particles] = resample_from_index(particles, weight_particles, indexes);
        end
        [predicted_pose, estimated_pose, current_ground_truth, clustered_det_plot,unclustered_det_plot] = plot_on_global_map(ax, bep, timestep, particles_pre, particles, weight_particles, ego_translation, ego_orientation, radar, Z_plot, map_scaling_factor);
        legend([predicted_pose, estimated_pose, current_ground_truth, ground_truth_track, static_landmarks, moving_landmarks,clustered_det_plot,unclustered_det_plot],...
            ["Predicted position","Estimated position","Actual position","Ground truth track","Static landmarks","Moving landmarks","Clustered detections","Unclustered detections"],...
            'Location','northwest');
        A = getframe(gcf);
        writeVideo(v, A);
        T = 0;
    end
    prev_time = ordered_timestamps(timestep);
end



%% Save filter collected data
% save("nuscenes_implementation\workspaces\kf_slam_filter_collected_data_scene-"+scene_name+".mat", "filter_collected_data");
plot_global_estimated_map(particles, map_scaling_factor);
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
function plot_global_estimated_map(particles, map_scaling_factor)
    global global_point_landmark_map;
    global_point_landmark_map = containers.Map;
    keys = particles(1).local_landmark_map.keys;
    for m = 1:size(keys,2)
        point_map = [0;0];
        for p = 1:size(particles,2)
            land = particles(p).local_landmark_map(string(keys{m}));
            point_map = point_map + land.X;
        end
        point_map = point_map / size(particles,2);
        global_point_landmark_map(string(keys{m})) = point_map;
    end
    
    % Plot all landmarks
    landmark_temp = global_point_landmark_map.values;
    landmark_positions = [];
    for i = 1:length(landmark_temp)
        landmark_positions = [landmark_positions; landmark_temp{i}'];
    end
%     if ~isempty(landmark_positions)
%         plot(landmark_positions(:,1),landmark_positions(:,2),'b+','Tag','landmarks','MarkerSize',5);
%     end
    
    % Plot point estimate of landmarks
    indices = string(global_point_landmark_map.keys);
    values = global_point_landmark_map.values;
    split_indices = split(indices',"_");
    [num_count, ~] = histcounts(categorical(split_indices(:,1)));
    X_plot = containers.Map;
    for i = 1: length(indices)
        temp = split(indices(i),"_");
        temp = temp(1);
        if X_plot.isKey(temp)
            X_plot(temp) = X_plot(temp) + values{i};
        else
            X_plot(temp) = values{i};
        end
    end
    X_plot = cell2mat(X_plot.values) ./ num_count;
   if ~isempty(X_plot)
        plot(X_plot(1,:)*map_scaling_factor, X_plot(2,:)*map_scaling_factor,'bo','Tag','landmarks','MarkerSize',5);
        plot(X_plot(1,:)*map_scaling_factor, X_plot(2,:)*map_scaling_factor,'b+','Tag','landmarks','MarkerSize',5);
    end
end

% Prediction
function [particles] = predict(particles, T, u, can_message)
    global landmark_index_map;
    global n_states;
    std_velocity = 5; 
    std_heading = deg2rad(1);
    
    for p = 1:size(particles,2) % Use Parallel toolbox
        trans1 = randn(1)*std_velocity;
        trans2 = randn(1)*std_velocity;
        rot1 = randn(1)*std_heading;
        rot2 = randn(1)*std_heading;
        particles(p).X(1) = particles(p).X(1) + (T*(u(1) + trans1)*cos(u(2) + rot1));
        particles(p).X(2) = (u(1) + trans1)*cos(u(2) + rot1);
        particles(p).X(3) = particles(p).X(3) + (T*(u(1) + trans2)*sin(u(2) + rot2));
        particles(p).X(4) = (u(1) + trans2)*sin(u(2) + rot2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         trans1 = randn(1)*std_velocity;
%         trans2 = randn(1)*std_velocity;
%         particles(p).X(1) = particles(p).X(1) + (T*(particles(p).X(2) + trans1));
%         particles(p).X(2) = particles(p).X(2) + (trans1);
%         particles(p).X(3) = particles(p).X(3) + (T*(particles(p).X(4) + trans2));
%         particles(p).X(4) = particles(p).X(4) + (trans2);

    end
end

% Update equations
function [particles, weights] = update(particles, weights, Z, ax)
    global n_states;
    global landmark_index_map;
    global extended_object_processing;
    global global_point_landmark_map;
    % For already observed landmarks
    for m = 1:size(Z.id,2)
        if global_point_landmark_map.isKey(string(Z.id(m))) 
            for p = 1:size(particles,2)
                
                % EKF Implementation
                land = particles(p).local_landmark_map(string(Z.id(m)));
                
                x_prime = land.X(1,1) - particles(p).X(1,1);
                y_prime = land.X(2,1) - particles(p).X(3,1);
                
                h = [sqrt(x_prime^2 + y_prime^2); atan2(y_prime, x_prime)];
                R = blkdiag(1,deg2rad(5)^2);

                %EKF scene0655 :0.6,0.01 or 0.5,0.01 ; EKF scene0916: 0.8,0.07 or 0.8,0.05  multi_0916: 0.1, 0.005; one_0655:5,0.001 ; multi_0655: 0.1, 0.005
                %R = blkdiag(1,deg2rad(1)^2);
                measured_range = sqrt(Z.zpos(1,m)^2 + Z.zpos(2,m)^2);
                measured_bearing = atan2(Z.zpos(2,m),Z.zpos(1,m));
                Z_ = [measured_range; measured_bearing];            
            
                % Innovation
                Y = Z_ - h;
                Y(2) = normalize_angle(Y(2));
                H = [x_prime/sqrt(x_prime^2 + y_prime^2), y_prime/sqrt(x_prime^2 + y_prime^2);
                     -y_prime/(x_prime^2 + y_prime^2),    x_prime/(x_prime^2 + y_prime^2)   ];
                
                % Innovation covariance
                P_pre = land.P;
                S = H * P_pre * H' + R;
                % Kalman Gain
                K = P_pre * H' * S^(-1);
                % Update State 
                land.X = land.X + K * Y;
                % Update State Covariance
                land.P = P_pre - K*H*P_pre;
                land.P = (land.P + land.P')/2;

                particles(p).local_landmark_map(string(Z.id(m))) = land;
                
                % Calculate weight of the particle
                weights(p) = weights(p) * exp(-0.5 *Y'*S^(-1)*Y)/sqrt(2*pi*det(S));
            end
            global_point_landmark_map(string(Z.id(m))) = global_point_landmark_map(string(Z.id(m))) + 1;
        end
    end

    weights = weights + 1e-300;
    weights = weights/sum(weights);
    
    [point_mean, ~] = point_estimate(particles, weights);
    % For new landmarks
    for m = 1:size(Z.id,2)
        if ~global_point_landmark_map.isKey(string(Z.id(m))) 
            for p = 1:size(particles,2)
                landmark = struct;
%                 landmark.X = particles(p).X([1,3],1) + Z.zpos(1:2,m);
                landmark.X = point_mean([1,3],1) + Z.zpos(1:2,m);
                landmark.P = blkdiag(5.5,5.5);
                particles(p).local_landmark_map(string(Z.id(m))) = landmark; 
            end
            global_point_landmark_map(string(Z.id(m))) = 1;
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
        
        % Apply RANSAC to filter out moving targets
%         point_cloud = struct;
%         point_cloud.x = 1:length(tmp_struct.radial_velocity);
%         point_cloud.y = tmp_struct.radial_velocity;
%         [~, filter_moving_targets, ~] = ransac2d(point_cloud,length(tmp_struct.radial_velocity)*2,0.1);    
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
        u = [u_vel; u_angle];
    end
end

function [predicted_pose, estimated_pose, current_ground_truth, clustered_det_plot,unclustered_det_plot] = plot_on_global_map(ax, bep, timestep, particles_pre, particles, weight_particles, ego_translation, ego_orientation, radar, Z_plot, map_scaling_factor)
    global extended_object_processing;
    global landmark_index_map;
    
    % Plot the ego's current ground truth position along with coverage
    % area, clustered detections and unclustered detections
    delete(findall(ax,'Tag','ego_current_position'));
    delete(findobj(bep.Plotters,'Tag','coverage_area'));
    delete(findall(ax,'Tag','clustered_detections'));
    delete(findall(ax,'Tag','unclustered_detections'));
    delete(findall(ax,'Tag','landmarks'));
    delete(findall(ax,'Tag','particles_predicted'));
    delete(findall(ax,'Tag','particles'));
    
    % Prepare canvas for plotting
    for i = 1:size(particles_pre,2)
        plot(particles_pre(i).X(1)*map_scaling_factor, particles_pre(i).X(3)*map_scaling_factor,'c.','Tag','particles_predicted');
    end
    for i = 1:size(particles,2)
        plot(particles(i).X(1)*map_scaling_factor, particles(i).X(3)*map_scaling_factor,'k.','Tag','particles');
    end

    [point_mean_pre, point_var_pre] = point_estimate(particles_pre, weight_particles);
    predicted_pose = plot(point_mean_pre(1)*map_scaling_factor, point_mean_pre(3)*map_scaling_factor, "c*", 'Tag', "point_particles_predicted");
    
    [point_mean, point_var] = point_estimate(particles, weight_particles);
    estimated_pose = plot(point_mean(1)*map_scaling_factor, point_mean(3)*map_scaling_factor, "k*", 'Tag', "point_particles");
    
    current_ground_truth = plot(ego_translation(1,timestep) * map_scaling_factor, ego_translation(2,timestep) * map_scaling_factor,'m*','Tag','ego_current_position');

    ego_car_rot_mat = rotmat(quaternion(ego_orientation(:,timestep)'),'point');
    ego_car_trans = [ego_translation(1,timestep), ego_translation(2,timestep)];
    ego_car_yaw = eulerd(quaternion(ego_orientation(:,timestep)'),'ZYX','point'); %yaw pitch roll
    ego_car_yaw = ego_car_yaw(1);
    
    sensor_labels = {'RADAR\_FRONT\_LEFT', 'RADAR\_FRONT', 'RADAR\_FRONT\_RIGHT', 'RADAR\_BACK\_RIGHT', 'RADAR\_BACK\_LEFT'};
    sensor_colors = {'magenta','black','blue','green','cyan'};
    for a = 1:length(radar)
       cap = coverageAreaPlotter(bep,'FaceColor',sensor_colors{a},'EdgeColor','none','Tag','coverage_area','DisplayName', sensor_labels{a});
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
    q_rotmat = [cos(orientation), -sin(orientation), 0;
                sin(orientation),  cos(orientation), 0;
                0               ,  0               , 1];
    %q = quaternion(angle2quat(orientation, 0, 0, 'ZYX'));
    %q_rotmat = rotmat(q,'point');   
    temp = rotate_point_to_global_frame([Z_.zpos(1:2,:);zeros(1,size(Z_.zpos,2))],q_rotmat);
    Z_.zpos(1:2,:) = temp(1:2,:);
end

function point = translate_point_to_global_frame(point, translation_matrix)
    point(1:3,:) = point(1:3,:) + translation_matrix;
end

function point = rotate_point_to_global_frame(point, rotation_matrix)
    point(1:3,:) = rotation_matrix * point(1:3,:);
end

function [particles] = create_gaussian_particles(mean, std, N)
    particles = [];
    for i = 1:N
        x =[mean(1)+(randn(1)*std(1)); 
            mean(2)+(randn(1)*std(2));
            mean(3)+(randn(1)*std(3));
            mean(4)+(randn(1)*std(4))];
        particles = [particles, Particle(x)];
    end
end

% Point estimate
function [point_mean, point_var] = point_estimate(particles, weights)
    point_mean = zeros(4,1);
    point_var = zeros(4,1);
    for i = 1:size(particles,2)
        point_mean = point_mean + (particles(i).X * weights(i));
        point_var  = point_var + ((particles(i).X - point_mean).^2 * weights(i));
    end
    point_mean = point_mean / sum(weights);
    point_var = point_var / sum(weights);
end

% Effective N for resampling condition
function [Nf] = effective_no(weights)
    Nf = 1/sum(weights.^2); 
end

% Resampling from indices
function [particles, weights] = resample_from_index(particles, weights, indexes)
    particles = particles(indexes);
    weights = ones(1,size(weights,2)) * 1/size(weights,2);
end

% Stratified Resampling algorithm
function indices = stratified_resampling(weights)
    n_samples = length(weights);
    positions = ((0:(n_samples-1)) + rand(1,n_samples))/(n_samples);
    indices = zeros(1,n_samples);
    cum_sum = cumsum(weights);
    i = 0;
    j = 0;
    while i < n_samples
       if positions(i+1) < cum_sum(j+1)
           indices(i+1) = j;
           i = i+1;
       else
           j = j+1;
       end
    end
    indices = indices + 1;
end

% Systematic resampling algorithm
function indices = systematic_resampling(weights)
    n_samples = length(weights);
    positions = ((0:n_samples-1) + rand(1))/(n_samples);
    indices = zeros(1,n_samples);
    cum_sum = cumsum(weights);
    i = 0;
    j = 0;
    while i < n_samples
       if positions(i+1) < cum_sum(j+1)
           indices(i+1) = j;
           i = i+1;
       else
           j = j+1;
       end
    end
    indices = indices + 1;
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