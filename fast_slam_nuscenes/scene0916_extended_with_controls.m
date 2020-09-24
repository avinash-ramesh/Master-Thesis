%% SLAM - x, vx, y, vy
clc;
close all;
clear all;
random_num_gen = rng(100); % Initialize random number generator
scene_name = ["0655","0061","0916"];
scene_name = scene_name(3);
dead_reckoning = false;
extended_object_processing = true;
consistency_checks = true;
filter_consistency = [];
distance_based_filtering = false;
global n_states;
global landmark_index_map;
landmark_index_map = containers.Map;
global global_point_landmark_map;
global_point_landmark_map = containers.Map;

load("nuscenes_implementation\workspaces\ground_truth-scene"+scene_name+".mat","ground_truth_ego_pos");
load("nuscenes_implementation\workspaces\control_input_1-scene"+scene_name+".mat","control_input");
load("nuscenes_implementation\workspaces\timestamp-scene"+scene_name+".mat","timestamp");

if ~extended_object_processing
    % For one detection per target - Point object processing
    load("nuscenes_implementation\workspaces\global_landmarks_map-scene"+scene_name+".mat","global_landmarks_map");
    if distance_based_filtering
        load("nuscenes_implementation\workspaces\filtered_measurements-10m-scene"+scene_name+".mat","measurements");
        load("nuscenes_implementation\workspaces\filtered_measurements_global-10m-scene"+scene_name+".mat","measurements_global");
    else
        load("nuscenes_implementation\workspaces\measurements-scene"+scene_name+".mat","measurements");
        load("nuscenes_implementation\workspaces\measurements_global-scene"+scene_name+".mat","measurements_global");
    end
else
    % For multiple detections per target - Extended object processing
    load("nuscenes_implementation\workspaces\global_landmarks_map-multi_box_scene"+scene_name+".mat","global_landmarks_map");    
    if distance_based_filtering
        load("nuscenes_implementation\workspaces\filtered_measurements-multi_box-10m-scene"+scene_name+".mat","measurements");
        load("nuscenes_implementation\workspaces\filtered_measurements_multi_box-global-10m-scene"+scene_name+".mat","measurements_global");
    else
        load("nuscenes_implementation\workspaces\measurements-multi_box-scene"+scene_name+".mat","measurements");
        load("nuscenes_implementation\workspaces\measurements_multi_box-global-scene"+scene_name+".mat","measurements_global");
    end
end

ax = gca;
set(gcf, 'Position', get(0, 'Screensize'));
hold on;
plot_scenario(ground_truth_ego_pos, global_landmarks_map); % Both these variables come from loading the workspace

% [measurements, measurements_global] = filter_measurements_one_per_box_at_one_timestamp(measurements, measurements_global);
% [measurements, measurements_global] = filter_measurements_one_per_box(measurements, measurements_global);
% [measurements, measurements_global] = filter_measurements_distance_based(measurements, measurements_global, 5); % Works best with one per box
% [measurements, measurements_global] = filter_measurements_distance_based(measurements, measurements_global);

%% Filter Initialization
timestep = 1;
n_states = 4;

N_particles = 1000;
weight_particles = ones(1,N_particles) * 1/N_particles;

x = ground_truth_ego_pos(1,timestep);
vx = control_input(1,timestep) * cos(control_input(2,timestep));
y = ground_truth_ego_pos(2,timestep);
vy = control_input(1,timestep) * sin(control_input(2,timestep));

% % Uniformly generate particles - may lead to filter degeneracy due to
% % sample impoverishment as a result of random spread of the particles
% x_min = [x-20, vx-5, y-20, vy-5]; % x, vx, y, vy
% x_max = [x+20, vx+5, y+20, vy+5];
% particles = create_uniform_particles(x_min, x_max, N);

% Particles generated from normal distribution
mean_state = [x, vx, y, vy]; % x, vx, y, vy
std_state = [2, 1, 2, 1]; % position with 20m 3sig error, and velocity with 5m/s 1sig error. %2, 1, 2, 1
particles_pre = create_gaussian_particles(mean_state, std_state, N_particles); % A array of Particles is created

% First Update
if dead_reckoning
    particles = particles_pre;
else
	[Z_, Z_plot] = get_current_measurement(timestep, measurements, measurements_global);
    if isfield(Z_, "id") && size(Z_.id,2) ~= 0
        % Either convert measurements from local frame to global frame or convert landmarks in state vector from global to local frame
        Z_ = rotate_measurements_to_global_frame(Z_, control_input(2,timestep));
        [particles, weight_particles] = update(particles_pre, weight_particles, Z_);
        if effective_no(weight_particles) < N_particles/2
            indexes = systematic_resampling(weight_particles);
            [particles, weight_particles] = resample_from_index(particles, weight_particles, indexes);
        end
    end
    % After resampling from indices, check if the weights should be assigned equally or not.
end

[filter_consistency(timestep).ground_truth, filter_consistency(timestep).X_pre, filter_consistency(timestep).X] = plot_on_global_map(timestep, particles_pre, particles, weight_particles, ground_truth_ego_pos, measurements_global, ax);
% pause(1);

prev_time = timestamp(timestep);
T = 0;

%% Predict and Update steps
for timestep = 2 : size(timestamp,2)
    T = T + (timestamp(timestep) - prev_time) * 1e-6;
    [Z_, Z_plot] = get_current_measurement(timestep, measurements, measurements_global);
    
    if isfield(Z_, "id") && size(Z_.id,2) ~= 0 % Perform filtering only when measurements in the Z variable are present
        velocity = control_input(1,timestep);
        heading = control_input(2,timestep);
        particles_pre = predict(particles, T, velocity, heading);
        if dead_reckoning
            particles = particles_pre;
        else
            % Either convert measurements from local frame to global frame or convert landmarks in state vector from global to local frame
            Z_ = rotate_measurements_to_global_frame(Z_, control_input(2,timestep));
            [particles, weight_particles] = update(particles_pre, weight_particles, Z_);
            indexes = systematic_resampling(weight_particles);
            [particles, weight_particles] = resample_from_index(particles, weight_particles, indexes);
        end
        [filter_consistency(timestep).ground_truth, filter_consistency(timestep).X_pre, filter_consistency(timestep).X] = plot_on_global_map(timestep, particles_pre, particles, weight_particles, ground_truth_ego_pos, measurements_global, ax);
        T = 0;
    end
    prev_time = timestamp(timestep);
end

%% Plot the final map
plot_global_estimated_map(particles);
hold off;

if consistency_checks
    %save("nuscenes_implementation\workspaces\kf_slam_consistency1.mat","filter_consistency");
    run_consistency_checks(filter_consistency);
end

%% Helper functions
function run_consistency_checks(filter_consistency)
    displacement_error_k = [];
    states_k = [];
    ground_truth = [];
    for i = 1:length(filter_consistency)
        states_k = [states_k, filter_consistency(i).X([1,3],1)];
        ground_truth = [ground_truth, filter_consistency(i).ground_truth(:,1)];
        displacement_error_k = [displacement_error_k, sqrt(sum((filter_consistency(i).X([1,3],1) - filter_consistency(i).ground_truth(:,1)).^2))];
    end

    % Check 1: Displacement (Eucledian distance) Error (minADE_k) - The pointwise L2 distances between the estimated trajectory and ground truth.  
    mean_error = mean(displacement_error_k);
    fprintf("Mean displacement error is: %f\n", mean_error);

    % Check 2: Final Displacement Error (minFDE_k) - The L2 distance between the final points of the estimation and ground truth.
    fprintf("Final displacement error is: %f\n", displacement_error_k(end));
    
    % Check 3: Miss rate at 1 meters - If pointwise L2 distance between the
    % estimation and ground truth is greater than 1 meters, we define it as a miss.
    fprintf("Miss rate at 1 meters (in percentage): %f\n", sum(displacement_error_k > 1) * 100 / length(displacement_error_k));
    
    % Check 4: Visual inspection of estimation error, it should be normal
    estimated_error = mean(states_k([1,2],:)-ground_truth, 2);
    fprintf("Mean estimated error: [x = %s, y = %s]\n", estimated_error(1), estimated_error(2));
    figure;
    t = tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
    set(gcf, 'Position', get(0, 'Screensize'));
    hold on;
    ax = gca;
    ax.XLim = [min(states_k(1,:))-20, max(states_k(1,:))+20];
    ax.YLim = [min(states_k(2,:))-20, max(states_k(2,:))+20];
    text_to_display = {['Mean displacement error is: ', char(num2str(mean_error))],...
                       ['Final displacement error is: ', char(num2str(displacement_error_k(end)))],...
                       ['Miss rate at 1 meters: ', char(num2str(sum(displacement_error_k > 1) * 100 / length(displacement_error_k))), '%'],...
                       ['Mean estimated error: [x = ', char(num2str(estimated_error(1))),', y = ', char(num2str(estimated_error(2))),']']};
    annotation('textbox',[.6 .25 .1 .1],'String',text_to_display,'FitBoxToText','on', 'HorizontalAlignment','left','BackgroundColor', [150/255,255/255,150/255], 'Tag','text_to_display');

    ax = nexttile;
    hold on;
    ax.XLim = [1, size(ground_truth,2)];
    error = states_k([1,2],:)-ground_truth;
    error_plot = plot(error(1,:),'r-', 'LineWidth', 2);
    plot(zeros(1,size(error(1,:),2)),'b-');
    legend(error_plot,"Estimation error",'Location','northeast', 'interpreter','latex');
    title('First Order Error in Position X', 'Interpreter', 'latex');
    xlabel('Timesteps $\longrightarrow$', 'Interpreter', 'latex');
    ylabel('Position (meters) $\longrightarrow$', 'Interpreter', 'latex');
    hold off;
    
    ax = nexttile;
    hold on;
    ax.XLim = [1, size(ground_truth,2)];
    error = states_k([1,2],:)-ground_truth;
    error_plot = plot(error(2,:),'r-','LineWidth', 2);
    plot(zeros(1,size(error(1,:),2)),'b-');
    legend(error_plot,"Estimation error", 'Location','northeast', 'interpreter','latex');
    title('First Order Error in Position Y', 'Interpreter', 'latex');
    xlabel('Timesteps $\longrightarrow$', 'Interpreter', 'latex');
    ylabel('Position (meters) $\longrightarrow$', 'Interpreter', 'latex');
    hold off;
end

function plot_scenario(ground_truth_ego_pos, global_landmarks_map)
    % Plot ground truth trajectory of the ego car
    plot(ground_truth_ego_pos(1,:), ground_truth_ego_pos(2,:), 'b-o', 'MarkerFaceColor', 'y');

    % Plot landmarks in global coordinates
    global_landmarks_map_keys = global_landmarks_map.keys;
    for i = 1: length(global_landmarks_map.keys)
        land = global_landmarks_map(global_landmarks_map_keys{i});
        plot(land(1), land(2), "b*");
        % text(land(1), land(2), global_landmarks_map_keys{i}, "Color", "b");
    end
end

function plot_global_estimated_map(particles)
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
    if ~isempty(landmark_positions)
        plot(landmark_positions(:,1),landmark_positions(:,2),'g+','Tag','landmarks','MarkerSize',5);
    end
    
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
        plot(X_plot(1,:), X_plot(2,:),'go','Tag','landmarks','MarkerSize',5);
    end
end

function [ground_truth, point_mean_pre, point_mean] = plot_on_global_map(timestep, particles_pre, particles, weights, ground_truth_ego_pos, measurements_global, ax)
    legend off;
    delete(findall(ax, "Tag","current_ground_truth_position"));
% 	delete(findall(ax, "Tag","mean_cluster"));
% 	delete(findall(ax, "Tag","unclustered"));
% 	delete(findall(ax, "Tag","cluster_id"));
    delete(findall(ax, "Tag","landmarks"));
%     delete(findall(ax,'Tag','particles_predicted'));
%     delete(findall(ax,'Tag','particles'));
    
    % Prepare canvas for plotting
%     for i = 1:size(particles_pre,2)
%         plot(particles_pre(i).X(1), particles_pre(i).X(3),'c.','Tag','particles_predicted');
%     end
%     for i = 1:size(particles,2)
%         plot(particles(i).X(1), particles(i).X(3),'k.','Tag','particles');
%     end

    [point_mean_pre, point_var_pre] = point_estimate(particles_pre, weights);
    plot(point_mean_pre(1), point_mean_pre(3), "c*", 'Tag', "point_particles_predicted");
    
    [point_mean, point_var] = point_estimate(particles, weights);
    plot(point_mean(1), point_mean(3), "k*", 'Tag', "point_particles");
    
	plot(ground_truth_ego_pos(1,timestep), ground_truth_ego_pos(2,timestep), "m*", "Tag","current_ground_truth_position");
    ground_truth = ground_truth_ego_pos(:,timestep);
% 	scatter(measurements_global{timestep}.zpos(1,:), measurements_global{timestep}.zpos(2,:), 'Marker','o','MarkerFaceColor' ,'g', 'MarkerEdgeColor','g', "Tag", "mean_cluster");
% 	scatter(measurements_global{timestep}.detections(1,:), measurements_global{timestep}.detections(2,:), 'Marker','o', 'MarkerEdgeColor','r', "Tag", "unclustered");
	% text(measurements_global{timestep}.zpos(1,:), measurements_global{timestep}.zpos(2,:), measurements_global{timestep}.id', "Tag", "cluster_id", "Color", "g");
    drawnow;
end

function [Z_, Z_plot] = get_current_measurement(timestep, measurements, measurements_global)
    Z_ = struct;
    Z_plot = struct;
    
    if (isfield(measurements{timestep}, "id"))
        Z_.id = measurements{timestep}.id';
        Z_.zpos = measurements{timestep}.zpos;
        Z_.detections = measurements{timestep}.detections;
        Z_plot.id = measurements_global{timestep}.id';
        Z_plot.zpos = measurements_global{timestep}.zpos;
        Z_plot.detections = measurements_global{timestep}.detections;
    end
end

% Predict equations
function [particles] = predict(particles, T, velocity, heading)
    %KF both scenes - 1; EKF
    std_velocity = 5; 
    std_heading = deg2rad(1); 

%     std_velocity = 1; 
%     std_heading = deg2rad(1);  

    %KF both scenes - 0.08; EKF
    for p = 1:size(particles,2) % Use Parallel toolbox
        trans1 = randn(1)*std_velocity;
        trans2 = randn(1)*std_velocity;
        rot1 = randn(1)*std_heading;
        rot2 = randn(1)*std_heading;
        particles(p).X(1) = particles(p).X(1) + (T*(velocity + trans1)*cos(heading + rot1));
        particles(p).X(2) = (velocity + trans1)*cos(heading + rot1);
        particles(p).X(3) = particles(p).X(3) + (T*(velocity + trans2)*sin(heading + rot2));
        particles(p).X(4) = (velocity + trans2)*sin(heading + rot2);

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
function [particles, weights] = update(particles, weights, Z) 
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
                
                
                
                % KF implementation
                % H = [1, 0;
                     % 0, 1];
                % R = blkdiag(0.15,0.15); %Scene 0916 - 0.1, 0.2, 0.15 ; 
                % land = particles(p).local_landmark_map(string(Z.id(m)));
                % X_pre = land.X - particles(p).X([1,3],1);

                % % Innovation
                % Z_ = Z.zpos(1:2,m);
                % Y = Z_(:) - X_pre;
                % % Innovation covariance
                % P_pre = land.P;
                % S = H * P_pre * H' + R;
                % % Kalman Gain
                % K = P_pre * H' * S^(-1);
                % % Update State 
                % land.X = land.X + K * Y;
                % % Update State Covariance
                % land.P = P_pre - K*H*P_pre;
                % land.P = (land.P + land.P')/2;

                % particles(p).local_landmark_map(string(Z.id(m))) = land;

                % % Calculate weight of the particle
                % weights(p) = weights(p) * exp(-0.5 *Y'*S^(-1)*Y)/sqrt(2*pi*det(S));
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

function [particles] = create_uniform_particles(x_min, x_max, N)
    particles = [];
    for i = 1:N
        x =[(x_max(1)-x_min(1)).*rand(1)+x_min(1); 
            (x_max(2)-x_min(2)).*rand(1)+x_min(2);
            (x_max(3)-x_min(3)).*rand(1)+x_min(3);
            (x_max(4)-x_min(4)).*rand(1)+x_min(4)];
        particles = [particles, Particle(x)];
    end
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

function [measurements_new, measurements_global_new] = filter_measurements_one_per_box_at_one_timestamp(measurements, measurements_global)
    measurements_new = {};
    measurements_global_new = {};
    for i = 1: size(measurements,1)
        temp_map = containers.Map;
        tmp = struct;
        tmp_global = struct;
        valid = 1;
        for j = 1: size(measurements{i}.id,1)
            s = split(measurements{i}.id(j),"_");
            if ~temp_map.isKey(s(1))
                temp_map(s(1)) = 1;
                tmp.id(valid,1) = measurements{i}.id(j,1);
                if isfield(measurements{i},"parent_id")
                    tmp.parent_id(valid,1) = measurements{i}.parent_id(j,1);
                end
                tmp.zpos(:, valid) = measurements{i}.zpos(:,j);
                tmp.detections = measurements{i}.detections;
                
                tmp_global.id(valid,1) = measurements_global{i}.id(j,1);
                if isfield(measurements_global{i},"parent_id")
                    tmp_global.parent_id(valid,1) = measurements_global{i}.parent_id(j,1);
                end
                tmp_global.zpos(:, valid) = measurements_global{i}.zpos(:,j);
                tmp_global.detections = measurements_global{i}.detections;
                
                valid = valid + 1;
            end
        end
        measurements_new = [measurements_new; tmp];
        measurements_global_new = [measurements_global_new; tmp_global];
    end
end

function [measurements_new, measurements_global_new] = filter_measurements_one_per_box(measurements, measurements_global)
    temp_map = containers.Map;
    measurements_new = {};
    measurements_global_new = {};
    for i = 1: size(measurements,1)
        tmp = struct;
        tmp_global = struct;
        valid = 1;
        for j = 1: size(measurements{i}.id,1)
            s = split(measurements{i}.id(j),"_");
            if ~temp_map.isKey(s(1))
                temp_map(s(1)) = 1;
                tmp.id(valid,1) = measurements{i}.id(j,1);
                if isfield(measurements{i},"parent_id")
                    tmp.parent_id(valid,1) = measurements{i}.parent_id(j,1);
                end
                tmp.zpos(:, valid) = measurements{i}.zpos(:,j);
                tmp.detections = measurements{i}.detections;
                
                tmp_global.id(valid,1) = measurements_global{i}.id(j,1);
                if isfield(measurements_global{i},"parent_id")
                    tmp_global.parent_id(valid,1) = measurements_global{i}.parent_id(j,1);
                end
                tmp_global.zpos(:, valid) = measurements_global{i}.zpos(:,j);
                tmp_global.detections = measurements_global{i}.detections;
                
                valid = valid + 1;
            end
        end
        measurements_new = [measurements_new; tmp];
        measurements_global_new = [measurements_global_new; tmp_global];
    end
end

function [measurements_new, measurements_global_new] = filter_measurements_distance_based(measurements, measurements_global, distance_tolerance)    
    measurements_new = {};
    measurements_global_new = {};
    for i = 1: size(measurements,1)
        temp_map = containers.Map;
        tmp = struct;
        tmp_global = struct;
        valid = 1;
        distances = pdist(measurements{i}.zpos(1:2,:)');
        distances_mat = squareform(distances);
        [~,q] = find(distances_mat<=distance_tolerance & distances_mat>0);
        q = unique(q);
        possible_scenario = struct;
        possible_scenario.choice = {};
        possible_scenario.distance = [];
        
        if ~isempty(q)
            for k = 1: size(q,1)
                if ~isempty(possible_scenario.choice) && (k>size(possible_scenario.choice{1},2))
                    break;
                end
                C = nchoosek(q, k);
               
                for m = 1:size(C,1)
                  distances_mat_temp = distances_mat;
                  distances_mat_temp([C(m,:)],:) = [];
                  distances_mat_temp(:,[C(m,:)]) = [];
                  [~,n] = find(distances_mat_temp<=distance_tolerance & distances_mat_temp>0);
                  if isempty(n) && ~isempty(distances_mat_temp)
                      possible_scenario.choice = [possible_scenario.choice, {C(m,:)}];
                      possible_scenario.distance = [possible_scenario.distance, sum(distances_mat_temp, 'all') * size(distances_mat_temp,1)];
                  end
                end
            end
        end

        if ~isempty(possible_scenario.distance)
            [~,choice] = max(possible_scenario.distance);
            measurements{i}.id([possible_scenario.choice{choice}]) = [];
            measurements_global{i}.id([possible_scenario.choice{choice}]) = [];
            if isfield(measurements{i},"parent_id")
                measurements{i}.parent_id([possible_scenario.choice{choice}]) = [];
                measurements_global{i}.parent_id([possible_scenario.choice{choice}]) = [];
            end
            measurements{i}.zpos(:,[possible_scenario.choice{choice}]) = [];
            measurements_global{i}.zpos(:,[possible_scenario.choice{choice}]) = [];
        end
        measurements_new = [measurements_new; measurements{i}];
        measurements_global_new = [measurements_global_new; measurements_global{i}];
    end
end


%% Backup -5
% %% SLAM - x, y, theta
% clc;
% close all;
% clear all;
% rand_num_gen = rng(100); % Initialize random number generator
% scene_name = ["0655","0916"];
% scene_name = scene_name(2);
% dead_reckoning = false;
% extended_object_processing = true;
% global n_states;
% global landmark_index_map;
% landmark_index_map = containers.Map;
% global global_point_landmark_map;
% global_point_landmark_map = containers.Map;
% 
% load("nuscenes_implementation\workspaces\ground_truth-scene"+scene_name+".mat","ground_truth_ego_pos");
% load("nuscenes_implementation\workspaces\control_input_1-scene"+scene_name+".mat","control_input");
% load("nuscenes_implementation\workspaces\timestamp-scene"+scene_name+".mat","timestamp");
% 
% if ~extended_object_processing
%     % For one detection per target - Point object processing
%     load("nuscenes_implementation\workspaces\global_landmarks_map-scene"+scene_name+".mat","global_landmarks_map");
%     load("nuscenes_implementation\workspaces\measurements-scene"+scene_name+".mat","measurements");
%     load("nuscenes_implementation\workspaces\measurements_global-scene"+scene_name+".mat","measurements_global");
% else
%     % For multiple detections per target - Extended object processing
%     load("nuscenes_implementation\workspaces\global_landmarks_map-multi_box_scene"+scene_name+".mat","global_landmarks_map");
%     load("nuscenes_implementation\workspaces\measurements-multi_box-scene"+scene_name+".mat","measurements");
%     load("nuscenes_implementation\workspaces\measurements_multi_box-global-scene"+scene_name+".mat","measurements_global");
% end
% 
% ax = gca;
% set(gcf, 'Position', get(0, 'Screensize'));
% hold on;
% plot_scenario(ground_truth_ego_pos, global_landmarks_map); % Both these variables come from loading the workspace
% 
% %% Filter Initialization
% timestep = 1;
% n_states = 3;
% 
% N_particles = 1000;
% weight_particles = ones(1,N_particles) * 1/N_particles;
% 
% x = ground_truth_ego_pos(1,timestep);
% y = ground_truth_ego_pos(2,timestep);
% theta = control_input(2,timestep);
% 
% % % Uniformly generate particles - may lead to filter degeneracy due to
% % % sample impoverishment as a result of random spread of the particles
% % x_min = [x-20, vx-5, y-20, vy-5]; % x, vx, y, vy
% % x_max = [x+20, vx+5, y+20, vy+5];
% % particles = create_uniform_particles(x_min, x_max, N);
% 
% % Particles generated from normal distribution
% mean_state = [x, y, theta]; % x, y, v, theta
% std_state = [7, 7, deg2rad(5)]; % position with 20m 3sig error, and velocity with 5m/s 1sig error.
% particles_pre = create_gaussian_particles(mean_state, std_state, N_particles); % A array of Particles is created
% 
% % First Update
% if dead_reckoning
%     particles = particles_pre;
% else
% 	[Z_, Z_plot] = get_current_measurement(timestep, measurements, measurements_global);
%     % Either convert measurements from local frame to global frame or convert landmarks in state vector from global to local frame
% %     Z_ = rotate_measurements_to_global_frame(Z_, control_input(2,timestep));
% 	[particles, weight_particles] = update(particles_pre, weight_particles, Z_, control_input(2,timestep));
%     if effective_no(weight_particles) < N_particles/2
%         indexes = systematic_resampling(weight_particles);
%         [particles, weight_particles] = resample_from_index(particles, weight_particles, indexes);
%     end
%     % After resampling from indices, check if the weights should be
%     % assigned equally or not.
% end
% 
% plot_on_global_map(timestep, particles_pre, particles, weight_particles, ground_truth_ego_pos, measurements_global, ax);
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
%         velocity = control_input(1,timestep);
%         heading = control_input(2,timestep);
%         particles_pre = predict(particles, T, velocity, heading);
%         if dead_reckoning
%             particles = particles_pre;
%         else
%             % Either convert measurements from local frame to global frame or convert landmarks in state vector from global to local frame
% %             Z_ = rotate_measurements_to_global_frame(Z_, control_input(2,timestep));
%             [particles, weight_particles] = update(particles_pre, weight_particles, Z_, control_input(2,timestep));
% %             if effective_no(weight_particles) < N_particles/2
%                 indexes = systematic_resampling(weight_particles);
%                 [particles, weight_particles] = resample_from_index(particles, weight_particles, indexes);
% %             end
%         end
%         plot_on_global_map(timestep, particles_pre, particles, weight_particles, ground_truth_ego_pos, measurements_global, ax);
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
%     plot(ground_truth_ego_pos(1,:), ground_truth_ego_pos(2,:), 'b-o', 'MarkerFaceColor', 'y');
% 
%     % Plot landmarks in global coordinates
%     global_landmarks_map_keys = global_landmarks_map.keys;
%     for i = 1: length(global_landmarks_map.keys)
%         land = global_landmarks_map(global_landmarks_map_keys{i});
%         plot(land(1), land(2), "b*");
% %         text(land(1), land(2), global_landmarks_map_keys{i}, "Color", "b");
%     end
% end
% 
% function plot_on_global_map(timestep, particles_pre, particles, weights, ground_truth_ego_pos, measurements_global, ax)
%     global global_point_landmark_map;
%     legend off;
%     delete(findall(ax, "Tag","current_ground_truth_position"));
% 	delete(findall(ax, "Tag","mean_cluster"));
% 	delete(findall(ax, "Tag","unclustered"));
% 	delete(findall(ax, "Tag","cluster_id"));
%     delete(findall(ax, "Tag","landmarks"));
%     delete(findall(ax,'Tag','particles_predicted'));
%     delete(findall(ax,'Tag','particles'));
%     
%     % Prepare canvas for plotting
%     for i = 1:size(particles_pre,2)
%         plot(particles_pre(i).X(1), particles_pre(i).X(2),'c.','Tag','particles_predicted');
%     end
%     for i = 1:size(particles,2)
%         plot(particles(i).X(1), particles(i).X(2),'k.','Tag','particles');
%     end
% 
%     [point_mean, point_var] = point_estimate(particles_pre, weights);
%     plot(point_mean(1), point_mean(2), "c*", 'Tag', "point_particles_predicted");
%     
%     [point_mean, point_var] = point_estimate(particles, weights);
%     plot(point_mean(1), point_mean(2), "k*", 'Tag', "point_particles");
%     
% %     landmark_temp = global_point_landmark_map.values;
% %     landmark_positions = [];
% %     for i = 1:length(landmark_temp)
% %         landmark_positions = [landmark_positions; landmark_temp{i}.X'];
% %     end
% %     if ~isempty(landmark_positions)
% %         plot(landmark_positions(:,1),landmark_positions(:,2),'g+','Tag','landmarks','MarkerSize',5);
% %     end
%     
% 	plot(ground_truth_ego_pos(1,timestep), ground_truth_ego_pos(2,timestep), "m*", "Tag","current_ground_truth_position");
% 	scatter(measurements_global{timestep}.zpos(1,:), measurements_global{timestep}.zpos(2,:), 'Marker','o','MarkerFaceColor' ,'g', 'MarkerEdgeColor','g', "Tag", "mean_cluster");
% 	scatter(measurements_global{timestep}.detections(1,:), measurements_global{timestep}.detections(2,:), 'Marker','o', 'MarkerEdgeColor','r', "Tag", "unclustered");
% % 	text(measurements_global{timestep}.zpos(1,:), measurements_global{timestep}.zpos(2,:), measurements_global{timestep}.id', "Tag", "cluster_id", "Color", "g");
%     drawnow;
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
% % Predict equations
% function [particles] = predict(particles, T, velocity, heading)
%     std_velocity = 5;
%     std_heading = deg2rad(45);
%     for p = 1:size(particles,2) % Parallel toolbox
%         random_velocity = randn(1);
%         random_heading = randn(1);
%         particles(p).X(1) = particles(p).X(1) + (T*(velocity + (random_velocity*std_velocity))*cos(heading + (random_heading*std_heading)));
%         particles(p).X(2) = particles(p).X(2) + (T*(velocity + (random_velocity*std_velocity))*sin(heading + (random_heading*std_heading)));
%         particles(p).X(3) = heading + (random_heading*std_heading);
%     end
% end
% 
% % Update equations
% function [particles, weights] = update(particles, weights, Z, control_heading) 
%     global global_point_landmark_map;
%     % For already observed landmarks
%     for m = 1:size(Z.id,2)
%         if global_point_landmark_map.isKey(string(Z.id(m))) 
%             for p = 1:size(particles,2)
%                 H = [1, 0;
%                      0, 1];
%                 R = blkdiag(5,5);
%                 land = particles(p).local_landmark_map(string(Z.id(m)));
%                 rotation_matrix = [cos(particles(p).X(3)), -sin(particles(p).X(3)), particles(p).X(1);
%                                    sin(particles(p).X(3)), cos(particles(p).X(3)) , particles(p).X(2);
%                                    0                     , 0                      , 1                ];
%                 X_pre = rotation_matrix\[land.X(:,1); 1]; % inv(rotation_matrix) * global_land;
%                 
%                 % Innovation
%                 Z_ = Z.zpos(1:2,m);
%                 Y = Z_(:) - X_pre(1:2,1);
%                 % Innovation covariance
%                 P_pre = land.P;
%                 S = H * P_pre * H' + R;
%                 % Kalman Gain
%                 K = P_pre * H' * S^(-1);
%                 % Update State 
%                 land.X = land.X + K * Y;
%                 % Update State Covariance
%                 land.P = P_pre - K*H*P_pre;
%                 land.P = (land.P + land.P')/2;
% 
%                 particles(p).local_landmark_map(string(Z.id(m))) = land;
%                 
%                 % Calculate weight of the particle
%                 weights(p) = weights(p) * exp(-0.5 *Y'*S^(-1)*Y)/sqrt(2*pi*det(S));
%             end
%             global_point_landmark_map(string(Z.id(m))) = global_point_landmark_map(string(Z.id(m))) + 1;
%         end
%     end
% 
%     % For new landmarks
%     for m = 1:size(Z.id,2)
%         if ~global_point_landmark_map.isKey(string(Z.id(m))) 
%             for p = 1:size(particles,2)
%                 landmark = struct;
%                 rotation_matrix = [cos(particles(p).X(3)), -sin(particles(p).X(3)), particles(p).X(1);
%                                    sin(particles(p).X(3)), cos(particles(p).X(3)) , particles(p).X(2);
%                                    0                     , 0                      , 1                ];
%                 landmark.X = rotation_matrix * [Z.zpos(1:2,m); 1];
%                 landmark.X = landmark.X(1:2,1);
%                 landmark.P = blkdiag(1,1);
%                 particles(p).local_landmark_map(string(Z.id(m))) = landmark; 
%             end
%             global_point_landmark_map(string(Z.id(m))) = 1;
%         end
%     end
%     
%     weights = weights + 1e-300;
%     weights = weights/sum(weights);
% end
% 
% function Z_ = rotate_measurements_to_global_frame(Z_,orientation)
%     q = quaternion(angle2quat(orientation, 0, 0, 'ZYX'));
%     q_rotmat = rotmat(q,'point');   
%     temp = rotate_point_to_global_frame([Z_.zpos(1:2,:);zeros(1,size(Z_.zpos,2))],q_rotmat);
%     Z_.zpos(1:2,:) = temp(1:2,:);
% end
%             
% function point = translate_point_to_global_frame(point, translation_matrix)
%     point(1:3,:) = point(1:3,:) + translation_matrix;
% end
% 
% function point = rotate_point_to_global_frame(point, rotation_matrix)
%     point(1:3,:) = rotation_matrix * point(1:3,:);
% end
% 
% function [particles] = create_uniform_particles(x_min, x_max, N)
%     particles = [];
%     for i = 1:N
%         x =[(x_max(1)-x_min(1)).*rand(1)+x_min(1); 
%             (x_max(2)-x_min(2)).*rand(1)+x_min(2);
%             (x_max(3)-x_min(3)).*rand(1)+x_min(3)];
%         particles = [particles, Particle(x)];
%     end
% end
% 
% function [particles] = create_gaussian_particles(mean, std, N)
%     particles = [];
%     for i = 1:N
%         x =[mean(1)+(randn(1)*std(1)); 
%             mean(2)+(randn(1)*std(2));
%             mean(3)+(randn(1)*std(3))];
%         particles = [particles, Particle(x)];
%     end
% end
% 
% % Effective N for resampling condition
% function [Nf] = effective_no(weights)
%     Nf = 1/sum(weights.^2); 
% end
% 
% % Resampling from indices
% function [particles, weights] = resample_from_index(particles, weights, indexes)
%     particles = particles(indexes);
%     weights = ones(1,size(weights,2)) * 1/size(weights,2);
% end
% 
% % Stratified Resampling algorithm
% function indices = stratified_resampling(weights)
%     n_samples = length(weights);
%     positions = ((0:(n_samples-1)) + rand(1,n_samples))/(n_samples);
%     indices = zeros(1,n_samples);
%     cum_sum = cumsum(weights);
%     i = 0;
%     j = 0;
%     while i < n_samples
%        if positions(i+1) < cum_sum(j+1)
%            indices(i+1) = j;
%            i = i+1;
%        else
%            j = j+1;
%        end
%     end
%     indices = indices + 1;
% end
% 
% % Systematic resampling algorithm
% function indices = systematic_resampling(weights)
%     n_samples = length(weights);
%     positions = ((0:n_samples-1) + rand(1))/(n_samples);
%     indices = zeros(1,n_samples);
%     cum_sum = cumsum(weights);
%     i = 0;
%     j = 0;
%     while i < n_samples
%        if positions(i+1) < cum_sum(j+1)
%            indices(i+1) = j;
%            i = i+1;
%        else
%            j = j+1;
%        end
%     end
%     indices = indices + 1;
% end
% 
% % Point estimate
% function [point_mean, point_var] = point_estimate(particles, weights)
%     point_mean = zeros(3,1);
%     point_var = zeros(3,1);
%     for i = 1:size(particles,2)
%         point_mean = point_mean + (particles(i).X * weights(i));
%         point_var  = point_var + ((particles(i).X - point_mean).^2 * weights(i));
%     end
%     point_mean = point_mean / sum(weights);
%     point_var = point_var / sum(weights);
% end




%% Backup -4
% % SLAM
% clc;
% close all;
% clear all;
% rand_num_gen = rng(100); % Initialize random number generator
% scene_name = ["0655","0916"];
% scene_name = scene_name(2);
% dead_reckoning = false;
% extended_object_processing = false;
% global n_states;
% global landmark_index_map;
% landmark_index_map = containers.Map;
% global global_point_landmark_map;
% global_point_landmark_map = containers.Map;
% 
% load("nuscenes_implementation\workspaces\ground_truth-scene"+scene_name+".mat","ground_truth_ego_pos");
% load("nuscenes_implementation\workspaces\control_input_1-scene"+scene_name+".mat","control_input");
% load("nuscenes_implementation\workspaces\timestamp-scene"+scene_name+".mat","timestamp");
% 
% if ~extended_object_processing
%     % For one detection per target - Point object processing
%     load("nuscenes_implementation\workspaces\global_landmarks_map-scene"+scene_name+".mat","global_landmarks_map");
%     load("nuscenes_implementation\workspaces\measurements-scene"+scene_name+".mat","measurements");
%     load("nuscenes_implementation\workspaces\measurements_global-scene"+scene_name+".mat","measurements_global");
% else
%     % For multiple detections per target - Extended object processing
%     load("nuscenes_implementation\workspaces\global_landmarks_map-multi_box_scene"+scene_name+".mat","global_landmarks_map");
%     load("nuscenes_implementation\workspaces\measurements-multi_box-scene"+scene_name+".mat","measurements");
%     load("nuscenes_implementation\workspaces\measurements_multi_box-global-scene"+scene_name+".mat","measurements_global");
% end
% 
% ax = gca;
% set(gcf, 'Position', get(0, 'Screensize'));
% hold on;
% plot_scenario(ground_truth_ego_pos, global_landmarks_map); % Both these variables come from loading the workspace
% 
% %% Filter Initialization
% timestep = 1;
% n_states = 4;
% 
% N_particles = 500;
% weight_particles = ones(1,N_particles) * 1/N_particles;
% 
% x = ground_truth_ego_pos(1,timestep);
% vx = control_input(1,timestep) * cos(control_input(2,timestep));
% y = ground_truth_ego_pos(2,timestep);
% vy = control_input(1,timestep) * sin(control_input(2,timestep));
% 
% % % Uniformly generate particles - may lead to filter degeneracy due to
% % % sample impoverishment as a result of random spread of the particles
% % x_min = [x-20, vx-5, y-20, vy-5]; % x, vx, y, vy
% % x_max = [x+20, vx+5, y+20, vy+5];
% % particles = create_uniform_particles(x_min, x_max, N);
% 
% % Particles generated from normal distribution
% mean_state = [x, vx, y, vy]; % x, vx, y, vy
% std_state = [5, 25, 5, 25]; % position with 20m 3sig error, and velocity with 5m/s 1sig error.
% particles_pre = create_gaussian_particles(mean_state, std_state, N_particles); % A array of Particles is created
% 
% % First Update
% if dead_reckoning
%     particles = particles_pre;
% else
% 	[Z_, Z_plot] = get_current_measurement(timestep, measurements, measurements_global);
%     % Either convert measurements from local frame to global frame or convert landmarks in state vector from global to local frame
%     Z_ = rotate_measurements_to_global_frame(Z_, control_input(2,timestep));
% 	[particles, weight_particles] = update(particles_pre, weight_particles, Z_);
%     if effective_no(weight_particles) < N_particles/2
%         indexes = systematic_resampling(weight_particles);
%         [particles, weight_particles] = resample_from_index(particles, weight_particles, indexes);
%     end
%     % After resampling from indices, check if the weights should be
%     % assigned equally or not.
% end
% 
% plot_on_global_map(timestep, particles_pre, particles, weight_particles, ground_truth_ego_pos, measurements_global, ax);
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
%         velocity = control_input(1,timestep);
%         heading = control_input(2,timestep);
%         particles_pre = predict(particles, T, velocity, heading);
%         if dead_reckoning
%             particles = particles_pre;
%         else
%             % Either convert measurements from local frame to global frame or convert landmarks in state vector from global to local frame
%             Z_ = rotate_measurements_to_global_frame(Z_, control_input(2,timestep));
%             [particles, weight_particles] = update(particles_pre, weight_particles, Z_);
% %             if effective_no(weight_particles) < N_particles/2
%                 indexes = systematic_resampling(weight_particles);
%                 [particles, weight_particles] = resample_from_index(particles, weight_particles, indexes);
% %             end
%         end
%         plot_on_global_map(timestep, particles_pre, particles, weight_particles, ground_truth_ego_pos, measurements_global, ax);
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
%     plot(ground_truth_ego_pos(1,:), ground_truth_ego_pos(2,:), 'y-o', 'MarkerFaceColor', 'y');
% 
%     % Plot landmarks in global coordinates
%     global_landmarks_map_keys = global_landmarks_map.keys;
%     for i = 1: length(global_landmarks_map.keys)
%         land = global_landmarks_map(global_landmarks_map_keys{i});
%         plot(land(1), land(2), "y*");
% %         text(land(1), land(2), global_landmarks_map_keys{i}, "Color", "b");
%     end
% end
% 
% function plot_on_global_map(timestep, particles_pre, particles, weights, ground_truth_ego_pos, measurements_global, ax)
%     global global_point_landmark_map;
%     legend off;
%     delete(findall(ax, "Tag","current_ground_truth_position"));
% 	delete(findall(ax, "Tag","mean_cluster"));
% 	delete(findall(ax, "Tag","unclustered"));
% 	delete(findall(ax, "Tag","cluster_id"));
%     delete(findall(ax, "Tag","landmarks"));
%     delete(findall(ax,'Tag','particles_predicted'));
%     delete(findall(ax,'Tag','particles'));
%     
%     % Prepare canvas for plotting
%     for i = 1:size(particles_pre,2)
%         plot(particles_pre(i).X(1), particles_pre(i).X(3),'c.','Tag','particles_predicted');
%     end
%     for i = 1:size(particles,2)
%         plot(particles(i).X(1), particles(i).X(3),'k.','Tag','particles');
%     end
% 
%     [point_mean, point_var] = point_estimate(particles_pre, weights);
%     plot(point_mean(1), point_mean(3), "c*", 'Tag', "point_particles_predicted");
%     
%     [point_mean, point_var] = point_estimate(particles, weights);
%     plot(point_mean(1), point_mean(3), "k*", 'Tag', "point_particles");
%     
%     landmark_temp = global_point_landmark_map.values;
%     landmark_positions = [];
%     for i = 1:length(landmark_temp)
%         landmark_positions = [landmark_positions; landmark_temp{i}.X'];
%     end
%     if ~isempty(landmark_positions)
%         plot(landmark_positions(:,1),landmark_positions(:,2),'g+','Tag','landmarks','MarkerSize',5);
%     end
%     
% 	plot(ground_truth_ego_pos(1,timestep), ground_truth_ego_pos(2,timestep), "m*", "Tag","current_ground_truth_position");
% 	scatter(measurements_global{timestep}.zpos(1,:), measurements_global{timestep}.zpos(2,:), 'Marker','o','MarkerFaceColor' ,'g', 'MarkerEdgeColor','g', "Tag", "mean_cluster");
% 	scatter(measurements_global{timestep}.detections(1,:), measurements_global{timestep}.detections(2,:), 'Marker','o', 'MarkerEdgeColor','b', "Tag", "unclustered");
% % 	text(measurements_global{timestep}.zpos(1,:), measurements_global{timestep}.zpos(2,:), measurements_global{timestep}.id', "Tag", "cluster_id", "Color", "g");
%     drawnow;
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
% % Predict equations
% function [particles] = predict(particles, T, velocity, heading)
%     std_velocity = 50;
%     std_heading = deg2rad(50);
%     for p = 1:size(particles,2) % Parallel toolbox
%         random_velocity = randn(1);
%         random_heading = randn(1);
%         particles(p).X(1) = particles(p).X(1) + (T*(velocity + (random_velocity*std_velocity))*cos(heading + (random_heading*std_heading)));
%         particles(p).X(2) = (velocity + (random_velocity*std_velocity))* cos(heading + (random_heading*std_heading));
%         particles(p).X(3) = particles(p).X(3) + T*(velocity + (random_velocity*std_velocity))*sin(heading + (random_heading*std_heading));
%         particles(p).X(4) = (velocity + (random_velocity*std_velocity))* sin(heading + (random_heading*std_heading));
%     end
% end
% 
% % Update equations
% function [particles, weights] = update(particles, weights, Z) 
%     global global_point_landmark_map;
%     
%     % For already observed landmarks
%     for m = 1:size(Z.id,2)
%         if global_point_landmark_map.isKey(string(Z.id(m))) 
%             point_map_mean = [0;0];
%             point_map_var = [0;0];
%             for p = 1:size(particles,2)
%                 H = [1, 0;
%                      0, 1];
%                 R = [1, 0;
%                      0, 1];
%                 land = particles(p).local_landmark_map(string(Z.id(m)));
%                 X_pre = land.X - particles(p).X([1,3],1);
% 
%                 % Innovation
%                 Z_ = Z.zpos(1:2,m);
%                 Y = Z_(:) - H * X_pre;
%                 % Innovation covariance
%                 P_pre = land.P;
%                 S = H * P_pre * H' + R;
%                 % Kalman Gain
%                 K = P_pre * H' * S^(-1);
%                 % Update State 
%                 land.X = land.X + K * Y;
%                 % Update State Covariance
%                 land.P = P_pre - K*H*P_pre;
%                 land.P = (land.P + land.P')/2;
% 
%                 particles(p).local_landmark_map(string(Z.id(m))) = land;
%                 
%                 % Calculate weight of the particle
%                 weights(p) = weights(p) * exp(-0.5 *Y'*S^(-1)*Y)/sqrt(2*pi*det(S));
% 
%                 point_map_mean = point_map_mean + (land.X * weights(p));
%                 point_map_var = point_map_var + ((land.X - point_map_mean).^2) * weights(p);
%             end
% 
%             point_map_mean = point_map_mean / sum(weights);
%             point_map_var = point_map_var / sum(weights);
%             global_land.X = point_map_mean;
%             global_land.P = point_map_var;
%             global_point_landmark_map(string(Z.id(m))) = global_land;
%         end
%     end
% 
%     % For new landmarks
%     for m = 1:size(Z.id,2)
%         if ~global_point_landmark_map.isKey(string(Z.id(m))) 
%             point_map_mean = [0;0];
%             point_map_var = [0;0];
%             for p = 1:size(particles,2)
%                 landmark.X = particles(p).X([1,3],1) + Z.zpos(1:2,m);
%                 landmark.P = blkdiag(25,25);
%                 particles(p).local_landmark_map(string(Z.id(m))) = landmark;
%                 point_map_mean = point_map_mean + (landmark.X * weights(p));
%                 point_map_var = point_map_var + ((landmark.X - point_map_mean).^2) * weights(p); 
%             end
%             point_map_mean = point_map_mean / sum(weights);
%             point_map_var = point_map_var / sum(weights);
%             global_land.X = point_map_mean;
%             global_land.P = point_map_var;
%             global_point_landmark_map(string(Z.id(m))) = global_land;
%         end
%     end
%     
%     weights = weights/sum(weights);
% end
% 
% function Z_ = rotate_measurements_to_global_frame(Z_,orientation)
%     q = quaternion(angle2quat(orientation, 0, 0, 'ZYX'));
%     q_rotmat = rotmat(q,'point');   
%     temp = rotate_point_to_global_frame([Z_.zpos(1:2,:);zeros(1,size(Z_.zpos,2))],q_rotmat);
%     Z_.zpos(1:2,:) = temp(1:2,:);
% end
%             
% function point = translate_point_to_global_frame(point, translation_matrix)
%     point(1:3,:) = point(1:3,:) + translation_matrix;
% end
% 
% function point = rotate_point_to_global_frame(point, rotation_matrix)
%     point(1:3,:) = rotation_matrix * point(1:3,:);
% end
% 
% function [particles] = create_uniform_particles(x_min, x_max, N)
%     particles = [];
%     for i = 1:N
%         x =[(x_max(1)-x_min(1)).*rand(1)+x_min(1); 
%             (x_max(2)-x_min(2)).*rand(1)+x_min(2);
%             (x_max(3)-x_min(3)).*rand(1)+x_min(3);
%             (x_max(4)-x_min(4)).*rand(1)+x_min(4)];
%         particles = [particles, Particle(x)];
%     end
% end
% 
% function [particles] = create_gaussian_particles(mean, std, N)
%     particles = [];
%     for i = 1:N
%         x =[mean(1)+(randn(1)*std(1)); 
%             mean(2)+(randn(1)*std(2));
%             mean(3)+(randn(1)*std(3)); 
%             mean(4)+(randn(1)*std(4))];
%         particles = [particles, Particle(x)];
%     end
% end
% 
% % Effective N for resampling condition
% function [Nf] = effective_no(weights)
%     Nf = 1/sum(weights.^2); 
% end
% 
% % Resampling from indices
% function [particles, weights] = resample_from_index(particles, weights, indexes)
%     particles = particles(indexes);
%     weights = ones(1,size(weights,2)) * 1/size(weights,2);
% end
% 
% % Stratified Resampling algorithm
% function indices = stratified_resampling(weights)
%     n_samples = length(weights);
%     positions = ((0:(n_samples-1)) + rand(1,n_samples))/(n_samples);
%     indices = zeros(1,n_samples);
%     cum_sum = cumsum(weights);
%     i = 0;
%     j = 0;
%     while i < n_samples
%        if positions(i+1) < cum_sum(j+1)
%            indices(i+1) = j;
%            i = i+1;
%        else
%            j = j+1;
%        end
%     end
%     indices = indices + 1;
% end
% 
% % Systematic resampling algorithm
% function indices = systematic_resampling(weights)
%     n_samples = length(weights);
%     positions = ((0:n_samples-1) + rand(1))/(n_samples);
%     indices = zeros(1,n_samples);
%     cum_sum = cumsum(weights);
%     i = 0;
%     j = 0;
%     while i < n_samples
%        if positions(i+1) < cum_sum(j+1)
%            indices(i+1) = j;
%            i = i+1;
%        else
%            j = j+1;
%        end
%     end
%     indices = indices + 1;
% end
% 
% % Point estimate
% function [point_mean, point_var] = point_estimate(particles, weights)
%     point_mean = zeros(4,1);
%     point_var = zeros(4,1);
%     for i = 1:size(particles,2)
%         point_mean = point_mean + (particles(i).X * weights(i));
%         point_var  = point_var + ((particles(i).X - point_mean).^2 * weights(i));
%     end
%     point_mean = point_mean / sum(weights);
%     point_var = point_var / sum(weights);
% end



%% Backup - 3
% %% SLAM
% clc;
% close all;
% clear all;
% rand_num_gen = rng(100); % Initialize random number generator
% scene_name = ["0655","0916"];
% scene_name = scene_name(2);
% dead_reckoning = false;
% extended_object_processing = false;
% 
% global landmark_index_map;
% landmark_index_map = containers.Map;
% global global_point_landmark_map;
% global_point_landmark_map = containers.Map;
% 
% load("nuscenes_implementation\workspaces\ground_truth-scene"+scene_name+".mat","ground_truth_ego_pos");
% load("nuscenes_implementation\workspaces\control_input_1-scene"+scene_name+".mat","control_input");
% load("nuscenes_implementation\workspaces\timestamp-scene"+scene_name+".mat","timestamp");
% 
% if ~extended_object_processing
%     % For one detection per target - Point object processing
%     load("nuscenes_implementation\workspaces\global_landmarks_map-scene"+scene_name+".mat","global_landmarks_map");
%     load("nuscenes_implementation\workspaces\measurements-scene"+scene_name+".mat","measurements");
%     load("nuscenes_implementation\workspaces\measurements_global-scene"+scene_name+".mat","measurements_global");
% else
%     % For multiple detections per target - Extended object processing
%     load("nuscenes_implementation\workspaces\global_landmarks_map-multi_box_scene"+scene_name+".mat","global_landmarks_map");
%     load("nuscenes_implementation\workspaces\measurements-multi_box-scene"+scene_name+".mat","measurements");
%     load("nuscenes_implementation\workspaces\measurements_multi_box-global-scene"+scene_name+".mat","measurements_global");
% end
% 
% ax = gca;
% set(gcf, 'Position', get(0, 'Screensize'));
% hold on;
% plot_scenario(ground_truth_ego_pos, global_landmarks_map); % Both these variables come from loading the workspace
% 
% %% Filter Initialization
% timestep = 1;
% 
% N_particles = 200;
% weight_particles = ones(1,N_particles) * 1/N_particles;
% 
% x = ground_truth_ego_pos(1,timestep);
% y = ground_truth_ego_pos(2,timestep);
% h = control_input(2,timestep);
% 
% % % Uniformly generate particles - may lead to filter degeneracy due to
% % % sample impoverishment as a result of random spread of the particles
% % x_min = [x-20, y-20, h-0.78539]; % x, y, heading
% % x_max = [x+20, y+20, h+0.78539];
% % particles = create_uniform_particles(x_min, x_max, N);
% 
% % Particles generated from normal distribution
% mean_state = [x, y, h]; % x, y, heading
% std_state = [45, 45, 0.0685]; % position with 20m 3sig error, and heading with 45deg 3sig error.
% particles_pre = create_gaussian_particles(mean_state, std_state, N_particles); % A array of Particles is created
% 
% % First Update
% if dead_reckoning
%     particles = particles_pre;
% else
% 	[Z_, Z_plot] = get_current_measurement(timestep, measurements, measurements_global);
%     % Either convert measurements from local frame to global frame or convert landmarks in state vector from global to local frame
% %     Z_ = rotate_measurements_to_global_frame(Z_, control_input(2,timestep)); % Rotate measurements for each particle's orientation
% 	[particles, weight_particles] = update(particles_pre, weight_particles, Z_, control_input(2,timestep));
%     if effective_no(weight_particles) < N_particles/2
%         indexes = systematic_resampling(weight_particles);
%         [particles, weight_particles] = resample_from_index(particles, weight_particles, indexes);
%     end
% end
% 
% plot_on_global_map(timestep, particles_pre, particles, weight_particles, ground_truth_ego_pos, measurements_global, ax);
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
%         velocity = control_input(1,timestep);
%         heading = control_input(2,timestep);
%         particles_pre = predict(particles, T, velocity, heading);
%         if dead_reckoning
%             particles = particles_pre;
%         else
%             % Either convert measurements from local frame to global frame or convert landmarks in state vector from global to local frame
% %             Z_ = rotate_measurements_to_global_frame(Z_, control_input(2,timestep));
%             [particles, weight_particles] = update(particles_pre, weight_particles, Z_, control_input(2,timestep));
% %             if effective_no(weight_particles) < N_particles/2
%                 indexes = systematic_resampling(weight_particles);
%                 [particles, weight_particles] = resample_from_index(particles, weight_particles, indexes);
% %             end
%         end
%         plot_on_global_map(timestep, particles_pre, particles, weight_particles, ground_truth_ego_pos, measurements_global, ax);
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
%     plot(ground_truth_ego_pos(1,:), ground_truth_ego_pos(2,:), 'y-o', 'MarkerFaceColor', 'y');
% 
%     % Plot landmarks in global coordinates
%     global_landmarks_map_keys = global_landmarks_map.keys;
%     for i = 1: length(global_landmarks_map.keys)
%         land = global_landmarks_map(global_landmarks_map_keys{i});
%         plot(land(1), land(2), "y*");
% %         text(land(1), land(2), global_landmarks_map_keys{i}, "Color", "b");
%     end
% end
% 
% function plot_on_global_map(timestep, particles_pre, particles, weights, ground_truth_ego_pos, measurements_global, ax)
%     global global_point_landmark_map;
%     legend off;
%     delete(findall(ax, "Tag","current_ground_truth_position"));
% 	delete(findall(ax, "Tag","mean_cluster"));
% 	delete(findall(ax, "Tag","unclustered"));
% 	delete(findall(ax, "Tag","cluster_id"));
%     delete(findall(ax, "Tag","landmarks"));
%     delete(findall(ax,'Tag','particles_predicted'));
%     delete(findall(ax,'Tag','particles'));
%     
%     % Prepare canvas for plotting
%     for i = 1:size(particles_pre,2)
%         plot(particles_pre(i).X(1), particles_pre(i).X(2),'c.','Tag','particles_predicted');
%     end
%     for i = 1:size(particles,2)
%         plot(particles(i).X(1), particles(i).X(2),'k.','Tag','particles');
%     end
% 
%     [point_mean, point_var] = point_estimate(particles_pre, weights);
%     plot(point_mean(1), point_mean(2), "c*", 'Tag', "point_particles_predicted");
%     
%     [point_mean, point_var] = point_estimate(particles, weights);
%     plot(point_mean(1), point_mean(2), "k*", 'Tag', "point_particles");
%     
% %     landmark_temp = global_point_landmark_map.values;
% %     landmark_positions = [];
% %     for i = 1:length(landmark_temp)
% %         landmark_positions = [landmark_positions; landmark_temp{i}.X'];
% %     end
% %     if ~isempty(landmark_positions)
% %         plot(landmark_positions(:,1),landmark_positions(:,2),'g+','Tag','landmarks','MarkerSize',5);
% %     end
%     
%     
% 	plot(ground_truth_ego_pos(1,timestep), ground_truth_ego_pos(2,timestep), "m*", "Tag","current_ground_truth_position");
% 	scatter(measurements_global{timestep}.zpos(1,:), measurements_global{timestep}.zpos(2,:), 'Marker','o','MarkerFaceColor' ,'g', 'MarkerEdgeColor','g', "Tag", "mean_cluster");
% 	scatter(measurements_global{timestep}.detections(1,:), measurements_global{timestep}.detections(2,:), 'Marker','o', 'MarkerEdgeColor','b', "Tag", "unclustered");
% % 	text(measurements_global{timestep}.zpos(1,:), measurements_global{timestep}.zpos(2,:), measurements_global{timestep}.id', "Tag", "cluster_id", "Color", "g");
%     drawnow;
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
% % Predict equations
% function [particles] = predict(particles, T, velocity, heading)
%     std_velocity = 4;
%     std_heading = deg2rad(10);
%     for p = 1:size(particles,2) % Parallel toolbox
%         random_velocity = randn(1);
%         random_heading = randn(1);
%         particles(p).X(1) = particles(p).X(1) + (T*(velocity + (random_velocity*std_velocity))*cos(heading + (random_heading*std_heading)));
%         particles(p).X(2) = particles(p).X(2) + (T*(velocity + (random_velocity*std_velocity))*sin(heading + (random_heading*std_heading)));
%         particles(p).X(3) = heading + (random_heading*std_heading);
%     end
% end
% 
% % Update equations
% function [particles, weights] = update(particles, weights, Z, control_heading) 
%     global global_point_landmark_map;
%     
%     % For already observed landmarks
%     for m = 1:size(Z.id,2)
%         if global_point_landmark_map.isKey(string(Z.id(m))) 
%             for p = 1:size(particles,2)
%                 land = struct;
%                 land.X = particles(p).local_landmark_map(string(Z.id(m))).X;
%                 land.P = particles(p).local_landmark_map(string(Z.id(m))).P;
%                 
%                 x_prime = land.X(1) - particles(p).X(1);
%                 y_prime = land.X(2) - particles(p).X(2);
%                 h = [sqrt(x_prime^2 + y_prime^2); atan2(y_prime, x_prime) - particles(p).X(3)]; % For bearing, either subtract the orientation of vehicle here, or rotate measurements to global frame before calling this function 
%                 R = blkdiag(5,5); %one_0916:5,5 ; multi_0916: 0.1, 0.005; one_0655:5,0.001 ; multi_0655: 0.1, 0.005
%             
%                 measured_range = sqrt(Z.zpos(1,m)^2 + Z.zpos(2,m)^2);
%                 measured_bearing = atan2(Z.zpos(2,m),Z.zpos(1,m));
%                 Z_ = [measured_range; measured_bearing];            
%             
%                 % Innovation
%                 Y = Z_ - h;
%                 
%                 H = [x_prime/sqrt(x_prime^2 + y_prime^2), y_prime/sqrt(x_prime^2 + y_prime^2);
%                      -y_prime/(x_prime^2 + y_prime^2),    x_prime/(x_prime^2 + y_prime^2)   ];
%                 
%                 % Innovation covariance
%                 S = H * land.P * H' + R;
%                 % Kalman Gain
%                 K = land.P * H' * S^(-1);
%                 % Update State 
%                 land.X = land.X + K * Y;
%                 % Update State Covariance
%                 land.P = land.P - K*H*land.P;
%                 land.P = (land.P + land.P')/2;
% 
%                 particles(p).local_landmark_map(string(Z.id(m))) = land;
%                 
%                 % Calculate weight of the particle
%                 weights(p) = weights(p) * exp(-0.5 * Y' * S^(-1) * Y)/sqrt(2*pi*det(S));
%             end
%             global_point_landmark_map(string(Z.id(m))) = 1;
%         end
%     end
% 
%     % For new landmarks
%     for m = 1:size(Z.id,2)
%         if ~global_point_landmark_map.isKey(string(Z.id(m))) 
%             for p = 1:size(particles,2)
%                 measured_range = sqrt(Z.zpos(1,m)^2 + Z.zpos(2,m)^2);
%                 measured_bearing = atan2(Z.zpos(2,m),Z.zpos(1,m));
%                 particle_state = particles(p).X;
%                 landmark = struct;
%                 landmark.X(1,1) = particle_state(1) + (measured_range * cos(particle_state(3) + measured_bearing));
%                 landmark.X(2,1) = particle_state(2) + (measured_range * sin(particle_state(3) + measured_bearing));
%                 landmark.P = blkdiag(25,25);
%                 particles(p).local_landmark_map(string(Z.id(m))) = landmark;
%             end
%             global_point_landmark_map(string(Z.id(m))) = 1;
%         end
%     end
%     
%     weights = weights + 1e-300;
%     weights = weights/sum(weights);
% end
% 
% function Z_ = rotate_measurements_to_global_frame(Z_,orientation)
%     q = quaternion(angle2quat(orientation, 0, 0, 'ZYX'));
%     q_rotmat = rotmat(q,'point');   
%     temp = rotate_point_to_global_frame([Z_.zpos(1:2,:);zeros(1,size(Z_.zpos,2))],q_rotmat);
%     Z_.zpos(1:2,:) = temp(1:2,:);
% end
% 
% function Z_ = rotate_one_measurement_to_global_frame(Z_,orientation)
%     q = quaternion(angle2quat(orientation, 0, 0, 'ZYX'));
%     q_rotmat = rotmat(q,'point');   
%     temp = rotate_point_to_global_frame([Z_(1:2,:);zeros(1,size(Z_,2))],q_rotmat);
%     Z_(1:2,:) = temp(1:2,:);
% end
% 
% function point = translate_point_to_global_frame(point, translation_matrix)
%     point(1:3,:) = point(1:3,:) + translation_matrix;
% end
% 
% function point = rotate_point_to_global_frame(point, rotation_matrix)
%     point(1:3,:) = rotation_matrix * point(1:3,:);
% end
% 
% function [particles] = create_uniform_particles(x_min, x_max, N)
%     particles = [];
%     for i = 1:N
%         x =[(x_max(1)-x_min(1)).*rand(1)+x_min(1); 
%             (x_max(2)-x_min(2)).*rand(1)+x_min(2);
%             (x_max(3)-x_min(3)).*rand(1)+x_min(3)];
%         particles = [particles, Particle(x)];
%     end
% end
% 
% function [particles] = create_gaussian_particles(mean, std, N)
%     particles = [];
%     for i = 1:N
%         x =[mean(1)+(randn(1)*std(1)); 
%             mean(2)+(randn(1)*std(2));
%             mean(3)+(randn(1)*std(3))];
%         particles = [particles, Particle(x)];
%     end
% end
% 
% % Effective N for resampling condition
% function [Nf] = effective_no(weights)
%     Nf = 1/sum(weights.^2); 
% end
% 
% % Resampling from indices
% function [particles, weights] = resample_from_index(particles, weights, indexes)
%     particles = particles(indexes);
%     weights = ones(1,size(weights,2)) * 1/size(weights,2);
% end
% 
% % Stratified Resampling algorithm
% function indices = stratified_resampling(weights)
%     n_samples = length(weights);
%     positions = ((0:(n_samples-1)) + rand(1,n_samples))/(n_samples);
%     indices = zeros(1,n_samples);
%     cum_sum = cumsum(weights);
%     i = 0;
%     j = 0;
%     while i < n_samples
%        if positions(i+1) < cum_sum(j+1)
%            indices(i+1) = j;
%            i = i+1;
%        else
%            j = j+1;
%        end
%     end
%     indices = indices + 1;
% end
% 
% % Systematic resampling algorithm
% function indices = systematic_resampling(weights)
%     n_samples = length(weights);
%     positions = ((0:n_samples-1) + rand(1))/(n_samples);
%     indices = zeros(1,n_samples);
%     cum_sum = cumsum(weights);
%     i = 0;
%     j = 0;
%     while i < n_samples
%        if positions(i+1) < cum_sum(j+1)
%            indices(i+1) = j;
%            i = i+1;
%        else
%            j = j+1;
%        end
%     end
%     indices = indices + 1;
% end
% 
% % Point estimate
% function [point_mean, point_var] = point_estimate(particles, weights)
%     point_mean = zeros(3,1);
%     point_var = zeros(3,1);
%     for i = 1:size(particles,2)
%         point_mean = point_mean + (particles(i).X * weights(i));
%         point_var  = point_var + ((particles(i).X - point_mean).^2 * weights(i));
%     end
%     point_mean = point_mean / sum(weights);
%     point_var = point_var / sum(weights);
% end
% 
% 



%% Backup - 2
% %% SLAM
% clc;
% close all;
% clear all;
% rand_num_gen = rng(100); % Initialize random number generator
% scene_name = ["0655","0916"];
% scene_name = scene_name(2);
% dead_reckoning = false;
% extended_object_processing = false;
% global n_states;
% global landmark_index_map;
% landmark_index_map = containers.Map;
% global global_point_landmark_map;
% global_point_landmark_map = containers.Map;
% 
% load("nuscenes_implementation\workspaces\ground_truth-scene"+scene_name+".mat","ground_truth_ego_pos");
% load("nuscenes_implementation\workspaces\control_input_1-scene"+scene_name+".mat","control_input");
% load("nuscenes_implementation\workspaces\timestamp-scene"+scene_name+".mat","timestamp");
% 
% if ~extended_object_processing
%     % For one detection per target - Point object processing
%     load("nuscenes_implementation\workspaces\global_landmarks_map-scene"+scene_name+".mat","global_landmarks_map");
%     load("nuscenes_implementation\workspaces\measurements-scene"+scene_name+".mat","measurements");
%     load("nuscenes_implementation\workspaces\measurements_global-scene"+scene_name+".mat","measurements_global");
% else
%     % For multiple detections per target - Extended object processing
%     load("nuscenes_implementation\workspaces\global_landmarks_map-multi_box_scene"+scene_name+".mat","global_landmarks_map");
%     load("nuscenes_implementation\workspaces\measurements-multi_box-scene"+scene_name+".mat","measurements");
%     load("nuscenes_implementation\workspaces\measurements_multi_box-global-scene"+scene_name+".mat","measurements_global");
% end
% 
% ax = gca;
% set(gcf, 'Position', get(0, 'Screensize'));
% hold on;
% plot_scenario(ground_truth_ego_pos, global_landmarks_map); % Both these variables come from loading the workspace
% 
% %% Filter Initialization
% timestep = 1;
% n_states = 4;
% 
% N_particles = 3;
% weight_particles = ones(1,N_particles) * 1/N_particles;
% 
% x = ground_truth_ego_pos(1,timestep);
% y = ground_truth_ego_pos(2,timestep);
% h = control_input(2,timestep);
% 
% % % Uniformly generate particles - may lead to filter degeneracy due to
% % % sample impoverishment as a result of random spread of the particles
% % x_min = [x-20, y-20, h-0.78539]; % x, y, heading
% % x_max = [x+20, y+20, h+0.78539];
% % particles = create_uniform_particles(x_min, x_max, N);
% 
% % Particles generated from normal distribution
% mean_state = [x, y, h]; % x, y, heading
% std_state = [45, 45, 0.0685]; % position with 20m 3sig error, and heading with 45deg 3sig error.
% particles_pre = create_gaussian_particles(mean_state, std_state, N_particles); % A array of Particles is created
% 
% % First Update
% if dead_reckoning
%     particles = particles_pre;
% else
% 	[Z_, Z_plot] = get_current_measurement(timestep, measurements, measurements_global);
%     % Either convert measurements from local frame to global frame or convert landmarks in state vector from global to local frame
% %     Z_ = rotate_measurements_to_global_frame(Z_, control_input(2,timestep)); % Rotate measurements for each particle's orientation
% 	[particles, weight_particles] = update(particles_pre, weight_particles, Z_, control_input(2,timestep));
%     if effective_no(weight_particles) < N_particles/2
%         indexes = systematic_resampling(weight_particles);
%         [particles, weight_particles] = resample_from_index(particles, weight_particles, indexes);
%     end
% end
% 
% plot_on_global_map(timestep, particles_pre, particles, weight_particles, ground_truth_ego_pos, measurements_global, ax);
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
%         velocity = control_input(1,timestep);
%         heading = control_input(2,timestep);
%         particles_pre = predict(particles, T, velocity, heading);
%         if dead_reckoning
%             particles = particles_pre;
%         else
%             % Either convert measurements from local frame to global frame or convert landmarks in state vector from global to local frame
% %             Z_ = rotate_measurements_to_global_frame(Z_, control_input(2,timestep));
%             [particles, weight_particles] = update(particles_pre, weight_particles, Z_, control_input(2,timestep));
% %             if effective_no(weight_particles) < N_particles/2
%                 indexes = systematic_resampling(weight_particles);
%                 [particles, weight_particles] = resample_from_index(particles, weight_particles, indexes);
% %             end
%         end
%         plot_on_global_map(timestep, particles_pre, particles, weight_particles, ground_truth_ego_pos, measurements_global, ax);
%         pause(1);
%         T = 0;
%     end
%     prev_time = timestamp(timestep);
% end
% hold off;
% 
% %% Helper functions
% function plot_scenario(ground_truth_ego_pos, global_landmarks_map)
%     % Plot ground truth trajectory of the ego car
%     plot(ground_truth_ego_pos(1,:), ground_truth_ego_pos(2,:), 'y-o', 'MarkerFaceColor', 'y');
% 
%     % Plot landmarks in global coordinates
%     global_landmarks_map_keys = global_landmarks_map.keys;
%     for i = 1: length(global_landmarks_map.keys)
%         land = global_landmarks_map(global_landmarks_map_keys{i});
%         plot(land(1), land(2), "y*");
% %         text(land(1), land(2), global_landmarks_map_keys{i}, "Color", "b");
%     end
% end
% 
% function plot_on_global_map(timestep, particles_pre, particles, weights, ground_truth_ego_pos, measurements_global, ax)
%     global global_point_landmark_map;
%     legend off;
%     delete(findall(ax, "Tag","current_ground_truth_position"));
% 	delete(findall(ax, "Tag","mean_cluster"));
% 	delete(findall(ax, "Tag","unclustered"));
% 	delete(findall(ax, "Tag","cluster_id"));
%     delete(findall(ax, "Tag","landmarks"));
%     delete(findall(ax,'Tag','particles_predicted'));
%     delete(findall(ax,'Tag','particles'));
%     
%     % Prepare canvas for plotting
%     for i = 1:size(particles_pre,2)
%         plot(particles_pre(i).X(1), particles_pre(i).X(2),'c.','Tag','particles_predicted');
%     end
%     for i = 1:size(particles,2)
%         plot(particles(i).X(1), particles(i).X(2),'k.','Tag','particles');
%     end
% 
%     [point_mean, point_var] = point_estimate(particles_pre, weights);
%     plot(point_mean(1), point_mean(2), "c*", 'Tag', "point_particles_predicted");
%     
%     [point_mean, point_var] = point_estimate(particles, weights);
%     plot(point_mean(1), point_mean(2), "k*", 'Tag', "point_particles");
%     
%     landmark_temp = global_point_landmark_map.values;
%     landmark_positions = [];
%     for i = 1:length(landmark_temp)
%         landmark_positions = [landmark_positions; landmark_temp{i}.X'];
%     end
%     if ~isempty(landmark_positions)
%         plot(landmark_positions(:,1),landmark_positions(:,2),'g+','Tag','landmarks','MarkerSize',5);
%     end
%     
% 	plot(ground_truth_ego_pos(1,timestep), ground_truth_ego_pos(2,timestep), "m*", "Tag","current_ground_truth_position");
% 	scatter(measurements_global{timestep}.zpos(1,:), measurements_global{timestep}.zpos(2,:), 'Marker','o','MarkerFaceColor' ,'g', 'MarkerEdgeColor','g', "Tag", "mean_cluster");
% 	scatter(measurements_global{timestep}.detections(1,:), measurements_global{timestep}.detections(2,:), 'Marker','o', 'MarkerEdgeColor','b', "Tag", "unclustered");
% % 	text(measurements_global{timestep}.zpos(1,:), measurements_global{timestep}.zpos(2,:), measurements_global{timestep}.id', "Tag", "cluster_id", "Color", "g");
%     drawnow;
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
% % Predict equations
% function [particles] = predict(particles, T, velocity, heading)
%     std_velocity = 5;
%     std_heading = deg2rad(5);
%     for p = 1:size(particles,2) % Parallel toolbox
%         random_velocity = randn(1);
%         random_heading = randn(1);
%         particles(p).X(1) = particles(p).X(1) + (T*(velocity + (random_velocity*std_velocity))*cos(heading + (random_heading*std_heading)));
%         particles(p).X(2) = particles(p).X(2) + (T*(velocity + (random_velocity*std_velocity))*sin(heading + (random_heading*std_heading)));
%         particles(p).X(3) = heading + (random_heading*std_heading);
%     end
% end
% 
% % Update equations
% function [particles, weights] = update(particles, weights, Z, control_heading) 
%     global global_point_landmark_map;
%     
%     % For already observed landmarks
%     for m = 1:size(Z.id,2)
%         if global_point_landmark_map.isKey(string(Z.id(m))) 
%             point_map_mean = [0;0];
%             point_map_var = [0;0];
%             for p = 1:size(particles,2)
%                 H = [1, 0;
%                      0, 1];
%                 R = [1, 0;
%                      0, 1];
%                 land = particles(p).local_landmark_map(string(Z.id(m)));
%                 X_pre = land.X - particles(p).X([1,2],1);
% 
%                 Z_ = rotate_measurements_to_global_frame(Z.zpos(1:2,m), control_heading);
%                 % Innovation
% %                 Z_ = Z_;
%                 Y = Z_(:) - H * X_pre;
%                 % Innovation covariance
%                 P_pre = land.P;
%                 S = H * P_pre * H' + R;
%                 % Kalman Gain
%                 K = P_pre * H' * S^(-1);
%                 % Update State 
%                 land.X = land.X + K * Y;
%                 % Update State Covariance
%                 land.P = P_pre - K*H*P_pre;
%                 land.P = (land.P + land.P')/2;
% 
%                 particles(p).local_landmark_map(string(Z.id(m))) = land;
%                 
%                 % Calculate weight of the particle
%                 weights(p) = weights(p) * exp(-0.5 *Y'*S^(-1)*Y)/sqrt(2*pi*det(S));
% 
%                 point_map_mean = point_map_mean + (land.X * weights(p));
%                 point_map_var = point_map_var + ((land.X - point_map_mean).^2) * weights(p);
%             end
% 
%             point_map_mean = point_map_mean / sum(weights);
%             point_map_var = point_map_var / sum(weights);
%             global_land.X = point_map_mean;
%             global_land.P = point_map_var;
%             global_point_landmark_map(string(Z.id(m))) = global_land;
%         end
%     end
% 
%     % For new landmarks
%     for m = 1:size(Z.id,2)
%         if ~global_point_landmark_map.isKey(string(Z.id(m))) 
%             point_map_mean = [0;0];
%             point_map_var = [0;0];
%             for p = 1:size(particles,2)
%                 landmark.X = particles(p).X;
%                 Z_ = rotate_measurements_to_global_frame(Z.zpos(1:2,m), particles(p).X(3));
%                 landmark.X = landmark.X([1,2],1) + Z_;
%                 landmark.P = blkdiag(25,25);
%                 particles(p).local_landmark_map(string(Z.id(m))) = landmark;
%                 
%                 point_map_mean = point_map_mean + (landmark.X * weights(p));
%                 point_map_var = point_map_var + ((landmark.X - point_map_mean).^2) * weights(p); 
%             end
%             point_map_mean = point_map_mean / sum(weights);
%             point_map_var = point_map_var / sum(weights);
%             global_land.X = point_map_mean;
%             global_land.P = point_map_var;
%             global_point_landmark_map(string(Z.id(m))) = global_land;
%         end
%     end
%     
%     weights = weights/sum(weights);
% end
% 
% function Z_ = rotate_measurements_to_global_frame(Z_,orientation)
%     q = quaternion(angle2quat(orientation, 0, 0, 'ZYX'));
%     q_rotmat = rotmat(q,'point');   
%     temp = rotate_point_to_global_frame([Z_(1:2,:);zeros(1,size(Z_,2))],q_rotmat);
%     Z_(1:2,:) = temp(1:2,:);
% end
% 
% function point = translate_point_to_global_frame(point, translation_matrix)
%     point(1:3,:) = point(1:3,:) + translation_matrix;
% end
% 
% function point = rotate_point_to_global_frame(point, rotation_matrix)
%     point(1:3,:) = rotation_matrix * point(1:3,:);
% end
% 
% function [particles] = create_uniform_particles(x_min, x_max, N)
%     particles = [];
%     for i = 1:N
%         x =[(x_max(1)-x_min(1)).*rand(1)+x_min(1); 
%             (x_max(2)-x_min(2)).*rand(1)+x_min(2);
%             (x_max(3)-x_min(3)).*rand(1)+x_min(3)];
%         particles = [particles, Particle(x)];
%     end
% end
% 
% function [particles] = create_gaussian_particles(mean, std, N)
%     particles = [];
%     for i = 1:N
%         x =[mean(1)+(randn(1)*std(1)); 
%             mean(2)+(randn(1)*std(2));
%             mean(3)+(randn(1)*std(3))];
%         particles = [particles, Particle(x)];
%     end
% end
% 
% % Effective N for resampling condition
% function [Nf] = effective_no(weights)
%     Nf = 1/sum(weights.^2); 
% end
% 
% % Resampling from indices
% function [particles, weights] = resample_from_index(particles, weights, indexes)
%     particles = particles(indexes);
%     weights = ones(1,size(weights,2)) * 1/size(weights,2);
% end
% 
% % Stratified Resampling algorithm
% function indices = stratified_resampling(weights)
%     n_samples = length(weights);
%     positions = ((0:(n_samples-1)) + rand(1,n_samples))/(n_samples);
%     indices = zeros(1,n_samples);
%     cum_sum = cumsum(weights);
%     i = 0;
%     j = 0;
%     while i < n_samples
%        if positions(i+1) < cum_sum(j+1)
%            indices(i+1) = j;
%            i = i+1;
%        else
%            j = j+1;
%        end
%     end
%     indices = indices + 1;
% end
% 
% % Systematic resampling algorithm
% function indices = systematic_resampling(weights)
%     n_samples = length(weights);
%     positions = ((0:n_samples-1) + rand(1))/(n_samples);
%     indices = zeros(1,n_samples);
%     cum_sum = cumsum(weights);
%     i = 0;
%     j = 0;
%     while i < n_samples
%        if positions(i+1) < cum_sum(j+1)
%            indices(i+1) = j;
%            i = i+1;
%        else
%            j = j+1;
%        end
%     end
%     indices = indices + 1;
% end
% 
% % Point estimate
% function [point_mean, point_var] = point_estimate(particles, weights)
%     point_mean = zeros(3,1);
%     point_var = zeros(3,1);
%     for i = 1:size(particles,2)
%         point_mean = point_mean + (particles(i).X * weights(i));
%         point_var  = point_var + ((particles(i).X - point_mean).^2 * weights(i));
%     end
%     point_mean = point_mean / sum(weights);
%     point_var = point_var / sum(weights);
% end



%% Backup - 1
% %% SLAM
% clc;
% close all;
% clear all;
% rand_num_gen = rng(100); % Initialize random number generator
% scene_name = ["0655","0916"];
% scene_name = scene_name(2);
% dead_reckoning = false;
% extended_object_processing = false;
% global n_states;
% global landmark_index_map;
% landmark_index_map = containers.Map;
% global global_point_landmark_map;
% global_point_landmark_map = containers.Map;
% 
% load("nuscenes_implementation\workspaces\ground_truth-scene"+scene_name+".mat","ground_truth_ego_pos");
% load("nuscenes_implementation\workspaces\control_input_1-scene"+scene_name+".mat","control_input");
% load("nuscenes_implementation\workspaces\timestamp-scene"+scene_name+".mat","timestamp");
% 
% if ~extended_object_processing
%     % For one detection per target - Point object processing
%     load("nuscenes_implementation\workspaces\global_landmarks_map-scene"+scene_name+".mat","global_landmarks_map");
%     load("nuscenes_implementation\workspaces\measurements-scene"+scene_name+".mat","measurements");
%     load("nuscenes_implementation\workspaces\measurements_global-scene"+scene_name+".mat","measurements_global");
% else
%     % For multiple detections per target - Extended object processing
%     load("nuscenes_implementation\workspaces\global_landmarks_map-multi_box_scene"+scene_name+".mat","global_landmarks_map");
%     load("nuscenes_implementation\workspaces\measurements-multi_box-scene"+scene_name+".mat","measurements");
%     load("nuscenes_implementation\workspaces\measurements_multi_box-global-scene"+scene_name+".mat","measurements_global");
% end
% 
% ax = gca;
% set(gcf, 'Position', get(0, 'Screensize'));
% hold on;
% plot_scenario(ground_truth_ego_pos, global_landmarks_map); % Both these variables come from loading the workspace
% 
% %% Filter Initialization
% timestep = 1;
% n_states = 4;
% 
% N_particles = 3;
% weight_particles = ones(1,N_particles) * 1/N_particles;
% 
% x = ground_truth_ego_pos(1,timestep);
% vx = control_input(1,timestep) * cos(control_input(2,timestep));
% y = ground_truth_ego_pos(2,timestep);
% vy = control_input(1,timestep) * sin(control_input(2,timestep));
% 
% % % Uniformly generate particles - may lead to filter degeneracy due to
% % % sample impoverishment as a result of random spread of the particles
% % x_min = [x-20, vx-5, y-20, vy-5]; % x, vx, y, vy
% % x_max = [x+20, vx+5, y+20, vy+5];
% % particles = create_uniform_particles(x_min, x_max, N);
% 
% % Particles generated from normal distribution
% mean_state = [x, vx, y, vy]; % x, vx, y, vy
% std_state = [45, 25, 45, 25]; % position with 20m 3sig error, and velocity with 5m/s 1sig error.
% particles_pre = create_gaussian_particles(mean_state, std_state, N_particles); % A array of Particles is created
% 
% % First Update
% if dead_reckoning
%     particles = particles_pre;
% else
% 	[Z_, Z_plot] = get_current_measurement(timestep, measurements, measurements_global);
%     % Either convert measurements from local frame to global frame or convert landmarks in state vector from global to local frame
%     Z_ = rotate_measurements_to_global_frame(Z_, control_input(2,timestep));
% 	[particles, weight_particles] = update(particles_pre, weight_particles, Z_);
%     if effective_no(weight_particles) < N_particles/2
%         indexes = systematic_resampling(weight_particles);
%         [particles, weight_particles] = resample_from_index(particles, weight_particles, indexes);
%     end
%     % After resampling from indices, check if the weights should be
%     % assigned equally or not.
% end
% 
% plot_on_global_map(timestep, particles_pre, particles, weight_particles, ground_truth_ego_pos, measurements_global, ax);
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
%         velocity = control_input(1,timestep);
%         heading = control_input(2,timestep);
%         particles_pre = predict(particles, T, velocity, heading);
%         if dead_reckoning
%             particles = particles_pre;
%         else
%             % Either convert measurements from local frame to global frame or convert landmarks in state vector from global to local frame
%             Z_ = rotate_measurements_to_global_frame(Z_, control_input(2,timestep));
%             [particles, weight_particles] = update(particles_pre, weight_particles, Z_);
% %             if effective_no(weight_particles) < N_particles/2
%                 indexes = systematic_resampling(weight_particles);
%                 [particles, weight_particles] = resample_from_index(particles, weight_particles, indexes);
% %             end
%         end
%         plot_on_global_map(timestep, particles_pre, particles, weight_particles, ground_truth_ego_pos, measurements_global, ax);
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
%     plot(ground_truth_ego_pos(1,:), ground_truth_ego_pos(2,:), 'y-o', 'MarkerFaceColor', 'y');
% 
%     % Plot landmarks in global coordinates
%     global_landmarks_map_keys = global_landmarks_map.keys;
%     for i = 1: length(global_landmarks_map.keys)
%         land = global_landmarks_map(global_landmarks_map_keys{i});
%         plot(land(1), land(2), "y*");
% %         text(land(1), land(2), global_landmarks_map_keys{i}, "Color", "b");
%     end
% end
% 
% function plot_on_global_map(timestep, particles_pre, particles, weights, ground_truth_ego_pos, measurements_global, ax)
%     global global_point_landmark_map;
%     legend off;
%     delete(findall(ax, "Tag","current_ground_truth_position"));
% 	delete(findall(ax, "Tag","mean_cluster"));
% 	delete(findall(ax, "Tag","unclustered"));
% 	delete(findall(ax, "Tag","cluster_id"));
%     delete(findall(ax, "Tag","landmarks"));
%     delete(findall(ax,'Tag','particles_predicted'));
%     delete(findall(ax,'Tag','particles'));
%     
%     % Prepare canvas for plotting
%     for i = 1:size(particles_pre,2)
%         plot(particles_pre(i).X(1), particles_pre(i).X(3),'c.','Tag','particles_predicted');
%     end
%     for i = 1:size(particles,2)
%         plot(particles(i).X(1), particles(i).X(3),'k.','Tag','particles');
%     end
% 
%     [point_mean, point_var] = point_estimate(particles_pre, weights);
%     plot(point_mean(1), point_mean(3), "c*", 'Tag', "point_particles_predicted");
%     
%     [point_mean, point_var] = point_estimate(particles, weights);
%     plot(point_mean(1), point_mean(3), "k*", 'Tag', "point_particles");
%     
%     landmark_temp = global_point_landmark_map.values;
%     landmark_positions = [];
%     for i = 1:length(landmark_temp)
%         landmark_positions = [landmark_positions; landmark_temp{i}.X'];
%     end
%     if ~isempty(landmark_positions)
%         plot(landmark_positions(:,1),landmark_positions(:,2),'g+','Tag','landmarks','MarkerSize',5);
%     end
%     
% 	plot(ground_truth_ego_pos(1,timestep), ground_truth_ego_pos(2,timestep), "m*", "Tag","current_ground_truth_position");
% 	scatter(measurements_global{timestep}.zpos(1,:), measurements_global{timestep}.zpos(2,:), 'Marker','o','MarkerFaceColor' ,'g', 'MarkerEdgeColor','g', "Tag", "mean_cluster");
% 	scatter(measurements_global{timestep}.detections(1,:), measurements_global{timestep}.detections(2,:), 'Marker','o', 'MarkerEdgeColor','b', "Tag", "unclustered");
% % 	text(measurements_global{timestep}.zpos(1,:), measurements_global{timestep}.zpos(2,:), measurements_global{timestep}.id', "Tag", "cluster_id", "Color", "g");
%     drawnow;
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
% % Predict equations
% function [particles] = predict(particles, T, velocity, heading)
%     std_velocity = 5;
%     std_heading = deg2rad(5);
%     for p = 1:size(particles,2) % Parallel toolbox
%         random_velocity = randn(1);
%         random_heading = randn(1);
%         particles(p).X(1) = particles(p).X(1) + (T*(velocity + (random_velocity*std_velocity))*cos(heading + (random_heading*std_heading)));
%         particles(p).X(2) = (velocity + (random_velocity*std_velocity))* cos(heading + (random_heading*std_heading));
%         particles(p).X(3) = particles(p).X(3) + T*(velocity + (random_velocity*std_velocity))*sin(heading + (random_heading*std_heading));
%         particles(p).X(4) = (velocity + (random_velocity*std_velocity))* sin(heading + (random_heading*std_heading));
%     end
% end
% 
% % Update equations
% function [particles, weights] = update(particles, weights, Z) 
%     global global_point_landmark_map;
%     
%     % For already observed landmarks
%     for m = 1:size(Z.id,2)
%         if global_point_landmark_map.isKey(string(Z.id(m))) 
%             point_map_mean = [0;0];
%             point_map_var = [0;0];
%             for p = 1:size(particles,2)
%                 H = [1, 0;
%                      0, 1];
%                 R = [1, 0;
%                      0, 1];
%                 land = particles(p).local_landmark_map(string(Z.id(m)));
%                 X_pre = land.X - particles(p).X([1,3],1);
% 
%                 % Innovation
%                 Z_ = Z.zpos(1:2,m);
%                 Y = Z_(:) - H * X_pre;
%                 % Innovation covariance
%                 P_pre = land.P;
%                 S = H * P_pre * H' + R;
%                 % Kalman Gain
%                 K = P_pre * H' * S^(-1);
%                 % Update State 
%                 land.X = land.X + K * Y;
%                 % Update State Covariance
%                 land.P = P_pre - K*H*P_pre;
%                 land.P = (land.P + land.P')/2;
% 
%                 particles(p).local_landmark_map(string(Z.id(m))) = land;
%                 
%                 % Calculate weight of the particle
%                 weights(p) = weights(p) * exp(-0.5 *Y'*S^(-1)*Y)/sqrt(2*pi*det(S));
% 
%                 point_map_mean = point_map_mean + (land.X * weights(p));
%                 point_map_var = point_map_var + ((land.X - point_map_mean).^2) * weights(p);
%             end
% 
%             point_map_mean = point_map_mean / sum(weights);
%             point_map_var = point_map_var / sum(weights);
%             global_land.X = point_map_mean;
%             global_land.P = point_map_var;
%             global_point_landmark_map(string(Z.id(m))) = global_land;
%         end
%     end
% 
%     % For new landmarks
%     for m = 1:size(Z.id,2)
%         if ~global_point_landmark_map.isKey(string(Z.id(m))) 
%             point_map_mean = [0;0];
%             point_map_var = [0;0];
%             for p = 1:size(particles,2)
%                 landmark.X = particles(p).X;
%                 landmark.X = landmark.X([1,3],1) + Z.zpos(1:2,m);
%                 landmark.P = blkdiag(25,25);
%                 particles(p).local_landmark_map(string(Z.id(m))) = landmark;
%                 point_map_mean = point_map_mean + (landmark.X * weights(p));
%                 point_map_var = point_map_var + ((landmark.X - point_map_mean).^2) * weights(p); 
%             end
%             point_map_mean = point_map_mean / sum(weights);
%             point_map_var = point_map_var / sum(weights);
%             global_land.X = point_map_mean;
%             global_land.P = point_map_var;
%             global_point_landmark_map(string(Z.id(m))) = global_land;
%         end
%     end
%     
%     weights = weights/sum(weights);
% end
% 
% function Z_ = rotate_measurements_to_global_frame(Z_,orientation)
%     q = quaternion(angle2quat(orientation, 0, 0, 'ZYX'));
%     q_rotmat = rotmat(q,'point');   
%     temp = rotate_point_to_global_frame([Z_.zpos(1:2,:);zeros(1,size(Z_.zpos,2))],q_rotmat);
%     Z_.zpos(1:2,:) = temp(1:2,:);
% end
%             
% function point = translate_point_to_global_frame(point, translation_matrix)
%     point(1:3,:) = point(1:3,:) + translation_matrix;
% end
% 
% function point = rotate_point_to_global_frame(point, rotation_matrix)
%     point(1:3,:) = rotation_matrix * point(1:3,:);
% end
% 
% function [particles] = create_uniform_particles(x_min, x_max, N)
%     particles = [];
%     for i = 1:N
%         x =[(x_max(1)-x_min(1)).*rand(1)+x_min(1); 
%             (x_max(2)-x_min(2)).*rand(1)+x_min(2);
%             (x_max(3)-x_min(3)).*rand(1)+x_min(3);
%             (x_max(4)-x_min(4)).*rand(1)+x_min(4)];
%         particles = [particles, Particle(x)];
%     end
% end
% 
% function [particles] = create_gaussian_particles(mean, std, N)
%     particles = [];
%     for i = 1:N
%         x =[mean(1)+(randn(1)*std(1)); 
%             mean(2)+(randn(1)*std(2));
%             mean(3)+(randn(1)*std(3)); 
%             mean(4)+(randn(1)*std(4))];
%         particles = [particles, Particle(x)];
%     end
% end
% 
% % Effective N for resampling condition
% function [Nf] = effective_no(weights)
%     Nf = 1/sum(weights.^2); 
% end
% 
% % Resampling from indices
% function [particles, weights] = resample_from_index(particles, weights, indexes)
%     particles = particles(indexes);
%     weights = ones(1,size(weights,2)) * 1/size(weights,2);
% end
% 
% % Stratified Resampling algorithm
% function indices = stratified_resampling(weights)
%     n_samples = length(weights);
%     positions = ((0:(n_samples-1)) + rand(1,n_samples))/(n_samples);
%     indices = zeros(1,n_samples);
%     cum_sum = cumsum(weights);
%     i = 0;
%     j = 0;
%     while i < n_samples
%        if positions(i+1) < cum_sum(j+1)
%            indices(i+1) = j;
%            i = i+1;
%        else
%            j = j+1;
%        end
%     end
%     indices = indices + 1;
% end
% 
% % Systematic resampling algorithm
% function indices = systematic_resampling(weights)
%     n_samples = length(weights);
%     positions = ((0:n_samples-1) + rand(1))/(n_samples);
%     indices = zeros(1,n_samples);
%     cum_sum = cumsum(weights);
%     i = 0;
%     j = 0;
%     while i < n_samples
%        if positions(i+1) < cum_sum(j+1)
%            indices(i+1) = j;
%            i = i+1;
%        else
%            j = j+1;
%        end
%     end
%     indices = indices + 1;
% end
% 
% % Point estimate
% function [point_mean, point_var] = point_estimate(particles, weights)
%     point_mean = zeros(4,1);
%     point_var = zeros(4,1);
%     for i = 1:size(particles,2)
%         point_mean = point_mean + (particles(i).X * weights(i));
%         point_var  = point_var + ((particles(i).X - point_mean).^2 * weights(i));
%     end
%     point_mean = point_mean / sum(weights);
%     point_var = point_var / sum(weights);
% end