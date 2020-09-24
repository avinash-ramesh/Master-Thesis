%% Self Localization
clc;
close all;
clear all;
rand_num_gen = rng(100); % Initialize random number generator
scene_name = ["0655","0061","0916"];
scene_name = scene_name(3);
dead_reckoning = false;
extended_object_processing = true;
with_controls = true;
v = VideoWriter("nuscenes_implementation\videos\particle_filter_localization-extended_objects-with_controls-scene"+scene_name+".avi");
v.FrameRate = 2;
open(v);

consistency_checks = true;
filter_consistency = [];
global landmark_index_map;
landmark_index_map = containers.Map;
global global_point_landmark_map;
global_point_landmark_map = containers.Map;
global global_landmarks_map;

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
plot_scenario(ground_truth_ego_pos, global_landmarks_map); % Both these variables come from loading the workspace

%% Filter Initialization
timestep = 1;

N_particles = 1000;
weight_particles = ones(1,N_particles) * 1/N_particles;

x = ground_truth_ego_pos(1,timestep);
y = ground_truth_ego_pos(2,timestep);
vx = control_input(1,timestep) * cos(control_input(2,timestep));
vy = control_input(1,timestep) * sin(control_input(2,timestep));

% % Uniformly generate particles - may lead to filter degeneracy due to
% % sample impoverishment as a result of random spread of the particles
% x_min = [x-20, y-20, h-0.78539]; % x, y, heading
% x_max = [x+20, y+20, h+0.78539];
% particles = create_uniform_particles(x_min, x_max, N);

% Particles generated from normal distribution
% mean_state = [x, y, h]; % x, y, heading
% std_state = [20/3, 20/3, deg2rad(15)]; % position with 20m 3sig error, and heading with 45deg 3sig error.
mean_state = [x, vx, y, vy]; % x, vx, y, vy
std_state = [20/3, 5/3, 20/3, 5/3]; % position with 20m 3sig error, and heading with 45deg 3sig error.
particles_pre = create_gaussian_particles(mean_state, std_state, N_particles); % A array of Particles is created

% First Update
if dead_reckoning
    particles = particles_pre;
else
	[Z_, Z_plot] = get_current_measurement(timestep, measurements, measurements_global);
    [particles, weight_particles] = update(particles_pre, weight_particles, Z_);
    if effective_no(weight_particles) < N_particles/2
        indexes = systematic_resampling(weight_particles);
        [particles, weight_particles] = resample_from_index(particles, weight_particles, indexes);
    end
end

[filter_consistency(timestep).ground_truth, filter_consistency(timestep).X_pre, filter_consistency(timestep).X] = plot_on_global_map(timestep, particles_pre, particles, weight_particles, ground_truth_ego_pos, measurements_global, ax);
prev_time = timestamp(timestep);
T = 0;
A = getframe(gcf);
writeVideo(v, A);


%% Predict and Update steps
for timestep = 2 : size(timestamp,2)
    T = T + (timestamp(timestep) - prev_time) * 1e-6;
    [Z_, Z_plot] = get_current_measurement(timestep, measurements, measurements_global);
    
    if size(Z_.id,2) ~= 0 % Perform filtering only when measurements in the Z variable are present
        velocity = control_input(1,timestep);
        heading = control_input(2,timestep);
        particles_pre = predict(particles, T, velocity, heading);
        if dead_reckoning
            particles = particles_pre;
        else
            % Either convert measurements from local frame to global frame or convert landmarks in state vector from global to local frame
%             Z_ = rotate_measurements_to_global_frame(Z_, control_input(2,timestep)); % Rotate measurements for each particle's orientation
            [particles, weight_particles] = update(particles_pre, weight_particles, Z_); %, control_input(2,timestep)
            indexes = systematic_resampling(weight_particles);
            [particles, weight_particles] = resample_from_index(particles, weight_particles, indexes);
        end
        [filter_consistency(timestep).ground_truth, filter_consistency(timestep).X_pre, filter_consistency(timestep).X] = plot_on_global_map(timestep, particles_pre, particles, weight_particles, ground_truth_ego_pos, measurements_global, ax);
        A = getframe(gcf);
        writeVideo(v, A);

        T = 0;
    end
    prev_time = timestamp(timestep);
end
hold off;
close(v);

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
%         text(land(1), land(2), global_landmarks_map_keys{i}, "Color", "b");
    end
    
    title("Particle filter based self localization in Scene 0916","Interpreter","latex");
    xlabel("X(m) $\longrightarrow$","Interpreter","latex");
    ylabel("Y(m) $\longrightarrow$","Interpreter","latex");
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
    global global_point_landmark_map;
    legend off;
    delete(findall(ax, "Tag","current_ground_truth_position"));
	delete(findall(ax, "Tag","mean_cluster"));
	delete(findall(ax, "Tag","unclustered"));
	delete(findall(ax, "Tag","cluster_id"));
    delete(findall(ax, "Tag","landmarks"));
    delete(findall(ax,'Tag','particles_predicted'));
    delete(findall(ax,'Tag','particles'));
    
    % Prepare canvas for plotting
    for i = 1:size(particles_pre,2)
        plot(particles_pre(i).X(1), particles_pre(i).X(3),'c.','Tag','particles_predicted');
    end
    for i = 1:size(particles,2)
        plot(particles(i).X(1), particles(i).X(3),'k.','Tag','particles');
    end

    [point_mean_pre, point_var_pre] = point_estimate(particles_pre, weights);
    plot(point_mean_pre(1), point_mean_pre(3), "c*", 'Tag', "point_particles_predicted");
    
    [point_mean, point_var] = point_estimate(particles, weights);
    plot(point_mean(1), point_mean(3), "k*", 'Tag', "point_particles");
    
	plot(ground_truth_ego_pos(1,timestep), ground_truth_ego_pos(2,timestep), "m*", "Tag","current_ground_truth_position");
	ground_truth = ground_truth_ego_pos(:,timestep);
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

% Predict equations
function [particles] = predict(particles, T, velocity, heading)
    std_velocity = 2;
    std_heading = deg2rad(15);
    for p = 1:size(particles,2) % Parallel toolbox
%         random_velocity = randn(1);
%         random_heading = randn(1);
%         particles(p).X(1) = particles(p).X(1) + (T*(velocity + (random_velocity*std_velocity))*cos(heading + (random_heading*std_heading)));
%         particles(p).X(2) = particles(p).X(2) + (T*(velocity + (random_velocity*std_velocity))*sin(heading + (random_heading*std_heading)));
%         particles(p).X(3) = heading + (random_heading*std_heading);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
        trans_noise = randn(1)*std_velocity;
        rot_noise = randn(1)*std_heading;
        
        particles(p).X(1) = particles(p).X(1) + (T* particles(p).X(2)); 
        particles(p).X(2) = (velocity + trans_noise) * cos(heading + rot_noise);
        particles(p).X(3) = particles(p).X(3) + (T* particles(p).X(4));
        particles(p).X(4) = (velocity + trans_noise) * sin(heading + rot_noise);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         trans_noise = randn(1)*std_velocity;
%         rot_noise = randn(1)*std_heading;
%         
%         particles(p).X(1) = particles(p).X(1) + (T* particles(p).X(2)); 
%         particles(p).X(2) = particles(p).X(2) + (trans_noise * cos(rot_noise));
%         particles(p).X(3) = particles(p).X(3) + (T* particles(p).X(4));
%         particles(p).X(4) = particles(p).X(4) + (trans_noise * sin(rot_noise));

    
    end
end

% Update equations
function [particles, weights] = update(particles, weights, Z) 
    global global_landmarks_map;
	for p = 1:size(particles,2)
        % Working in cartesian coordinates
        Z_ = convert_measurements_to_global_frame(Z, particles(p).X(:,1)); % Rotate measurements for each particle's orientation
		for m = 1: size(Z_.id,2)
			land = global_landmarks_map(string(Z_.id(m)));
			Y = land - Z_.zpos([1,2],m);
			R = blkdiag(15,15); % (15,15) for one_per_box_0916 as well as multi_box_0916
			weights(p) = weights(p) * exp(-0.5 * Y' * R^(-1) * Y)/sqrt(2*pi*det(R));
		end
	end
	
	weights = weights + 1e-300;
    weights = weights/sum(weights);    
end

function Z_ = convert_measurements_to_global_frame(Z_,X)
    heading = atan2(X(4,1),X(2,1));
    q_rotmat = [cos(heading), -sin(heading), 0;
                sin(heading),  cos(heading), 0;
                0           ,  0           , 1];
   
    temp = q_rotmat * [Z_.zpos(1:2,:);zeros(1,size(Z_.zpos,2))];
    Z_.zpos(1:2,:) = temp(1:2,:) + X([1,3],1);
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