%% Data loading
global nusc;
% scene_name = "0916";
load("nuscenes_implementation\workspaces\nusc.mat","nusc");
if ~extended_object_processing
    load("nuscenes_implementation\workspaces\egopose_landmark_map-one_box_carlos-all_radars-default_filters-40m-"+scene_name+".mat", "egopose_landmark_map");
else
    load("nuscenes_implementation\workspaces\egopose_landmark_map-multi_box_carlos-all_radars-default_filters-40m-"+scene_name+".mat", "egopose_landmark_map");
end
load("nuscenes_implementation\workspaces\sample_data_key_"+scene_name+".mat","sample_data");
load("nuscenes_implementation\workspaces\radar_"+scene_name+".mat","radar");


%% Transform the landmarks to global frame of reference
global_landmarks = containers.Map;
for i = 1: length(egopose_landmark_map)
    local_view_of_landmarks = egopose_landmark_map{i};
    ego_translation = local_view_of_landmarks{1};
    ego_rotation = local_view_of_landmarks{2};
    landmarks = {};
    for j = 3: length(local_view_of_landmarks)
        landmark_struct.label = local_view_of_landmarks{j}.label;
        landmark_struct.box = local_view_of_landmarks{j}.info;
        landmarks = [landmarks, landmark_struct]; 
    end

    for k = 1: size(landmarks,2)
        tmp = landmarks{k};
        if ~global_landmarks.isKey(tmp.label)
            q = quaternion(ego_rotation');
            q_rotmat = rotmat(q,'point');
            box_corners = tmp.box.corners();
            box_corners = rotate_point_to_global_frame(box_corners,q_rotmat);
            box_corners = translate_point_to_global_frame(box_corners,ego_translation);
            box_corners = box_corners(1:2, :);
            global_landmarks(tmp.label) = {tmp.box,box_corners};
        end
    end
end

%% Visualize the Ego car motion in Global map
landmark_labels = string(global_landmarks.keys);
moving_targets = [];
hold on;
axis normal;
for i = 1: length(landmark_labels)
    tmp = global_landmarks(string(landmark_labels(i)));
    box = tmp{1};
    box_corners = tmp{2} * 10; % Scaling factor
    [red_color, green_color, blue_color] = get_color(box.name);
    colors = [[red_color, green_color, blue_color];[red_color, green_color, blue_color];[red_color, green_color, blue_color]];
    ax = gca;
    corners_tmp = box_corners';
    for j = 1:4
        plot(ax,[corners_tmp(j,1), corners_tmp(j+4,1)],[corners_tmp(j,2), corners_tmp(j+4,2)],'Color',colors(3,:)/255.0,'LineWidth',2);
    end
    draw_rect(corners_tmp(1:4,:), colors(1,:), ax);
    draw_rect(corners_tmp(5:end,:), colors(2,:), ax);

    center_bottom_forward = mean(corners_tmp(3:4,:),1);
    center_bottom = mean(corners_tmp([3,4,8,7],:),1);
    plot(ax,[center_bottom(1), center_bottom_forward(1)], [center_bottom(2), center_bottom_forward(2)],'Color',colors(1,:)/255.0,'LineWidth',2);
%     text(ax,center_bottom(1),center_bottom(2),string(landmark_labels(i)),'FontWeight','bold','FontSize',8);
    
    annotation_tokens = nusc.get_sample_annotation(box.token).attribute_tokens;
    
    if isempty(annotation_tokens) || ~any(ismember(annotation_tokens, {'cb5118da1ab342aa947717dc53544259', 'a14936d865eb4216b396adae8cb3939c', 'ab83627ff28b465b85c427162dec722f'})) % Static Landmarks
        static_landmarks = plot(center_bottom(1), center_bottom(2), 'r.'); % Consider X axis as Y axis and vice-versa to show that the car moves forward along x axis.
    else % Moving targets
        moving_landmarks = plot(center_bottom(1), center_bottom(2), 'g.');
        disp("Moving target: "+string(landmark_labels(i)));
        moving_targets = [moving_targets, string(landmark_labels(i))];
    end
end

%% Plot ground truth
current_axis = gca;
ylim = current_axis.YLim;
xlim = current_axis.XLim;
ego_translation = [];
ego_orientation = [];
for i = 1: length(egopose_landmark_map)
    local_view_of_landmarks = egopose_landmark_map{i};
    ego_translation = [ego_translation, local_view_of_landmarks{1}];
    ego_orientation = [ego_orientation, local_view_of_landmarks{2}];
end
ground_truth_track = plot(ego_translation(1,:) * 10, ego_translation(2,:) * 10, 'b-o', 'MarkerFaceColor', 'b'); % Multiply by the scaling factor

%% Render an underlay of road with the first ego position as the center of the image
current_axis = gca;

record = innerjoin(nusc.scene,nusc.log,'LeftKeys','log_token','RightKeys','token');
record.Properties.RowNames = convertStringsToChars(record.name);
map_name = record("scene-"+scene_name,'location');
map_name = split(nusc.map{map_name.location{1},'filename'}{1}, "/");
map_name = ['1_', map_name{2}]; % Select the inverted map starting with 1_ in its name
map_location = "datasets\NuScenes\dataset_v1.0-mini\maps\"+map_name; 
render_global_map(map_location, xlim, ylim, current_axis);

%%
hold off;

%% Functions for performing transformations
function point = translate_point_to_global_frame(point, translation_matrix)
    point(1:3,:) = point(1:3,:) + translation_matrix;
end

function point = rotate_point_to_global_frame(point, rotation_matrix)
    point(1:3,:) = rotation_matrix * point(1:3,:);
end

function [red_color, green_color, blue_color] = get_color(category_name)
    if contains(category_name,"bicycle",'IgnoreCase',true) ||  contains(category_name,"motorcycle",'IgnoreCase',true)
        red_color = 255;
        green_color = 61;
        blue_color = 99;
    elseif contains(category_name,"vehicle",'IgnoreCase',true) || any(contains(category_name,["bus", "car", "construction_vehicle", "trailer", "truck"],'IgnoreCase',true))
        red_color = 255;
        green_color = 158;
        blue_color = 0;
    elseif contains(category_name,"pedestrian",'IgnoreCase',true)
        red_color = 0;
        green_color = 0;
        blue_color = 230;
    elseif contains(category_name,["cone","barrier"],'IgnoreCase',true)
        red_color = 250;
        green_color = 250;
        blue_color = 0;
    else
        red_color = 255;
        green_color = 0;
        blue_color = 255;
    end
end

function draw_rect(selected_corners, color, ax)
    prev = selected_corners(end,:);
    for i = 1:length(selected_corners)
        plot(ax, [prev(1), selected_corners(i,1)], [prev(2), selected_corners(i,2)], 'Color',color/255.0,'LineWidth',2);
        prev = selected_corners(i,:);
    end
end

function render_global_map(map_location, xlim, ylim, ax)
    map_image = imread(map_location);
    map_image_flipped = flipud(map_image);
    imshow(map_image_flipped,'Parent',ax);
    clear map_image map_image_flipped;
    current_img = ax.Children.findobj('Type','Image');
    uistack(current_img,'bottom');
    set(gca,'YDir','normal');
    ax = gca;
    axis on;
    ax.XLim = xlim;
    ax.YLim = ylim;
end