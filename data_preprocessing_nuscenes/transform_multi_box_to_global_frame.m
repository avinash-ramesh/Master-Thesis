clear all;
close all;
clc;
scene_name = "0061";
load("nuscenes_implementation\workspaces\egopose_landmark_map-multi_box_carlos-all_radars-default_filters-40m-"+scene_name+".mat","egopose_landmark_map");

div_x = 3;
div_y = 3;

new_egopose_landmark_map = [];
for timestep = 1: size(egopose_landmark_map,1)
	tmp_egopose_landmark_map = [];
	tmp_egopose_landmark_map = [egopose_landmark_map{timestep}(1), egopose_landmark_map{timestep}(2)];
	for box_index = 3:size(egopose_landmark_map{timestep},2)
		box = egopose_landmark_map{timestep}{box_index};
		
		l = box.info.wlh(2);
		w = box.info.wlh(1);
		h = box.info.wlh(3);
		parent_label_id = box.label;
		orientation = box.info.orientation;
		center = box.info.center;
		name = box.info.name;
		token = box.info.token;
        new_l = l/div_x;
		new_w = w/div_y;
		wlh = [new_w, new_l, h]';
		
		box_x = [1,1,0,0,1];
		box_y = [0,1,1,0,0];
		pos_x = linspace(0,l,div_x+1);
		pos_y = linspace(0,w,div_y+1);
		
		boxes = [];
		for i = 1:length(pos_x)-1
			for j = 1:length(pos_y)-1
				tmp_x = pos_x(i) + (box_x .* new_l);
				tmp_y = pos_y(j) + (box_y .* new_w);
				boxes = horzcat(boxes, [tmp_x', tmp_y']);
			end
		end

		boxes_zero = [];
		for i = 1:2:size(boxes,2)
			boxes_zero = [boxes_zero, boxes(:,i) - (l/2), boxes(:,i+1) - (w/2)];
		end


		rotation_matrix = rotmat(orientation,'point');

		boxes_rotated = [];
		for i = 1:2:size(boxes_zero,2)
			tmp_rot = rotation_matrix * [boxes_zero(:,i)';boxes_zero(:,i+1)';ones(1,5)];
			boxes_rotated = [boxes_rotated, tmp_rot(1,:)', tmp_rot(2,:)'];
		end

		boxes_translated = [];
		for i = 1:2:size(boxes_rotated,2)
			boxes_translated = [boxes_translated, boxes_rotated(:,i)+center(1), boxes_rotated(:,i+1)+center(2)];
		end
		
		count = 1;
		for i = 1:2:size(boxes_translated,2)
			polygon = polyshape(boxes_translated(:,i),boxes_translated(:,i+1));
			[center_x, center_y] = centroid(polygon);
            center = [center_x, center_y, center(3)]';
			label = string(parent_label_id)+"_"+string(count);
            tmp_cluster = struct;
            tmp_cluster.label = label;
            tmp_cluster.info = Box(center, wlh, orientation, label, NaN, NaN, name, token);
			count = count + 1;
			tmp_egopose_landmark_map = [tmp_egopose_landmark_map, {tmp_cluster}];
		end
	end
	new_egopose_landmark_map = [new_egopose_landmark_map; {tmp_egopose_landmark_map}];
end
egopose_landmark_map = new_egopose_landmark_map;
save("nuscenes_implementation\workspaces\egopose_landmark_map-multi_box_carlos_new-all_radars-default_filters-40m-"+scene_name+".mat","egopose_landmark_map");