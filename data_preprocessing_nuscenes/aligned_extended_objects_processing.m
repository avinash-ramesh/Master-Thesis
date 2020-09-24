function [cluster] = aligned_extended_objects_processing(unclustered_detections, box, div_x, div_y, timestamp, parent_label_id)
    
    l = box.wlh(2);
    w = box.wlh(1);
    box_x = [1,1,0,0,1];
    box_y = [0,1,1,0,0];
    pos_x = linspace(0,l,div_x+1);
    pos_y = linspace(0,w,div_y+1);
    
    boxes = [];
    for i = 1:length(pos_x)-1
        for j = 1:length(pos_y)-1
            tmp_x = pos_x(i) + (box_x .* (l/div_x));
            tmp_y = pos_y(j) + (box_y .* (w/div_y));
            boxes = horzcat(boxes, [tmp_x', tmp_y']);
        end
    end

    boxes_zero = [];
    for i = 1:2:size(boxes,2)
        boxes_zero = [boxes_zero, boxes(:,i) - (l/2), boxes(:,i+1) - (w/2)];
    end

    orientation = box.orientation;
    center = box.center;
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
    
    cluster = [];
    count = 1;
    for i = 1:2:size(boxes_translated,2)
        tmp_cluster = struct;
        tmp_cluster.label = string(parent_label_id)+"_"+string(count);
        tmp_cluster.parent_label_id = parent_label_id;
        tmp_cluster.x = boxes_translated(:,i);
        tmp_cluster.y = boxes_translated(:,i+1);
        tmp_cluster.detections = [];
        inside_polygon = inpolygon(unclustered_detections(1,:), unclustered_detections(2,:), tmp_cluster.x', tmp_cluster.y');
        tmp_cluster.mean = mean(unclustered_detections(:,inside_polygon),2);
        tmp_cluster.detections = unclustered_detections(:,inside_polygon);
        tmp_cluster.timestamp = timestamp;
        tmp_cluster.range = sqrt((tmp_cluster.mean(1)^2) + (tmp_cluster.mean(2)^2));
        tmp_cluster.bearing = atan2(tmp_cluster.mean(2), tmp_cluster.mean(1));
        tmp_cluster.radial_velocity = mean(sqrt(tmp_cluster.detections(3,:).^2 + tmp_cluster.detections(4,:).^2));
        tmp_cluster.vx = mean(tmp_cluster.detections(3,:));
        tmp_cluster.vy = mean(tmp_cluster.detections(4,:));
        count = count + 1;
        cluster = [cluster, tmp_cluster];
    end
end