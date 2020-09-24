close all;
clear all;
clc;

workspace_name = "clustered_detections-one_box_carlos_ransac-all_radars-default_filters-40m-0655.mat";
load("nuscenes_implementation/workspaces/"+workspace_name,"clustered_detections");

keys = clustered_detections.keys;
simplified_cluster_detections = containers.Map;

% % Multi box - Aligned
% for i = 1:length(keys)
%     object_track = clustered_detections(keys{i});
%     for j = 1:length(object_track)
%         cluster.label = keys{i};
%         cluster.mean = object_track(j).mean; 
%         cluster.detections = object_track(j).detections;
%         cluster.range = object_track(j).range;
%         cluster.bearing = object_track(j).bearing;
%         cluster.radial_velocity = object_track(j).radial_velocity;
%         cluster.parent_label_id = object_track(j).parent_label_id;
%         cluster.vx = object_track(j).vx;
%         cluster.vy = object_track(j).vy;
%         if ~simplified_cluster_detections.isKey(string(object_track(j).timestamp))
%             simplified_cluster_detections(string(object_track(j).timestamp)) = cluster;
%         else
%             simplified_cluster_detections(string(object_track(j).timestamp)) = [simplified_cluster_detections(string(object_track(j).timestamp)), cluster];
%         end
%     end
% end

% % Multi box - Jorge
% for i = 1:length(keys)
%     object_track = clustered_detections(keys{i});
%     for j = 1:length(object_track)
%         cluster.label = keys{i};
%         cluster.mean = object_track(j).representative_mean; 
%         cluster.detections = object_track(j).detections;
%         cluster.range = object_track(j).range;
%         cluster.bearing = object_track(j).bearing;
%         cluster.radial_velocity = object_track(j).radial_velocity;
%         cluster.parent_label_id = object_track(j).parent_label_id;
%         cluster.vx = object_track(j).vx;
%         cluster.vy = object_track(j).vy;
%         if ~simplified_cluster_detections.isKey(string(object_track(j).timestamp))
%             simplified_cluster_detections(string(object_track(j).timestamp)) = cluster;
%         else
%             simplified_cluster_detections(string(object_track(j).timestamp)) = [simplified_cluster_detections(string(object_track(j).timestamp)), cluster];
%         end
%     end
% end

% One box    
for i = 1:length(keys)
    object_track = clustered_detections(keys{i});
    for j = 1:length(object_track)
        cluster.label = keys{i};
        cluster.mean = object_track(j).mean;
        cluster.detections = object_track(j).detections;
        cluster.range = object_track(j).range;
        cluster.bearing = object_track(j).bearing;
        cluster.radial_velocity = object_track(j).radial_velocity;
        cluster.vx = object_track(j).vx;
        cluster.vy = object_track(j).vy;
        if ~simplified_cluster_detections.isKey(string(object_track(j).timestamp))
            simplified_cluster_detections(string(object_track(j).timestamp)) = cluster;
        else
            simplified_cluster_detections(string(object_track(j).timestamp)) = [simplified_cluster_detections(string(object_track(j).timestamp)), cluster];
        end
    end
end

save("nuscenes_implementation\workspaces\simplified_"+workspace_name,"simplified_cluster_detections");