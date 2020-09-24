clear all;
close all;
clc;

%% Load NuScenes dataset
% nusc = NuScenes("v1.0-mini","datasets\NuScenes\dataset_v1.0-mini",true,0.1);
% save("nuscenes_implementation\workspaces\nusc.mat","nusc");
load("nuscenes_implementation\workspaces\nusc.mat","nusc");
scene_name = ["0655","0796","0553","0916","0061"];
scene_name = scene_name(1);
%% Sample data extraction
% sampledata = nusc.sample_data; % 31206 sample
% sample = nusc.sample;
% sample_data2 = innerjoin(sampledata, sample,'LeftKeys','sample_token','RightKeys','token'); % Get sample token information
% 
% % bebf5f5b2a674631ab5c88fd1aa9e87a - Scene-0655 (EKF SLAM)
% % c5224b9b454b4ded9b5d2d2634bbda8a - Scene-0796 (EKF SLAM)
% % 6f83169d067343658251f72e1dd17dbc - Scene-0553 (RANSAC)  
% % 325cef682f064c55a255f2625c533b75 - Scene-0916 (FastSLAM)
% % cc8c0bf57f984915a77078b10eb33198 - Scene-0061 (FastSLAM)
% 
% scene_token = {'cc8c0bf57f984915a77078b10eb33198'};
% sample_data3 = innerjoin(sample_data2, cell2table(scene_token), 'LeftKeys','scene_token','RightKeys','scene_token'); % 3136 samples in Scene 0655
% 
% ego_pose = nusc.ego_pose;
% calibrated_sensor = nusc.calibrated_sensor; 
% 
% sensor = innerjoin(sample_data3, calibrated_sensor, 'LeftKeys', 'calibrated_sensor_token', 'RightKeys', 'token');
% sample_data5 = innerjoin(sensor, ego_pose, 'LeftKeys' ,'ego_pose_token', 'RightKeys', 'token');
% 
% sensor = nusc.sensor;
% sample_data = innerjoin(sample_data5, sensor, 'LeftKeys' ,'sensor_token', 'RightKeys', 'token');
% sample_data = sortrows(sample_data,'timestamp_sampledata');
% 
% save("nuscenes_implementation\workspaces\sample_data_all_0061.mat","sample_data");
load("nuscenes_implementation\workspaces\sample_data_all_"+scene_name+".mat","sample_data");

%% Radar configuration
radar_channels = {'RADAR_FRONT_LEFT', 'RADAR_FRONT', 'RADAR_FRONT_RIGHT', 'RADAR_BACK_RIGHT', 'RADAR_BACK_LEFT'};
% radar_sensor_pose = containers.Map;
% 
% for i=1:size(sample_data,1)
%     if(radar_sensor_pose.length == length(radar_channels))
%         break;
%     elseif(radar_sensor_pose.isKey(sample_data.channel{i}))
%         continue;
%     elseif(any(ismember(sample_data.channel{i}, radar_channels)))
%         yaw = eulerd(quaternion(sample_data.rotation_sensor{i}'),'ZYX','point'); %yaw pitch roll
%         yaw = yaw(1);
%         radar_sensor_pose(sample_data.channel{i}) = [sample_data.translation_sensor{i}', yaw]; % x y z quaternion
%     end
% end
% 
% radar = cell(length(radar_channels),1);
% max_range_radars = zeros(1,length(radar_channels));
% min_azi_radars = zeros(1,length(radar_channels));
% max_azi_radars = zeros(1,length(radar_channels));
% FoV_radars = zeros(1,length(radar_channels));
% for i = 1:length(radar_channels)
%     tmp_channel = radar_channels(i);
%     tmp_sample_data = innerjoin(sample_data, cell2table(tmp_channel), 'LeftKeys' ,'channel', 'RightKeys', 'tmp_channel');
%     for j=1:size(tmp_sample_data,1)
%         datapath = tmp_sample_data.filename{j};
%         datapath  = join([nusc.dataroot,datapath],filesep);
%         pc = RadarPointCloud().point_cloud_from_file(datapath);
%         
%         if all(pc==0, 'all')
%             continue;
%         end
%         
%         range = sqrt((pc(1,:).^2) + (pc(2,:).^2));
%         max_range_radars(i) = max([max_range_radars(i),range]);
%         
%         azi = atan2d(pc(2,:),pc(1,:));
%         min_azi_radars(i) = min([min_azi_radars(i), azi]);
%         max_azi_radars(i) = max([max_azi_radars(i), azi]);
%         FoV_radars(i) = max_azi_radars(i) - min_azi_radars(i);
%     end  
%     tmp_pose = radar_sensor_pose(tmp_channel{1});
%     radar{i} = radarDetectionGenerator('SensorIndex', i, 'Height', tmp_pose(3), 'MaxRange', max_range_radars(i), ...
%         'SensorLocation', [tmp_pose(1), tmp_pose(2)], 'FieldOfView', [FoV_radars(i), 1], 'Yaw', tmp_pose(4));
% end
% save("nuscenes_implementation\workspaces\radar_0916.mat","radar");
load("nuscenes_implementation\workspaces\radar_"+scene_name+".mat","radar");

%% Configure display window for displaying camera images in chase view and bird's eye plot in top view

[BEP, hFigure, hBEVPanel] = create_demo_display(radar);
hBEVPanel.Children(2).Units = 'centimeters';
hBEVPanel_xlimits = hBEVPanel.Children(2).XLim;
hBEVPanel_ylimits = hBEVPanel.Children(2).YLim;
hBEVPanel_limits = [hBEVPanel_xlimits;hBEVPanel_ylimits];
hBEVPanel_position = hBEVPanel.Children(2).Position;
hBEVPanel_totalsize = sum(abs(hBEVPanel_limits),2);

%% Configure Video capture settings
set(gcf, 'Position', get(0, 'Screensize'));
% v = VideoWriter("nuscenes_implementation\videos\nuscenes_birds_eye_plot_multi_jorge_"+scene_name+".avi");
% v.FrameRate = 2;
% open(v);

%% Extract only key frames
%modality = {'radar'};
%radar_sample_data = innerjoin(sample_data, cell2table(modality), 'LeftKeys' ,'modality', 'RightKeys', 'modality');
%radar_sample_data = sortrows(radar_sample_data,'timestamp_sampledata');
% sample_data = sample_data(sample_data.is_key_frame==1,:);
% sample_data = sortrows(sample_data, 'timestamp_sampledata');
% save("nuscenes_implementation\workspaces\sample_data_key_0655.mat","sample_data");
load("nuscenes_implementation\workspaces\sample_data_key_"+scene_name+".mat","sample_data");


%% Clear Memory
% clear pc sample_data2 tmp_sample_data sample_data3 sample_data5 sampledata...
%     scene_token sampledata sample_data5 sample_data3 sample_data2 sample...
%     ego_pose calibrated_sensor radar_sensor_pose yaw radar_sensor_pose...
%     max_azi_radars FoV_radars datapath azi max_range_radars min_azi_radars...
%     pc radar range tmp_channel tmp_pose tmp_sample_data
	
%% Visualization
detections = containers.Map;
object_annotations = containers.Map;
clustered_detections = containers.Map;
total_objects = 1;
detections_ = {};
egopose_landmark_map = {};
hTopViewPanel = uipanel(hFigure, 'Position', [0 0 0.5 1], 'Title', 'Camera View');
t = tiledlayout(hTopViewPanel,3,2);
t.TileSpacing = 'none';
t.Padding = 'none';

%% Rendering
for i=1:size(sample_data,1)
    if sample_data.channel{i} == "CAM_FRONT_LEFT"
        hCarPlot = nexttile(t, 1);    
        nusc.render_sample_data(sample_data.token{i},true,1,40,hCarPlot,1,[],false,false);
    elseif sample_data.channel{i} == "CAM_FRONT"
        hCarPlot = nexttile(t, 3);
        nusc.render_sample_data(sample_data.token{i},true,1,40,hCarPlot,1,[],false,false);
    elseif sample_data.channel{i} == "CAM_FRONT_RIGHT"
        hCarPlot = nexttile(t, 2);
        nusc.render_sample_data(sample_data.token{i},true,1,40,hCarPlot,1,[],false,false);
    elseif sample_data.channel{i} == "CAM_BACK_LEFT"
        hCarPlot = nexttile(t, 5);    
        nusc.render_sample_data(sample_data.token{i},true,1,40,hCarPlot,1,[],false,false);
    elseif sample_data.channel{i} == "CAM_BACK"
        hCarPlot = nexttile(t, 4);    
        nusc.render_sample_data(sample_data.token{i},true,1,40,hCarPlot,1,[],false,false);
    elseif sample_data.channel{i} == "CAM_BACK_RIGHT"
        hCarPlot = nexttile(t, 6);    
        nusc.render_sample_data(sample_data.token{i},true,1,40,hCarPlot,1,[],false,false);
    elseif sample_data.modality{i} == "radar"
		datapath = sample_data.filename{i};
        datapath  = join([nusc.dataroot,datapath],filesep);
        rpc = RadarPointCloud();
%         rpc = rpc.disable_filters(); % To keep all radar returns
        pc = rpc.point_cloud_from_file(datapath);

        if all(pc==0,'all') || isempty(pc)
            continue;
        end
        
        q = quaternion(sample_data.rotation_sensor{i}');
        q_rotmat = rotmat(q,'point');
        pc = PointCloud.rotate(pc,q_rotmat);
        pc = PointCloud.translate(pc,sample_data.translation_sensor{i});

        % 7,8 corresponds to vx and vy which are uncompensated velocities
        % Use 9,10 for compensated vx and vy
        sensorDets = [pc([1,2,3,9,10],:); zeros(1,size(pc,2))];

        detections(sample_data.channel{i}) = 1;
        for j=1:size(sensorDets,2)
            det_ = objectDetection(sample_data.timestamp_sampledata(i), sensorDets(:,j));
            det_.MeasurementNoise = zeros('like',det_.MeasurementNoise);
            detections_ = [detections_; {det_}];
        end

        if length(detections) == length(radar_channels)
            unclustered_detections = [];
            for p = 1:size(detections_,1)
                unclustered_detections = [unclustered_detections, detections_{p}.Measurement([1,2,4,5])]; % x, y, vx, vy
            end
                    
            sd_record = nusc.detail_sample_data(sample_data.token{i});
            sample_rec = nusc.detail_sample(sd_record.sample_token);
            chan = sd_record.channel;
            ref_chan = 'LIDAR_TOP';
            ref_sd_token = sample_rec(ref_chan,:);
            ref_sd_record = nusc.detail_sample_data(ref_sd_token.token{1});
            
            current_axis_BEP = BEP.Parent;
            axes(current_axis_BEP);
            hold on;
            axis normal;
            delete(findall(hBEVPanel,'type','annotation'));
            delete(current_axis_BEP.Children.findobj('Tag',''));
            nusc.render_ego_centric_map(sample_data.token{i}, 40, current_axis_BEP);
            current_img = current_axis_BEP.Children.findobj('Type','Image');
            uistack(current_img,'bottom');
            
            % Show ego vehicle
            plot(current_axis_BEP, 0, 0, 'rx');
            
            % Get boxes
            [data_path, boxes, camera_intrinsic] = nusc.get_sample_data_with_annotation(ref_sd_token.token{1}, 1,[], true);
            
            tmp_egopose_landmark = [sample_data.translation_ego_pose(i), sample_data.rotation_ego_pose(i)];

            for j = 1: size(boxes,2)
                [red_color, green_color, blue_color] = nusc.get_color(boxes(j).name);
                boxes(j).render(current_axis_BEP, eye(4),[],[[red_color, green_color, blue_color];[red_color, green_color, blue_color];[red_color, green_color, blue_color]]);
                
%                 selected_radar = radar_index_map(string(sample_data.channel{i}));                
%                 range = sqrt(((boxes(j).center(1)-selected_radar.SensorLocation(1))^2) + ((boxes(j).center(2)-selected_radar.SensorLocation(2))^2));
%                 azimuth = atan2d((boxes(j).center(2)-selected_radar.SensorLocation(2)),(boxes(j).center(1)-selected_radar.SensorLocation(1)));
%                 max_range = selected_radar.MaxRange;
%                 azi_max = selected_radar.Yaw + selected_radar.FieldOfView(1)/2;
%                 azi_min = selected_radar.Yaw - selected_radar.FieldOfView(1)/2;
%                 if range >= 0 && range <= max_range && azimuth >= azi_min && azimuth <= azi_max  % Limit in range and azimuth of X direction of Panel

                if all(boxes(j).center(1:2) >= hBEVPanel_limits(:,1)) && all(boxes(j).center(1:2) <= hBEVPanel_limits(:,2))
                    axis normal;
                    a = annotation(hBEVPanel,'textbox');
                    a.Units = 'centimeters';
                    
                    tmp_sample_ann = nusc.sample_annotation(nusc.sample_annotation.token == string(boxes(j).token),:);
                    
                    if ~object_annotations.isKey(string(tmp_sample_ann.instance_token))
                        object_annotations(string(tmp_sample_ann.instance_token)) = {total_objects, boxes(j)};
                        total_objects = total_objects + 1;
                    else
                        object_label = object_annotations(string(tmp_sample_ann.instance_token));
                        object_annotations(string(tmp_sample_ann.instance_token)) = {object_label{1}, boxes(j)};
                    end
                    
                    tmp_string = object_annotations(string(tmp_sample_ann.instance_token));
                    a.String = tmp_string{1};
                    a.HorizontalAlignment = 'center';
                    a.VerticalAlignment = 'middle';
                    a.FontWeight = 'bold';
                    a.FontSize = 8;
                    a.EdgeColor = 'none';
                    if boxes(j).center(2) < 0 % X Position of annotation
                        x_pos = hBEVPanel_position(1)+ ((abs(hBEVPanel_limits(2,2))+abs(boxes(j).center(2)))*hBEVPanel_position(3)/hBEVPanel_totalsize(1))-(a.Position(3)/2);
                    else
                        x_pos = hBEVPanel_position(1)+ ((abs(hBEVPanel_limits(2,2))-abs(boxes(j).center(2)))*hBEVPanel_position(3)/hBEVPanel_totalsize(1))-(a.Position(3)/2);
                    end
                    if boxes(j).center(1) >= 0 % Y Position of annotation
                        y_pos = hBEVPanel_position(2)+ ((abs(hBEVPanel_limits(1,1))+abs(boxes(j).center(1)))*hBEVPanel_position(4)/hBEVPanel_totalsize(2))-(a.Position(4)/2);
                    else
                        y_pos = hBEVPanel_position(2)+ ((abs(hBEVPanel_limits(1,1))-abs(boxes(j).center(1)))*hBEVPanel_position(4)/hBEVPanel_totalsize(2))-(a.Position(4)/2);
                    end
                    a.Position(1:2) = [x_pos,y_pos];
                    
                    %#################### Static point object landmark (x,y) generation #####
                    landmark = struct;
                    landmark.label = a.String;
                    landmark.info = boxes(j);
                    tmp_egopose_landmark = [tmp_egopose_landmark, landmark];
                    
                    %#################### Cluster Detections ##########################
                    boxes_corners = boxes(j).corners();
                    boxes_corners = boxes_corners(1:2,:);
                    
%                     % Multiple detections per box - Aligned
%                     div_x = 3;
%                     div_y = 3;
%                     cluster = aligned_extended_objects_processing(unclustered_detections,boxes(j),div_x,div_y,sample_data.timestamp_sample(i),a.String);
%                     for c_i = 1:length(cluster)
%                         if ~isempty(cluster(c_i).detections)
%                             if clustered_detections.isKey(cluster(c_i).label)
%                                 clustered_detections(cluster(c_i).label) = [clustered_detections(cluster(c_i).label), cluster(c_i)];
%                             else
%                                 clustered_detections(cluster(c_i).label) = cluster(c_i);
%                             end
%                             plot(current_axis_BEP, cluster(c_i).mean(1),cluster(c_i).mean(2), 'Marker','o','MarkerFaceColor','g','MarkerEdgeColor','none');  
%                         end
%                     end
                    
                    % Multiple detections per box - Jorge                   
%                     box_corners.x = boxes_corners(1,[3,4,8,7]);%Polygon x-coordinates
%                     box_corners.y=  boxes_corners(2,[3,4,8,7]);%Polygon y-coordinates
%                     NX = 3;
%                     NY = 3;
%                     clusters = extended_objects_processing(unclustered_detections',box_corners,NX,NY,sample_data.timestamp_sample(i),double(string(a.String)));
%                     
%                     for c_i = 1:numel(clusters)
%                         if not(isempty(clusters{c_i})) && clusters{c_i}.mean_exists
%                             if clustered_detections.isKey(clusters{c_i}.Global_Label)
%                                 clustered_detections(clusters{c_i}.Global_Label) = [clustered_detections(clusters{c_i}.Global_Label), clusters{c_i}];
%                             else
%                                 clustered_detections(clusters{c_i}.Global_Label) = clusters{c_i};
%                             end
%                             plot(current_axis_BEP, clusters{c_i}.representative_mean(1,1),clusters{c_i}.representative_mean(2,1), 'Marker','o','MarkerFaceColor','g','MarkerEdgeColor','none');
%                         end
%                     end
                    
                    % One detection per box 
                    inside_polygon = inpolygon(unclustered_detections(1,:), unclustered_detections(2,:), boxes_corners(1,[3,4,8,7]), boxes_corners(2,[3,4,8,7]));

                    cluster.mean = mean(unclustered_detections(:,inside_polygon),2);
                    cluster.detections = unclustered_detections(:,inside_polygon);
                    cluster.timestamp = sample_data.timestamp_sample(i);
                    cluster.range = sqrt((cluster.mean(1)^2) + (cluster.mean(2)^2));
                    cluster.bearing = atan2(cluster.mean(2), cluster.mean(1));
                    cluster.radial_velocity = mean(sqrt(cluster.detections(3,:).^2 + cluster.detections(4,:).^2));
                    cluster.vx = mean(cluster.detections(3,:));
                    cluster.vy = mean(cluster.detections(4,:));
                    if ~isempty(cluster.detections)
                        if clustered_detections.isKey(string(a.String))
                            clustered_detections(string(a.String)) = [clustered_detections(string(a.String)), cluster];
                        else
                            clustered_detections(string(a.String)) = cluster;
                        end

                        plot(current_axis_BEP, cluster.mean(1),cluster.mean(2), 'Marker','o','MarkerFaceColor','g','MarkerEdgeColor','none');  
                    end
                    %##################################################################
                end
            end
            %##############################
            
            % Update bird's-eye plot
            updateBEP(BEP, detections_);
            hold off;
%             A = getframe(gcf);
%             writeVideo(v, A);
            detections = containers.Map;
            detections_ = {};
            egopose_landmark_map = [egopose_landmark_map; {tmp_egopose_landmark}];
        end
    end
end
% close(v);
% 
% save("nuscenes_implementation\workspaces\egopose_landmark_map-one_box_carlos-all_radars-default_filters-40m-"+scene_name+".mat","egopose_landmark_map");
% save("nuscenes_implementation\workspaces\clustered_detections-one_box_carlos-all_radars-default_filters-40m-"+scene_name+".mat","clustered_detections");


%% Helper functions
function updateBEP(BEP, detections)
    % Prepare and update detections display
    N = numel(detections);
    detPos = zeros(N,2);
    isRadar = true(N,1);
    for i = 1:N
        detPos(i,:) = detections{i}.Measurement(1:2)';
        if detections{i}.SensorIndex > 6 % Vision detections
            isRadar(i) = false;
        end
    end
    plotDetection(findPlotter(BEP,'DisplayName','radar'), detPos(isRadar,:));
end