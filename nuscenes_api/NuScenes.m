classdef NuScenes
    %NuScenes
    %   Database class for nuScenes to help query and retrieve information from the database.
    
    properties (SetAccess = private)
        version
        dataroot
        verbose
        map_resolution
        table_names
        category
        attribute
        visibility
        instance
        sensor
        calibrated_sensor
        ego_pose
        log
        scene
        sample
        sample_data
        sample_annotation
        map
        image_annotations
        map_mask
        
        get_table
        get_category
        get_attribute
        get_visibility
        get_instance
        get_sensor
        get_calibrated_sensor
        get_ego_pose
        get_log
        get_scene
        get_sample
        get_sample_data
        get_sample_annotation
        get_map
    end
    
    methods
        function self = NuScenes(version, dataroot, verbose, map_resolution)
            %NuScenes Construct an instance of this class
            %   Loads database and creates reverse indexes and shortcuts.
            %   version: Version to load (e.g. "v1.0", ...).
        	%   dataroot: Path to the tables and data.
            %   verbose: Whether to print status messages during load.
            %   map_resolution: Resolution of maps (meters).
            
            self.version = version;
            self.dataroot = dataroot;
            self.verbose = verbose;
            self.map_resolution = map_resolution;
            self.table_names = {'category', 'attribute', 'visibility', 'instance', 'sensor',...
                'calibrated_sensor', 'ego_pose', 'log', 'scene', 'sample', 'sample_data', 'sample_annotation', 'map'};
            
            self.category = self.load_table("category");
            self.get_category = self.create_map(self.category);
            self.category = struct2table(self.category);
            self.category.Properties.RowNames = convertStringsToChars(string(1:height(self.category)));
            
            self.attribute = self.load_table("attribute");
            self.get_attribute = self.create_map(self.attribute);
            self.attribute = struct2table(self.attribute);
            self.attribute.Properties.RowNames = convertStringsToChars(string(1:height(self.attribute)));
            
            self.visibility = self.load_table("visibility");
            self.get_visibility = self.create_map(self.visibility);
            self.visibility = struct2table(self.visibility);
            self.visibility.Properties.RowNames = convertStringsToChars(string(1:height(self.visibility)));
            
            self.instance = self.load_table("instance");
            self.get_instance = self.create_map(self.instance);
            self.instance = struct2table(self.instance);
            self.instance.Properties.RowNames = convertStringsToChars(string(1:height(self.instance)));
            
            self.sensor = self.load_table("sensor");
            self.get_sensor = self.create_map(self.sensor);
            self.sensor = struct2table(self.sensor);
            self.sensor.Properties.RowNames = convertStringsToChars(string(1:height(self.sensor)));
            
            self.calibrated_sensor = self.load_table("calibrated_sensor");
            self.get_calibrated_sensor = self.create_map(self.calibrated_sensor);
            self.calibrated_sensor = struct2table(self.calibrated_sensor);
            self.calibrated_sensor.Properties.RowNames = convertStringsToChars(string(1:height(self.calibrated_sensor)));
            
            self.ego_pose = self.load_table("ego_pose");
            self.get_ego_pose = self.create_map(self.ego_pose);
            self.ego_pose = struct2table(self.ego_pose);
            self.ego_pose.Properties.RowNames = convertStringsToChars(string(1:height(self.ego_pose)));
            
            self.log = self.load_table("log");
            self.get_log = self.create_map(self.log);
            self.log = struct2table(self.log);
            self.log.Properties.RowNames = convertStringsToChars(string(1:height(self.log)));
            
            self.scene = self.load_table("scene");
            self.get_scene = self.create_map(self.scene);
            self.scene = struct2table(self.scene);
            self.scene.Properties.RowNames = convertStringsToChars(string(1:height(self.scene)));
            
            self.sample = self.load_table("sample");
            self.get_sample = self.create_map(self.sample);
            self.sample = struct2table(self.sample);
            self.sample.Properties.RowNames = convertStringsToChars(string(1:height(self.sample)));
            
            self.sample_data = self.load_table("sample_data");
            self.get_sample_data = self.create_map(self.sample_data);
            self.sample_data = struct2table(self.sample_data);
            self.sample_data.Properties.RowNames = convertStringsToChars(string(1:height(self.sample_data)));
            
            self.sample_annotation = self.load_table("sample_annotation");
            self.get_sample_annotation = self.create_map(self.sample_annotation);
            self.sample_annotation = struct2table(self.sample_annotation);
            self.sample_annotation.Properties.RowNames = convertStringsToChars(string(1:height(self.sample_annotation)));
            
            self.map = self.load_table("map");
            self.get_map = self.create_map(self.map);
            self.map = struct2table(self.map);
            self.map.Properties.RowNames = {'singapore-onenorth','boston-seaport','singapore-queenstown','singapore-hollandvillage'};
            
            tables_values = {self.get_category, self.get_attribute, self.get_visibility, self.get_instance,...
                self.get_sensor, self.get_calibrated_sensor, self.get_ego_pose, self.get_log, self.get_scene,...
                self.get_sample, self.get_sample_data, self.get_sample_annotation, self.get_map};
            
            self.get_table = containers.Map(self.table_names,tables_values);
            if self.verbose == true
                fprintf("======\nLoading NuScenes tables for version %s...\r\n",self.version);
                for table_id = 1: size(self.table_names,2)
                    fprintf("%s is loaded.\r\n",self.table_names{table_id});
                end
            end
        end
        
        function record = detail_scene(self)
            record = join(self.scene,self.log,'LeftKeys','log_token','RightKeys','token');
            record = record(:,{'name','description','vehicle','nbr_samples','date_captured','location'});
            record.Properties.RowNames = convertStringsToChars(string(1:height(record)));
        end
        
        function record = detail_sample(self, sample_token)
            record = innerjoin(innerjoin(self.list_sample_data_in_sample(sample_token),...
                self.calibrated_sensor,'LeftKeys','calibrated_sensor_token','RightKeys','token'),...
                self.sensor,'LeftKeys','sensor_token','RightKeys','token');
            record = record(record.is_key_frame==1,:);
            record.Properties.RowNames = record.channel;
        end
        
        function record = detail_sample_data(self, sample_data_token)
            record = innerjoin(innerjoin(struct2table(self.get_sample_data(sample_data_token),'AsArray',true),...
                self.calibrated_sensor,'LeftKeys','calibrated_sensor_token','RightKeys','token'),...
                self.sensor,'LeftKeys','sensor_token','RightKeys','token'); 
        end
        
        function record = list_samples_in_scene(self,scene_token)
            record = innerjoin(self.sample,cell2table(scene_token),'LeftKeys','scene_token','RightKeys','scene_token');
            record.Properties.RowNames = convertStringsToChars(string(1:height(record)));
        end
        
        function record = list_sample_data_in_sample(self,sample_token)
            record = innerjoin(self.sample_data,cell2table(sample_token),'LeftKeys','sample_token','RightKeys','sample_token');
            record.Properties.RowNames = convertStringsToChars(string(1:height(record)));
        end
        
        function [points, coloring, im] = map_pointcloud_to_image(self, point_sensor, camera_sensor, render_intensity,  min_dist)
            if nargin < 5 || isempty(min_dist)
                min_dist = 1.0;
            end
            if nargin < 4 || isempty(render_intensity)
                render_intensity = false;
            end
            
            
            point_sensor_data_path = point_sensor.filename{1};
            point_sensor_data_path = join([self.dataroot,point_sensor_data_path],filesep);
            calibrated_point_sensor = self.get_calibrated_sensor(point_sensor.calibrated_sensor_token{1});
            point_sensor_info = self.get_sensor(calibrated_point_sensor.sensor_token);
            
            if point_sensor_info.modality == "lidar"
               pc = LidarPointCloud().point_cloud_from_file(point_sensor_data_path); 
            elseif point_sensor_info.modality == "radar"
               pc = RadarPointCloud().point_cloud_from_file(point_sensor_data_path);  
            end
            
            camera_sensor_data_path = camera_sensor.filename{1};
            camera_sensor_data_path = join([self.dataroot,camera_sensor_data_path],filesep);
            im = imread(camera_sensor_data_path);
            % Points live in the point sensor frame. So they need to be transformed via global to the image plane.
            % First step: transform the point-cloud to the ego vehicle frame for the timestamp of the sweep.
            q = quaternion(calibrated_point_sensor.rotation');
            q_rotmat = rotmat(q,'point');
            pc = PointCloud.rotate(pc,q_rotmat);
            pc = PointCloud.translate(pc,calibrated_point_sensor.translation);
            
            % Second step: transform to the global frame.
            ego_pose_point_sensor = self.get_ego_pose(point_sensor.ego_pose_token{1});
            q = quaternion(ego_pose_point_sensor.rotation');
            q_rotmat = rotmat(q,'point');
            pc = PointCloud.rotate(pc,q_rotmat);
            pc = PointCloud.translate(pc,ego_pose_point_sensor.translation);

            % Third step: transform into the ego vehicle frame for the timestamp of the image.
            ego_pose_camera_sensor = self.get_ego_pose(camera_sensor.ego_pose_token{1});
            pc = PointCloud.translate(pc,-ego_pose_camera_sensor.translation);
            q = quaternion(ego_pose_camera_sensor.rotation');
            q_rotmat = rotmat(q,'point');
            pc = PointCloud.rotate(pc,q_rotmat');
            
            % Fourth step: transform into the camera.
            calibrated_camera_sensor = self.get_calibrated_sensor(camera_sensor.calibrated_sensor_token{1});
            pc = PointCloud.translate(pc,-calibrated_camera_sensor.translation);
            q = quaternion(calibrated_camera_sensor.rotation');
            q_rotmat = rotmat(q,'point');
            pc = PointCloud.rotate(pc,q_rotmat');

            % Fifth step: actually take a "picture" of the point cloud.
            depths = pc(3, :);
            
            if point_sensor_info.modality == "lidar" && render_intensity
                assert(point_sensor_info.modality == "lidar", 'Error: Can only render intensity for lidar!');
                intensities = pc(4, :);
                intensities = (intensities - min(intensities))/((max(intensities) - min(intensities)));
                intensities = intensities .^ 0.1;
                intensities = max(0, (intensities - 0.5));
                coloring = intensities;
            elseif point_sensor_info.modality == "radar" && render_intensity
                intensities = pc(6, :);
                intensities = (intensities - min(intensities))/((max(intensities) - min(intensities)));
                intensities = intensities .^ 0.1;
                intensities = max(0, (intensities - 0.5));
                coloring = intensities;
            else
                coloring = depths;
            end
            
            pc = GeometryUtils.view_points(pc(1:3, :), calibrated_camera_sensor.camera_intrinsic, true);
            
            mask = true(1,size(depths,2));
            mask = and(mask, depths > min_dist);
            mask = and(mask, pc(1, :) > 1);
            mask = and(mask, pc(1, :) < size(im,2) - 1);
            mask = and(mask, pc(2, :) > 1);
            mask = and(mask, pc(2, :) < size(im,1) - 1);

            points = pc(:, mask);
            coloring = coloring(mask);        
        end
     
        function render_pointcloud_in_image(self, sample_token, dot_size, point_sensor_channel, camera_channel, render_intensity, out_path, ax)
            if nargin < 8 || isempty(ax)
                ax = nexttile;
            end
            if nargin < 7 || isempty(out_path)
                out_path = '';
            end
            if nargin < 6 || isempty(render_intensity)
                render_intensity = false;
            end
            if nargin < 5 || isempty(camera_channel)
                camera_channel = 'CAM_FRONT';
            end
            if nargin < 4 || isempty(point_sensor_channel)
                point_sensor_channel = 'LIDAR_TOP';
            end
            if nargin < 3 || isempty(dot_size)
                dot_size = 5;
            end
            
            sample_data1 = self.detail_sample(sample_token); % sample_data table always has information about the sensors for a sample in a scene.
            point_sensor = sample_data1(point_sensor_channel,:);
            camera_sensor = sample_data1(camera_channel,:);

            [points, coloring, im] = self.map_pointcloud_to_image(point_sensor, camera_sensor, render_intensity);
             
            imshow(im,'Parent',ax);
            hold on;
            scatter(ax,points(1, :), points(2, :), dot_size, coloring, 'filled');
            axis('off');
            hold off;
            
            if out_path 
               imwrite(im, out_path); 
            end
        end
        
        function [radar_data, nonradar_data] = render_sample(self,token,box_vis_level,nsweeps,out_path)
            if nargin < 5
                out_path = [];
            end
            if nargin < 4
                nsweeps = 1;
            end
            if nargin < 3
                box_vis_level = 1;
            end
          
            record = self.detail_sample(token);
            radar_data = containers.Map;
            nonradar_data = containers.Map;
            for i = 1:height(record)
                if any(ismember({'lidar', 'camera'},record.modality{i}))
                    nonradar_data(record.channel{i}) = record.token{i};
                else
                    radar_data(record.channel{i}) = record.token{i};
                end
            end
            
            if ~isempty(radar_data) 
                num_radar_plots = 1; 
            else
                num_radar_plots = 0;
            end
            n = num_radar_plots + length(nonradar_data);
            cols = 3;
            tiledlayout(ceil(n/cols),3,'TileSpacing','none','Padding','none');
            % Plot radars into a single subplot.
            if ~isempty(radar_data)
                hold on;
                ax = nexttile;
                ax.XAxis.Visible = 'off';
                ax.YAxis.Visible = 'off';
                
                for i = 1:length(radar_data)
                    map_val = radar_data.values;
                    disp(map_val{i});
                    show_anns = (i==1);
                    self.render_sample_data(map_val{i}, show_anns, box_vis_level,40, ax, nsweeps,[],show_anns,show_anns);
                end
                ax.Title.String = 'Fused RADARs';
                hold off;
            end
            % Plot camera and lidar in separate subplots.
            for i = 1:length(nonradar_data)
                ax = nexttile;
                ax.XAxis.Visible = 'off';
                ax.YAxis.Visible = 'off';
                hold on;
                map_val = nonradar_data.values;
                self.render_sample_data(map_val{i}, true, box_vis_level,40, ax, nsweeps,[],true,true);
                hold off;
            end
            
            % Write to disk.
            if ~isempty(out_path)
                plt.savefig(out_path)
            end
        end
        
        function render_sample_data(self,sample_data_token,with_anns,box_vis_level,axes_limit,ax,nsweeps,...
                out_path,underlay_map,use_flat_vehicle_coordinates)
            
            if nargin < 10 || isempty(use_flat_vehicle_coordinates)
                use_flat_vehicle_coordinates = true;
            end
            if nargin < 9 || isempty(underlay_map)
                underlay_map = true;
            end
            if nargin < 8 || isempty(out_path)
                out_path = '';
            end
            if nargin < 7 || isempty(nsweeps)
                nsweeps = 1;
            end
            if nargin < 6 || isempty(ax)
                ax = '';
            end
            if nargin < 5 || isempty(axes_limit)
                axes_limit = 40;
            end
            if nargin < 4 || isempty(box_vis_level)
               box_vis_level = 1; %BoxVisibility.ANY,
            end
            if nargin < 3 || isempty(with_anns)
                with_anns = true;
            end
            
            % Get Sensor Modality
            sd_record = self.detail_sample_data(sample_data_token);
            if any(ismember({'lidar','radar'},sd_record.modality))
                sample_rec = self.detail_sample(sd_record.sample_token);
                chan = sd_record.channel;
                ref_chan = 'LIDAR_TOP';
                ref_sd_token = sample_rec(ref_chan,:);
                ref_sd_record = self.detail_sample_data(ref_sd_token.token{1});
            
                if sd_record.modality{1} == "lidar"
                    [pc, times] = LidarPointCloud().from_file_multisweep(self, sample_rec, chan, ref_chan, nsweeps);
                    velocities = [];
                else
                    [pc, times] = RadarPointCloud().from_file_multisweep(self, sample_rec, chan, ref_chan, nsweeps);
                    velocities = pc{1}(9:10, :);  % Compensated velocity
                    velocities = vertcat(velocities, zeros(1,size(pc{1},2)));
                    
                    q = quaternion(sd_record.rotation{1}');
                    q_rotmat = rotmat(q,'point');
                    velocities = PointCloud.rotate(velocities,q_rotmat);
                    
                    q = quaternion(ref_sd_record.rotation{1}');
                    q_rotmat = rotmat(q,'point');
                    velocities = PointCloud.rotate(velocities,q_rotmat');
                    velocities(3, :) = zeros(1,size(pc{1},2));
                end
                
                if use_flat_vehicle_coordinates
                    ref_to_ego = GeometryUtils.transform_matrix(ref_sd_record.translation{1},quaternion(ref_sd_record.rotation{1}'));
                    pose_record = self.get_ego_pose(ref_sd_record.ego_pose_token{1});
                    
                    ego_yaw = euler(quaternion(pose_record.rotation'),'ZYX','point'); %yaw_pitch_roll
                    ego_yaw = ego_yaw(1);
                    
                    rotation_vehicle_flat_from_vehicle = rotmat(quaternion(cos(ego_yaw / 2), 0, 0, sin(ego_yaw / 2)),'point') * rotmat(quaternion(pose_record.rotation')','point');
                    vehicle_flat_from_vehicle = eye(4);
                    vehicle_flat_from_vehicle(1:3, 1:3) = rotation_vehicle_flat_from_vehicle;
                    viewpoint = vehicle_flat_from_vehicle * ref_to_ego;
                else
                    viewpoint = eye(4);
                end

                % Init axes.
                if isempty(ax)
                    tiledlayout(1,1,'TileSpacing','compact','Padding','compact');
                    ax = nexttile;
                    hold on;
                    %ax.XAxis.Visible = 'off';
                    %ax.YAxis.Visible = 'off';                
                end

                if underlay_map
                    assert(use_flat_vehicle_coordinates, "Error: underlay_map requires use_flat_vehicle_coordinates, otherwise the location does not correspond to the map!");
                    hold on;
                    self.render_ego_centric_map(sample_data_token, axes_limit, ax);
                end

                points = GeometryUtils.view_points(pc{1}(1:3,:), viewpoint, false);
                %dists = sqrt(sum(pc{1}(1:2,:).^2, 1));
                dists = sqrt((points(1,:).^2) + (points(2,:).^2));
                color_map = parula(size(dists,2));
                colors = [];
                min_colors = min(dists);
                max_colors = axes_limit;
                for s = 1: size(dists,2)
                    indx = (min(dists(s),max_colors) - min_colors)/(max_colors - min_colors);
                    indx = max(1,round(indx * size(dists,2)));
                    colors = [colors;color_map(indx,:)];
                end
                
                %colors = min(1, dists ./ axes_limit ./ sqrt(2));
                if sd_record.modality == "lidar"
                    point_scale = 0.5;
                else
                    point_scale = 1;
                end
                
                scatter(ax,points(1,:),points(2,:), point_scale,colors);
                
                
                if sd_record.modality == "radar"
                    points_vel = GeometryUtils.view_points(pc{1}(1:3, :) + velocities, viewpoint, false);
                    deltas_vel = points_vel - points;
                    deltas_vel = 6 * deltas_vel; % Arbitrary scaling
                    max_delta = 20;
                    deltas_vel = min(max_delta,max(-max_delta,deltas_vel));
                    colors_rgba = colors; %scatter.to_rgba(colors)
                    for i=1:size(points,2)
                       hold on;
                       %x1 = points(1,i)/(max(points(1,:))-min(points(1,:)));
                       %x2 = (points(1,i)+deltas_vel(1,i))/(max(points(1,:))-min(points(1,:)));
                       %y1 = points(2,i)/(max(points(2,:))-min(points(2,:)));
                       %y2 = (points(2,i)+deltas_vel(2,i))/(max(points(1,:))-min(points(1,:)));
                       %annotation('arrow',[x1, x2], [y1, y2]);
                       %annotation('arrow',[points(1,i), points(1,i)+deltas_vel(1,i)],[points(2,i), points(2,i)+deltas_vel(2,i)]);
                       plot([points(1,i), points(1,i)+deltas_vel(1,i)],[points(2,i), points(2,i)+deltas_vel(2,i)],'-k.');
                       plot(points(1,i)+deltas_vel(1,i),points(2,i)+deltas_vel(2,i),'ko');
                    end
                end
                % Show ego vehicle
                plot(ax, 0, 0, 'rx');

                % Get boxes
                [data_path, boxes, camera_intrinsic] = self.get_sample_data_with_annotation(ref_sd_token.token{1}, box_vis_level,[], use_flat_vehicle_coordinates);
                
                if with_anns
                    for i = 1: size(boxes,2)
                        [red_color, green_color, blue_color] = self.get_color(boxes(i).name);
                        hold on;
                        boxes(i).render(ax, eye(4),[],[[red_color, green_color, blue_color];[red_color, green_color, blue_color];[red_color, green_color, blue_color]]);
                    end
                end
                
                ax.XLim = [-axes_limit, axes_limit];
                ax.YLim = [-axes_limit, axes_limit];
            
            elseif sd_record.modality == "camera"
                [data_path, boxes, camera_intrinsic] = self.get_sample_data_with_annotation(sample_data_token, box_vis_level);
                
                data = imread(data_path);
                if nargin < 5
                    ax = nexttile;
%                     hold on;
%                     ax.XAxis.Visible = 'off';
%                     ax.YAxis.Visible = 'off';                
                end
                %hold on;
                imshow(data,'Parent',ax);
                
                if nargin>= 2 && with_anns
                    for i = 1: size(boxes,2)
                        [red_color, green_color, blue_color] = self.get_color(boxes(i).name);
                        hold on;
                        boxes(i).render(ax, camera_intrinsic,true, [[red_color, green_color, blue_color];[red_color, green_color, blue_color];[red_color, green_color, blue_color]]);
                    end
                end
                
                ax.XLim = [0, size(data,2)];
                ax.YLim = [0, size(data,1)];
                %RI = imref2d(size(ego_centric_map));
                %RI.XWorldLimits = [-axes_limit axes_limit];
                %RI.YWorldLimits = [-axes_limit axes_limit];
                %hold on;
                %imshow(ego_centric_map, RI,'Parent',ax);
            else
                assert(true,"Error: Unknown sensor modality!");
            end
            
            ax.Parent.Children(1).Title.Interpreter = 'None';
            ax.Parent.Children(1).Title.String = sd_record.channel{1};
            
            if nargin>=7 && ~isempty(out_path)
                saveas(gcf,out_path);
            end
        end
        
        function box = get_box(self, sample_annotation_token)
            record = self.get_sample_annotation(sample_annotation_token);
            record = innerjoin(struct2table(record, 'AsArray', true),self.instance, 'LeftKeys','instance_token','RightKeys','token');
            record = innerjoin(record,self.category, 'LeftKeys','category_token','RightKeys','token');
            box = Box(record.translation{1}, record.size{1}, quaternion(record.rotation{1}'),[],[],[], record.name{1}, record.token{1});
        end
        
        function record = get_annotations_for_sample(self, sample_token)
            record = innerjoin(self.sample(self.sample.token == string(sample_token),:), self.sample_annotation, 'LeftKeys','token','RightKeys','sample_token');
            record.Properties.RowNames = convertStringsToChars(string(1:height(record)));
        end
        
        function record = get_instance_for_annotation(self, sample_token)
            record = innerjoin(self.sample(self.sample.token == string(sample_token),:), self.sample_annotation, 'LeftKeys','token','RightKeys','sample_token');
            record.Properties.RowNames = convertStringsToChars(string(1:height(record)));
        end
        
        function boxes = get_boxes(self, sample_data_token)
            sd_record = self.get_sample_data(sample_data_token);
            curr_sample_record = self.get_annotations_for_sample(sd_record.sample_token);
            boxes = {};
            if isempty(curr_sample_record.prev_left{1}) || sd_record.is_key_frame
                for i = 1:size(curr_sample_record,1)
                    boxes = [boxes self.get_box(curr_sample_record.token_right{i})];
                end
            else
                prev_sample_record = self.get_annotations_for_sample(curr_sample_record.prev_left{1});
                curr_ann_recs = {};
                prev_ann_recs = {};
                for i = 1: length(curr_sample_record.token_right)
                    curr_ann_recs = [curr_ann_recs self.get_sample_annotation(curr_sample_record.token_right{i})];
                end
                for i = 1: length(prev_sample_record.token_right)
                    prev_ann_recs = [prev_ann_recs self.get_sample_annotation(prev_sample_record.token_right{i})];
                end
                
                prev_inst_map = containers.Map;
                for i = 1: length(prev_ann_recs)
                    prev_inst_map(prev_ann_recs{i}.instance_token) = prev_ann_recs{i};
                end

                t0 = prev_sample_record.timestamp(1);
                t1 = curr_sample_record.timestamp(1);
                t = sd_record.timestamp;

                t = max(t0, min(t1, t));

                boxes = {};
                for i = 1:length(curr_ann_recs)
                    if ismember(prev_inst_map.keys,curr_ann_recs{i}.instance_token)
                        prev_ann_rec = prev_inst_map(curr_ann_recs{i}.instance_token);
                        center = {};
                        zipped = cell2mat([prev_ann_rec.translation,curr_ann_recs{i}.translation]);
                        for j = 1:length(zipped)
                            center = [center interp1([t0, t1], [zipped{1}, zipped{2}], t)];
                        end

                        rotation = quatinterp(quaternion(prev_ann_rec.rotation'), quaternion(curr_ann_recs{i}.rotation'),(t-t0)/(t1 - t0));

                        box = Box(center, curr_ann_recs{i}.size, rotation, curr_ann_recs{i}.category_name, curr_ann_recs{i}.token);
                    else
                        box = self.get_box(curr_ann_recs{i}.token);
                    end
                    boxes = [boxes box];
                end
            end
        end
        
        function record = get_sample_data_path(self, sample_data_token)
            sd_record = self.get_sample_data(sample_data_token);
            record = join([self.dataroot, sd_record.filename],filesep);
        end

        function [data_path, box_list, camera_intrinsic] = get_sample_data_with_annotation(self, sample_data_token, box_vis_level,...
                selected_anntokens, use_flat_vehicle_coordinates)
            
            if nargin < 5
                use_flat_vehicle_coordinates = false;
            end
            if nargin < 4
                selected_anntokens = [];
            end
            if nargin < 3
                box_vis_level = 1; % Box visibility - All
            end
            
            sd_record = self.get_sample_data(sample_data_token);
            cs_record = self.get_calibrated_sensor(sd_record.calibrated_sensor_token);
            sensor_record = self.get_sensor(cs_record.sensor_token);
            pose_record = self.get_ego_pose(sd_record.ego_pose_token);

            data_path = self.get_sample_data_path(sample_data_token);

            if sensor_record.modality == "camera"
                camera_intrinsic = cs_record.camera_intrinsic;
                imsize = [sd_record.width, sd_record.height];
            else
                camera_intrinsic = [];
                imsize = [];
            end
            
            if ~isempty(selected_anntokens)
                boxes = {};
                for i = 1:length(selected_anntokens)
                    boxes = [boxes self.get_box(selected_anntokens(i))]; %%% Write a loop in get_box to handle a list of annotation tokens.
                end
            else
                boxes = self.get_boxes(sample_data_token);
            end
            
            box_list = {};
            for k = 1:length(boxes) %box in boxes:
                if use_flat_vehicle_coordinates
                    yaw = euler(quaternion(pose_record.rotation'),'ZYX','point'); %yaw_pitch_roll
                    yaw = yaw(1);
                    boxes(k) = boxes(k).translate(-pose_record.translation);
                    boxes(k) = boxes(k).rotate(quaternion(cos(yaw/2), 0, 0, sin(yaw/2))'); % Use ' instead of quatinv for quaternion inverse.
                else
                    boxes(k) = boxes(k).translate(-pose_record.translation);
                    boxes(k) = boxes(k).rotate(quaternion(pose_record.rotation')');

                    boxes(k) = boxes(k).translate(-cs_record.translation);
                    boxes(k) = boxes(k).rotate(quaternion(cs_record.rotation')');
                end
                
                if sensor_record.modality == "camera" && ~GeometryUtils.box_in_image(boxes(k), camera_intrinsic, imsize, box_vis_level)
                    
                else
                    box_list = [box_list boxes(k)];
                end
            end
        end
        
        function render_scene_channel(self,scene_token,channel,freq,imsize,out_path)
            if nargin < 6 || isempty(out_path)
                out_path = '';
            end
            if nargin < 5 || isempty(imsize)
               imsize = [640, 360]; 
            end
            if nargin < 4 || isempty(freq)
                freq = 10;
            end
            if nargin < 3 || isempty(channel)
                channel='CAM_FRONT';
            end

            valid_channels = {'CAM_FRONT_LEFT', 'CAM_FRONT', 'CAM_FRONT_RIGHT','CAM_BACK_LEFT', 'CAM_BACK', 'CAM_BACK_RIGHT'};

            assert((imsize(1) / imsize(2)) == (16 / 9), "Aspect ratio should be 16/9.");
            assert(any(ismember(channel,valid_channels)), ['Input channel ',channel,' not valid.']);

            if ~isempty(out_path)
                splitted_text = split(out_path,".");
                assert(all(splitted_text{end} == 'avi'),"The output path should be AVI format");
            end
            
            scene_rec = self.get_scene(scene_token);
            sample_rec = self.get_sample(scene_rec.first_sample_token);
            sd_rec = self.detail_sample({sample_rec.token});
            sd_rec = sd_rec(channel,:);
            sd_rec = self.get_sample_data(sd_rec.token{1});

            if ~isempty(out_path)
                v = VideoWriter(out_path,'Motion JPEG AVI');
                v.FrameRate = freq;
                open(v);
            else
                v = '';
            end

            has_more_frames = true;
            figure;
            while has_more_frames
                [impath, boxes, camera_intrinsic] = self.get_sample_data_with_annotation(sd_rec.token,1);
                im = imread(impath);
                hold on;
                imshow(im);
                for i = 1:size(boxes,2)
                    [r,g,b] = self.get_color(boxes(i).name);
                    boxes(i).render_cv2(im, camera_intrinsic, true, [[r,g,b]; [r,g,b]; [r,g,b]]);
                end
                
                ax = gca;
                ax.XLim = [0,size(im,2)];
                ax.YLim = [0,size(im,1)];
                hold off;
                drawnow;
                
                if ~isempty(out_path)
                    frame = getframe(gcf);
                    writeVideo(v,frame);
                end

                if ~isempty(sd_rec.next)
                    sd_rec = self.get_sample_data(sd_rec.next);
                else
                    has_more_frames = false;
                end
            end
            
            if ~isempty(out_path)
                close(v);
            end
        end
        
        function render_scene_channel_with_point_cloud(self,scene_token,channel,point_sensor_channel,freq,imsize,out_path,render_intensity)
            if nargin < 8 || isempty(render_intensity)
                render_intensity = false;
            end
            if nargin < 7 || isempty(out_path)
                out_path = '';
            end
            if nargin < 6 || isempty(imsize)
               imsize = [640, 360]; 
            end
            if nargin < 5 || isempty(freq)
                freq = 10;
            end
            if nargin < 4 || isempty(point_sensor_channel)
                point_sensor_channel='RADAR_FRONT';
            end
            if nargin < 3 || isempty(channel)
                channel='CAM_FRONT';
            end

            valid_channels = {'CAM_FRONT_LEFT', 'CAM_FRONT', 'CAM_FRONT_RIGHT','CAM_BACK_LEFT', 'CAM_BACK', 'CAM_BACK_RIGHT'};

            assert((imsize(1) / imsize(2)) == (16 / 9), "Aspect ratio should be 16/9.");
            assert(any(ismember(channel,valid_channels)), ['Input channel ',channel,' not valid.']);

            if ~isempty(out_path)
                splitted_text = split(out_path,".");
                assert(all(splitted_text{end} == 'avi'),"The output path should be AVI format");
            end
            
            scene_rec = self.get_scene(scene_token);
            sample_rec = self.get_sample(scene_rec.first_sample_token);
            sd_rec = self.detail_sample({sample_rec.token});
            sd_rec = sd_rec(channel,:);
            sd_rec = self.get_sample_data(sd_rec.token{1});

            if ~isempty(out_path)
                v = VideoWriter(out_path,'Motion JPEG AVI');
                v.FrameRate = freq;
                open(v);
            else
                v = '';
            end

            has_more_frames = true;
            figure;
            while has_more_frames
                [impath, boxes, camera_intrinsic] = self.get_sample_data_with_annotation(sd_rec.token,1);
                im = imread(impath);
                hold on;
                imshow(im);
                for i = 1:size(boxes,2)
                    [r,g,b] = self.get_color(boxes(i).name);
                    boxes(i).render_cv2(im, camera_intrinsic, true, [[r,g,b]; [r,g,b]; [r,g,b]]);
                end
                
                ax = gca;
                ax.XLim = [0,size(im,2)];
                ax.YLim = [0,size(im,1)];
                
                if sd_rec.is_key_frame
                    sensor_data = self.detail_sample({sample_rec.token});
                    point_sensor = sensor_data(point_sensor_channel,:); 
                    camera_sensor = sensor_data(channel,:);
                    [points, coloring, im1] = self.map_pointcloud_to_image(point_sensor, camera_sensor, render_intensity);
                    scatter(ax,points(1, :), points(2, :), 5, coloring, 'filled');
                    axis('off');
                end
                hold off;
                drawnow;
                
                if ~isempty(out_path)
                    frame = getframe(gcf);
                    writeVideo(v,frame);
                end

                if ~isempty(sd_rec.next)
                    sd_rec = self.get_sample_data(sd_rec.next);
                else
                    has_more_frames = false;
                end
            end
            
            if ~isempty(out_path)
                close(v);
            end
        end
            
        function render_ego_centric_map(self,sample_data_token,axes_limit,ax)

            function cropped_image = crop_image(image, x_px, y_px, axes_limit_px)
                x_min = int64(x_px - axes_limit_px);
                x_max = int64(x_px + axes_limit_px);
                y_min = int64(y_px - axes_limit_px);
                y_max = int64(y_px + axes_limit_px);

                cropped_image = image(y_min+1:y_max, x_min+1:x_max);
            end

            sd_record = self.get_sample_data(sample_data_token);
            sample = self.get_sample(sd_record.sample_token);
            scene = self.get_scene(sample.scene_token);
            log = self.get_log(scene.log_token);
            map_ = self.map(log.location,:);
            map_mask = MapMask(join([self.dataroot,map_.filename{1}],filesep),self.map_resolution);
            pose = self.get_ego_pose(sd_record.ego_pose_token);

            [px_coord, py_coord] = map_mask.to_pixel_coords(pose.translation(1), pose.translation(2));
            scaled_limit_px = int64(axes_limit * (1.0 / map_mask.resolution));
            mask_raster = map_mask.mask();
            cropped = crop_image(mask_raster, px_coord, py_coord, int64(scaled_limit_px * sqrt(2)));

            
            ypr_deg = eulerd(quaternion(pose.rotation'),'ZYX','point'); %yaw_pitch_roll
            ypr_deg = -ypr_deg(1);
            rotated_cropped = imrotate(cropped,ypr_deg,'nearest','crop');

            ego_centric_map = crop_image(rotated_cropped, size(rotated_cropped,2)/2, size(rotated_cropped,1)/2, scaled_limit_px);

            if nargin < 4 || isempty(ax)
                ax = nexttile;
            end
            
            ego_centric_map(ego_centric_map == map_mask.foreground) = 125;
            ego_centric_map(ego_centric_map == map_mask.background) = 255;
            RI = imref2d(size(ego_centric_map));
            RI.XWorldLimits = [-axes_limit axes_limit];
            RI.YWorldLimits = [-axes_limit axes_limit];
            ego_centric_map = flipud(ego_centric_map);
            imshow(ego_centric_map, RI,'Parent',ax);
            current_img = ax.Children.findobj('Type','Image');
            uistack(current_img,'bottom');
            set(gca,'YDir','normal');
        end
        
        function [red_color, green_color, blue_color] = get_color(self,category_name)
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
    end
    methods (Access = private)   
        function outputArg = load_table(self, table_name)
            %load_table Loads the data from its corresponding JSON file
            %   table_name specifies the name of JSON file
            outputArg = jsondecode(fileread(join([self.dataroot,self.version,table_name+".json"],filesep)));
        end
        
        function record = create_map(~, table)
            %create_map Returns a record from table as a map.
            %   table: Table name from which a map is to be created.
            %   Returns a map of Table.
            names = {table.token};
            values = {ones(1,length(names))};
            for index = 1: length(names)
                values{index} = table(index);
            end
            record = containers.Map(names,values);
        end
    end
end