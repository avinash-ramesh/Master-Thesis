classdef RadarPointCloud
    %RadarPointCloud Summary of this class goes here
    %   Detailed explanation goes here
    properties (SetAccess = private)
        invalid_states
        dynprop_states
        ambig_states
    end
    
    methods 
        function self = RadarPointCloud()
            self.invalid_states = 0;
            self.dynprop_states = 0:6;
            self.ambig_states = 3;
        end
        
        function self = disable_filters(self)
            self.invalid_states = 0:17;
            self.dynprop_states = 0:7;
            self.ambig_states = 0:4;
        end
        
        function self = default_filters(self)
            self.invalid_states = 0;
            self.dynprop_states = 0:6;
            self.ambig_states = 3;
        end
        
        function point_cloud = remove_close(self, points, radius)
            x_filt = abs(points(1, :)) < radius;
            y_filt = abs(points(2, :)) < radius;
            not_close = not(and(x_filt, y_filt));
            point_cloud = points(:, not_close);
        end
        
        function [all_pc, all_times] = from_file_multisweep(self,nusc,sample_rec,chan,ref_chan,nsweeps,min_distance)
            
            all_pc = {};
            all_times = {};
            
            if nargin < 7
                min_distance = 1.0; 
            end
            if nargin < 6
                nsweeps = 5;
            end

            ref_sd_rec = sample_rec(ref_chan,:);
            ref_pose_rec = nusc.get_ego_pose(ref_sd_rec.ego_pose_token{1});
            ref_cs_rec = nusc.get_calibrated_sensor(ref_sd_rec.calibrated_sensor_token{1});
            ref_time = 1e-6 * ref_sd_rec.timestamp;

            ref_from_car = GeometryUtils.transform_matrix(ref_cs_rec.translation, quaternion(ref_cs_rec.rotation'),true);

            car_from_global = GeometryUtils.transform_matrix(ref_pose_rec.translation, quaternion(ref_pose_rec.rotation'),true);

            sample_data_token = sample_rec(chan,:);
            current_sd_rec = nusc.get_sample_data(sample_data_token.token{1});
            for i = 1: nsweeps
                current_pc = self.point_cloud_from_file(join([nusc.dataroot,current_sd_rec.filename],filesep));
                current_pc = self.remove_close(current_pc, min_distance);

                current_pose_rec = nusc.get_ego_pose(current_sd_rec.ego_pose_token);
                global_from_car = GeometryUtils.transform_matrix(current_pose_rec.translation,quaternion(current_pose_rec.rotation'),false);

                current_cs_rec = nusc.get_calibrated_sensor(current_sd_rec.calibrated_sensor_token);
                car_from_current = GeometryUtils.transform_matrix(current_cs_rec.translation, quaternion(current_cs_rec.rotation'), false);

                trans_matrix = (((ref_from_car * car_from_global) * global_from_car) * car_from_current);
                current_pc = self.transform(current_pc,trans_matrix);

                time_lag = ref_time - 1e-6 * current_sd_rec.timestamp;
                times = time_lag * ones(1, size(current_pc,2));
                
                all_times = [all_times times];

                all_pc = [all_pc current_pc];

                if isempty(current_sd_rec.prev)
                    break;
                else
                    current_sd_rec = nusc.get_sample_data(current_sd_rec.prev);
                end
            end
        end
        
        function point_cloud = transform(~, points, transf_matrix)
            tmp = transf_matrix * ([points(1:3, :); ones(1,size(points,2))]);
            points(1:3, :) = tmp(1:3, :);
            point_cloud = points;
        end
        
        function point_cloud = point_cloud_from_file(self,file_name,invalid_states,dynprop_states,ambig_states)
            % Loads RADAR data from a Point Cloud Data file. See details below.
            % :param file_name: The path of the pointcloud file.
            % :param invalid_states: Radar states to be kept. See details below.
            % :param dynprop_states: Radar states to be kept. Use [0, 2, 6] for moving objects only. See details below.
            % :param ambig_states: Radar states to be kept. See details below.
            % To keep all radar returns, set each state filter to range(18).
            % :return: <np.float: d, n>. Point cloud matrix with d dimensions and n points.

            % Example of the header fields:
            % # .PCD v0.7 - Point Cloud Data file format
            % VERSION 0.7
            % FIELDS x y z dyn_prop id rcs vx vy vx_comp vy_comp is_quality_valid ambig_state x_rms y_rms invalid_state pdh0 vx_rms vy_rms
            % SIZE 4 4 4 1 2 4 4 4 4 4 1 1 1 1 1 1 1 1
            % TYPE F F F I I F F F F F I I I I I I I I
            % COUNT 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1
            % WIDTH 125
            % HEIGHT 1
            % VIEWPOINT 0 0 0 1 0 0 0
            % POINTS 125
            % DATA binary

            % Below some of the fields are explained in more detail:

            % x is front, y is left

            % vx, vy are the velocities in m/s.
            % vx_comp, vy_comp are the velocities in m/s compensated by the ego motion.
            % We recommend using the compensated velocities.

            % invalid_state: state of Cluster validity state.
            % (Invalid states)
            % 0x01	invalid due to low RCS
            % 0x02	invalid due to near-field artefact
            % 0x03	invalid far range cluster because not confirmed in near range
            % 0x05	reserved
            % 0x06	invalid cluster due to high mirror probability
            % 0x07	Invalid cluster because outside sensor field of view
            % 0x0d	reserved
            % 0x0e	invalid cluster because it is a harmonics
            % (Valid states)
            % 0x00	valid
            % 0x04	valid cluster with low RCS
            % 0x08	valid cluster with azimuth correction due to elevation
            % 0x09	valid cluster with high child probability
            % 0x0a	valid cluster with high probability of being a 50 deg artefact
            % 0x0b	valid cluster but no local maximum
            % 0x0c	valid cluster with high artefact probability
            % 0x0f	valid cluster with above 95m in near range
            % 0x10	valid cluster with high multi-target probability
            % 0x11	valid cluster with suspicious angle

            % dynProp: Dynamic property of cluster to indicate if is moving or not.
            % 0: moving
            % 1: stationary
            % 2: oncoming
            % 3: stationary candidate
            % 4: unknown
            % 5: crossing stationary
            % 6: crossing moving
            % 7: stopped

            % ambig_state: State of Doppler (radial velocity) ambiguity solution.
            % 0: invalid
            % 1: ambiguous
            % 2: staggered ramp
            % 3: unambiguous
            % 4: stationary candidates

            % pdh0: False alarm probability of cluster (i.e. probability of being an artefact caused by multipath or similar).
            % 0: invalid
            % 1: <25%
            % 2: 50%
            % 3: 75%
            % 4: 90%
            % 5: 99%
            % 6: 99.9%
            % 7: <=100%
            if nargin == 2
               self = self.default_filters();
            end
           
            fid = fopen(file_name);
            assert(startsWith(fgetl(fid),"#"), "First line must be comment");
            assert(startsWith(fgetl(fid),"VERSION"), "Second line must be VERSION");
            fgetl(fid);
            sizes = fgetl(fid);
            sizes = split(strtrim(sizes)," ");
            sizes = sizes(2:end);
            types =	fgetl(fid);
            types = split(strtrim(types)," ");
            types = types(2:end);
            counts = fgetl(fid);
            counts = split(strtrim(counts)," ");
            counts = counts(2:end);
            width = fgetl(fid);
            width = split(strtrim(width)," ");
            width = str2double(width{2});
            height = fgetl(fid);
            height = split(strtrim(height)," ");
            height = str2double(height{2});
            fgetl(fid);
            fgetl(fid);
            data = fgetl(fid);
            data = split(strtrim(data)," ");
            data = data{2};
            feature_count = length(types);
            assert(width > 0, "Error: Width is incorrect");
            assert(height == 1, "Error: height != 0 not supported!");
            assert(data == "binary", "Error: Format not supported");
            unpacking_lut = containers.Map({'F2','F4','F8','I1','I2','I4','I8','U1','U2','U4','U8'},{'e','f','d','b','h','i','q','B','H','I','Q'});
            types_str = [];
            zipped = cell2mat([types, sizes]);
            for i = 1:length(zipped)
                types_str = [types_str unpacking_lut(zipped(i,:))];
            end
            offset = 0;
            point_count = width;
            points = [];
            sizes = str2double(string(cell2mat(sizes)));
            unpacking_precision = containers.Map({'e','f','d','b','h','i','q','B','H','I','Q'},{'','single','double','int8','int16','int32','int64','uint8','uint16','uint32','uint64'});
            for i = 1:point_count
                point = [];
                for p = 1: feature_count
                    start_p = offset;
                    end_p = start_p + sizes(p);
                    point_p = fread(fid,1,unpacking_precision(types_str(p)));
                    point = [point point_p];
                    offset = end_p;
                end
                points = [points; point];
            end
            if isnan(points(1))
                point_cloud = zeros(1,feature_count);
                return;
            end
            points = points';

            valid = ismember(points(size(points,1)-3,:), self.invalid_states);
            points = points(:,valid);

            valid = ismember(points(4,:), self.dynprop_states);
            points = points(:,valid);

            valid = ismember(points(12,:), self.ambig_states);
            points = points(:,valid);
            point_cloud = points;
            
            fclose(fid);
        end
    end
end

