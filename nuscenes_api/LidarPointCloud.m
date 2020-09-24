classdef LidarPointCloud
    %LidarPointCloud Summary of this class goes here
    %   Detailed explanation goes here
    methods
        function point_cloud = point_cloud_from_file(~,full_path_to_file)
            file_id = fopen(full_path_to_file, 'r');
            point_cloud = fread(file_id,'single');
            fclose(file_id);
            point_cloud = reshape(point_cloud, 5, []);
            point_cloud = point_cloud(1:4,:);
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
    end
end

