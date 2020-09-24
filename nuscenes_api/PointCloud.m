classdef PointCloud
    methods (Static)
        function point_cloud = translate(point_cloud, translation_matrix)
            point_cloud(1:3,:) = point_cloud(1:3,:) + translation_matrix;
        end
        
        function point_cloud = rotate(point_cloud, rotation_matrix)
            point_cloud(1:3,:) = rotation_matrix * point_cloud(1:3,:);
        end
    end
end

