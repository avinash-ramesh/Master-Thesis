classdef GeometryUtils
    %GeometryUtils Summary of this class goes here
    %   Detailed explanation goes here
    
    methods (Static)
        function point_cloud = view_points(points, view, normalize)
            viewpad = eye(4);
            s = size(view);
            viewpad(1:s(1), 1:s(2)) = view;

            nbr_points = size(points,2);

            % Do operation in homogenous coordinates.
            points = cat(1,points,ones(1,nbr_points));
            points = viewpad * points;
            points = points(1:3, :);

            if normalize
                points = points ./ reshape(repmat(points(3,:),3,1),3,[]);
            end
            point_cloud = points;
        end
        
        function transformed_matrix = transform_matrix(translation,rotation,inverse)
            transformed_matrix = eye(4);
            if nargin <3
                inverse = false;
            end
            if inverse
                q_rotmat = rotmat(rotation,'point');
                rot_inv = q_rotmat';
                trans = -translation';
                transformed_matrix(1:3, 1:3) = rot_inv;
                transformed_matrix(1:3, 4) = rot_inv * trans';
            else
                q_rotmat = rotmat(rotation,'point');
                transformed_matrix(1:3, 1:3) = q_rotmat;
                transformed_matrix(1:3, 4) = translation;
            end
        end
        
        function boolean_value = box_in_image(box, intrinsic, imsize, vis_level)
            corners_3d = box.corners();
            corners_img = GeometryUtils.view_points(corners_3d, intrinsic, true);
            corners_img = corners_img(1:2, :);

            visible = and(corners_img(1, :) > 0, corners_img(1, :) < imsize(1));
            visible = and(visible, corners_img(2, :) < imsize(2));
            visible = and(visible, corners_img(2, :) > 0);
            visible = and(visible, corners_3d(3, :) > 1);

            in_front = corners_3d(3, :) > 0.1;

            if vis_level == 0 % All
                boolean_value = all(visible) && all(in_front);
            elseif vis_level == 1 % Any
                boolean_value = any(visible) && all(in_front);
            elseif vis_level == 2 % None
                boolean_value = true;
            else
                assert(true, "vis_level: " + vis_level + " is not valid");
            end
        end
    end
end

