classdef Box
    %Box Class
    %   Simple data class representing a 3d box including, label, score and velocity.
    
    properties
        center
        wlh
        orientation
        label
        score
        velocity
        name
        token
    end
    
    methods
        function self = Box(center,size, orientation, label, score, velocity, name, token)
            %Box Construct an instance of this class
            %   Detailed explanation goes here
            assert(~any(isnan(center)));
            assert(~any(isnan(size)));
            assert(length(center)==3);
            assert(length(size)==3);
            
            if isempty(label) || isequaln(label,nan)
                self.label = nan;
            elseif ~isempty(label) && ~isequaln(label,nan)
                self.label = str2double(label);
            end
            
            if isempty(score) || isequaln(score,nan)
                self.score = nan;
            elseif ~isempty(score) && ~isequaln(score,nan)
                self.score = str2double(score);
            end
            
            if isempty(velocity) || all(isnan(velocity))
                self.velocity = [nan, nan, nan];
            elseif ~isempty(velocity) && ~isequaln(velocity,nan)
                self.velocity = str2double(velocity);
            end
            
            self.center = center;
            self.wlh = size;
            self.orientation = orientation;
            self.name = name;
            self.token = token;
        end

        function self = translate(self, x)
            self.center = self.center + x;
        end

        function self = rotate(self, q)
            q_rotmat = rotmat(q,'point');
            self.center = q_rotmat * self.center;
            self.orientation = q .* self.orientation;
            self.velocity = q_rotmat * self.velocity';
            self.velocity = self.velocity';
        end

        function record = corners(self, wlh_factor)
            if nargin < 2
                wlh_factor = 1.0;
            end
            w = self.wlh(1) * wlh_factor;
            l = self.wlh(2) * wlh_factor;
            h = self.wlh(3) * wlh_factor;

            x_corners = l / 2 .* [1,  1,  1,  1, -1, -1, -1, -1];
            y_corners = w / 2 .* [1, -1, -1,  1,  1, -1, -1,  1];
            z_corners = h / 2 .* [1,  1, -1, -1,  1,  1, -1, -1];
            
            corners = vertcat(x_corners, y_corners, z_corners);

            corners = rotmat(self.orientation,'point') * corners;

            x = self.center(1);
            y = self.center(2);
            z = self.center(3);
            corners(1, :) = corners(1, :) + x;
            corners(2, :) = corners(2, :) + y;
            corners(3, :) = corners(3, :) + z;

            record = corners;
        end
        
        function render(self,ax,view,normalize,colors,linewidth)
            if nargin < 6
                linewidth = 2;
            end
            if nargin < 5
                colors = [[0, 0, 255];[255, 0, 0];[155, 155, 155]];
            end
            if nargin < 4
                normalize = false;
            end
            if nargin < 3
                view = eye(3);
            end
            
            corners = GeometryUtils.view_points(self.corners(), view, normalize);
            corners = corners(1:2, :);

            function draw_rect(selected_corners, color)
                prev = selected_corners(end,:);
                for i = 1:length(selected_corners)
                    plot(ax, [prev(1), selected_corners(i,1)], [prev(2), selected_corners(i,2)], 'Color',color/255.0,'LineWidth',linewidth);
                    prev = selected_corners(i,:);
                end
            end
            
            for j = 1:4
                corners_tmp = corners';
                plot([corners_tmp(j,1), corners_tmp(j+4,1)],[corners_tmp(j,2), corners_tmp(j+4,2)],'Color',colors(3,:)/255.0,'LineWidth',linewidth);
            end
            draw_rect(corners_tmp(1:4,:), colors(1,:));
            draw_rect(corners_tmp(5:end,:), colors(2,:));

            center_bottom_forward = mean(corners_tmp(3:4,:),1);
            center_bottom = mean(corners_tmp([3,4,8,7],:),1);
            plot(ax,[center_bottom(1), center_bottom_forward(1)], [center_bottom(2), center_bottom_forward(2)],'Color',colors(1,:)/255.0,'LineWidth',linewidth);
        end
        
        function render_cv2(self,im,view,normalize,colors,linewidth)
            
            if nargin < 6
                linewidth = 2;
            end
            if nargin < 5
                colors = [[0, 0, 255];[255, 0, 0];[155, 155, 155]];
            end
            if nargin < 4
                normalize = false;
            end
            if nargin < 3
                view = eye(3);
            end
            
            corners = GeometryUtils.view_points(self.corners(), view, normalize);
            corners = corners(1:2, :);

            function draw_rect(ax, selected_corners, color)
                prev = selected_corners(end,:);
                for i = 1:length(selected_corners)
                    plot(ax,[prev(1), selected_corners(i,1)], [prev(2), selected_corners(i,2)],'Color',color/255.0, 'LineWidth',linewidth);
                    prev = selected_corners(i,:);
                end
            end
            ax = gca;
            for j = 1:4
                corners_tmp = corners';
                plot(ax,[corners_tmp(j,1), corners_tmp(j+4,1)],[corners_tmp(j,2), corners_tmp(j+4,2)],'Color',colors(3,:)/255.0,'LineWidth',linewidth);
            end
            draw_rect(ax, corners_tmp(1:4,:), colors(1,:));
            draw_rect(ax, corners_tmp(5:end,:), colors(2,:));

            center_bottom_forward = mean(corners_tmp(3:4,:),1);
            center_bottom = mean(corners_tmp([3,4,8,7],:),1);
            plot(ax,[center_bottom(1), center_bottom_forward(1)], [center_bottom(2), center_bottom_forward(2)],'Color',colors(1,:)/255.0,'LineWidth',linewidth);
        end
    end
end

