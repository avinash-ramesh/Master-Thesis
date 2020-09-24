function [segmented_inliers, segmented_outliers, separating_line] = ransac1d(point_cloud,niterations,disttol)
    finalized_inliers = [];
    
    for i=1:niterations
        choosen_point = randi(size(point_cloud.x,2));
        inliers = [choosen_point];
%         while size(inliers,2)<2
%             choosen_point = randi(size(point_cloud.x,2));
%             inliers = unique([inliers, choosen_point]);
%         end 
        
        x1 = 1;
        y1 = point_cloud.y(inliers(1));
        x2 = max(point_cloud.x);
        y2 = point_cloud.y(inliers(1));
        
%         A = y1-y2;
%         B = x2-x1;
%         C = (x1*y2)-(x2*y1);
        
        for j=1:size(point_cloud.x,2)
            if any(ismember(j,inliers))
               continue; 
            end
            x3 = point_cloud.x(j);
            y3 = point_cloud.y(j);
%             cal_distance = abs((A*x3)+(B*y3)+C)/sqrt((A^2)+(B^2));
            cal_distance = abs(y3 - y2);
            if cal_distance <= disttol
                inliers = unique([inliers, j]);
            end
        end
        
        if size(finalized_inliers,2) < size(inliers,2)
            finalized_inliers = inliers;
            separating_line.x = [x1 x2];
            separating_line.y = [y1 y2];
        end
    end
    segmented_inliers.x = point_cloud.x(finalized_inliers);
    segmented_inliers.y = point_cloud.y(finalized_inliers);
    
    segmented_outliers.x = [];
    segmented_outliers.y = [];
    for k=1:size(point_cloud.x,2)
        if ~any(ismember(k,finalized_inliers))
            segmented_outliers.x = [segmented_outliers.x point_cloud.x(k)];
            segmented_outliers.y = [segmented_outliers.y point_cloud.y(k)];
        end
    end
end