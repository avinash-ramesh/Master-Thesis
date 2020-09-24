function [PXY] = extended_objects_processing(radar_detections,car_polygon,NX,NY,timestamp,parent_label_id)
%20200713 - Jorge Centenera - Fraunhofer FHR

%This function divides a car into several subareas.
%It is created to see how many radar detections fall inside a subpart of a
%car, representing the car as a bounding box (=Polygon).

%%INPUTS
% 	+ radar_detections	:  	"radar_detections x-coordinates" and "radar_detections y-coordinates"
%                           Matrix of Nx2 (column 1 is x, column 2 is y), N is
%                           the number of radar detections, and the number of rows.
% 	+ car_polygon       :  	"car_polygon x-coordinates" and "car_polygon y-coordinates"
%                           a structure consisting of car_polygon.x (vector of 
%                           x-coordinates) and car_polygon.y (vector of y-coordinates).
%                           Both subfields are row vectors.
% 	+ NX                :   Number of divisions in x direction
%                           scalar
% 	+ NY                :   NY is the Number of divisions in y direction
%                           scalar
% 	+ timestamp:            associated timestamp for the processing
% 	+ parent_label_id:      id of the car inside a dictionary of cars in the scenario starting with 1 to M, where M is the total of landmark's cars

%%OUTPUTS
% 	+ PXY	:  	a cell array where PX{i,j}.x and PX{i,j} are vectors of x 
%               and y coordinates of new polygon in (i,j) grid position
%    			cell array

    X=radar_detections(:,1);Y=radar_detections(:,2);
    %Call function from Ayad Al-Rumaithi (Divice Polygon - Mathworks)
    PXY = DIVIDEXY(car_polygon,NX,NY);
    
    %% Processing Part
    subpolygon_counter=1;
    for i=1:NX
        for j=1:NY
            if not(isempty(PXY{i,j}))
                PXY{i,j}.subpolygon_counter=subpolygon_counter;%Store the counter of subpolygon number
                PXY{i,j}.Area_Label=sprintf('%s %d','Area',PXY{i,j}.subpolygon_counter);%Store Area Label
                PXY{i,j}.sizeX=size(PXY{i,j}.x);%Store size of X
                PXY{i,j}.sizeY=size(PXY{i,j}.y);%Store size of Y
                %Calculate IN and ON points based on RADAR DETECTIONS X and Y
                [PXY{i,j}.IN PXY{i,j}.ON] = inpolygon(X,Y,PXY{i,j}.x,PXY{i,j}.y);
                %PXY{i,j}.NOT_IN= ~PXY{i,j}.IN;PXY{i,j}.NOT_ON= ~PXY{i,j}.ON;
                PXY{i,j}.edge_points=numel(X(PXY{i,j}.ON));%Store the number of points lying on the edge of the polygon area
                PXY{i,j}.out_points=numel(X(~PXY{i,j}.IN));%Store the number of points lying outside of the polygon area
                PXY{i,j}.in_points=numel(X(PXY{i,j}.IN));%Store the number of points lying inside of the polygon area                         
                PXY{i,j}.i=i;PXY{i,j}.j=j;%Store i and j
                if PXY{i,j}.in_points>0 | PXY{i,j}.edge_points>0
                    PXY{i,j}.mean_exists=1;%Store a flag indicating there is a representative (calculated as the mean)
                    %Store the mean representative
                    PXY{i,j}.representative_mean=mean([X(PXY{i,j}.IN),Y(PXY{i,j}.IN);X(PXY{i,j}.ON),Y(PXY{i,j}.ON);],1)';
                    PXY{i,j}.range = sqrt((PXY{i,j}.representative_mean(1)^2) + (PXY{i,j}.representative_mean(2)^2));
                    PXY{i,j}.bearing = atan2(PXY{i,j}.representative_mean(2), PXY{i,j}.representative_mean(1));
					
                    % These are just written here to save memory
                    [xcentroid,ycentroid] = centroid(polyshape(PXY{i,j}.x,PXY{i,j}.y));
                    PXY{i,j}.centroid=[xcentroid,ycentroid];%Store polygon or subarea centroid
                    PXY{i,j}.timestamp=timestamp;
                    PXY{i,j}.parent_label_id=parent_label_id;
                    PXY{i,j}.Global_Label=sprintf('%s %d - %s %d','Object',PXY{i,j}.parent_label_id,'Subarea',PXY{i,j}.subpolygon_counter);%Store Global Label
                    PXY{i,j}.detections = radar_detections(PXY{i,j}.IN,:)';
					PXY{i,j}.radial_velocity = mean(sqrt(PXY{i,j}.detections(3,:).^2 + PXY{i,j}.detections(4,:).^2));
                    PXY{i,j}.vx = mean(PXY{i,j}.detections(3,:));
                    PXY{i,j}.vy = mean(PXY{i,j}.detections(4,:));
                else
                    PXY{i,j}.mean_exists=0;%Store a flag indicating there is NOT any representative
                end
            end
            %Increase subpolygons counter:
            subpolygon_counter=subpolygon_counter+1;
        end
    end
end




function PXY=DIVIDEXY(polygon,NX,NY)
%Downloaded from MATHWORKS, licensed by Ayad Al-Rumaithi
%Divide polygon into smaller polygons set by grid.

%Input
%   +   polygon: a structure consist of polygon.x (vector of x-coordinates) and polygon.y (vector of y-coordinates) 
%   +   NX: Number of divisions in x direction
%   +   NY: Number of divisions in y direction
%Output
%   +   PXY: a cell array where PX{i,j}.x and PX{i,j} are vectors of x and y coordinates of new polygon in (i,j) grid position
    DX=(max(polygon.x)-min(polygon.x))/NX;
    DY=(max(polygon.y)-min(polygon.y))/NY;
    i=0;
    P=polygon;
    for X=min(polygon.x)+DX:DX:max(polygon.x)-DX
        i=i+1;
        [PX{i}, P]=DIVIDEX(P,X);
    end
    PX{NX}=P;
    for i=1:1:NX
        j=0;
        for Y=min(polygon.y)+DY:DY:max(polygon.y)-DY
            j=j+1;
            [PXY{i,j}, PX{i}]=DIVIDEY(PX{i},Y);     
        end
        PXY{i,NY}=PX{i};
    end
end


function [polygon1 polygon2]=DIVIDEX(polygon,X)
    polygon1=[];
    polygon2=[];
    if not(isempty(polygon))
        m=length(polygon.x);
        c1=0;
        c2=0;
        for i=1:m
            j=i+1;
            if i==m
                j=1;
            end 
            if polygon.x(i)<=X
                c1=c1+1;
                polygon1.x(c1)=polygon.x(i);
                polygon1.y(c1)=polygon.y(i);
            end
            if polygon.x(i)>=X
                c2=c2+1;
                polygon2.x(c2)=polygon.x(i);
                polygon2.y(c2)=polygon.y(i);  
            end
            if (polygon.x(i)>X && polygon.x(j)<X) || (polygon.x(i)<X && polygon.x(j)>X) 
                c1=c1+1;
                polygon1.x(c1)=X;
                polygon1.y(c1)=polygon.y(j)+(polygon.y(i)-polygon.y(j))/(polygon.x(i)-polygon.x(j))*(X-polygon.x(j));    
                c2=c2+1;
                polygon2.x(c2)=X;
                polygon2.y(c2)=polygon.y(j)+(polygon.y(i)-polygon.y(j))/(polygon.x(i)-polygon.x(j))*(X-polygon.x(j));           
            end
        end
    end
end


function [polygon1 polygon2]=DIVIDEY(polygon,Y)
    polygon1=[];
    polygon2=[];
    if not(isempty(polygon))
        m=length(polygon.y);
        c1=0;
        c2=0;
        for i=1:1:m
            j=i+1;
            if i==m
                j=1;
            end 
            if polygon.y(i)<=Y
                c1=c1+1;
                polygon1.x(c1)=polygon.x(i);
                polygon1.y(c1)=polygon.y(i);
            end
            if polygon.y(i)>=Y
                c2=c2+1;
                polygon2.x(c2)=polygon.x(i);
                polygon2.y(c2)=polygon.y(i);  
            end

            if (polygon.y(i)>Y && polygon.y(j)<Y) || (polygon.y(i)<Y && polygon.y(j)>Y) 
                c1=c1+1;
                polygon1.x(c1)=polygon.x(j)+(polygon.x(i)-polygon.x(j))/(polygon.y(i)-polygon.y(j))*(Y-polygon.y(j));    
                polygon1.y(c1)=Y;
                c2=c2+1;
                polygon2.x(c2)=polygon.x(j)+(polygon.x(i)-polygon.x(j))/(polygon.y(i)-polygon.y(j))*(Y-polygon.y(j));           
                polygon2.y(c2)=Y;
            end
        end
    end
end






























%%
% %Ayad Al-Rumaithi (2020). Divide Polygon (https://www.mathworks.com/matlabcentral/fileexchange/71635-divide-polygon), MATLAB Central File Exchange. Retrieved July 13, 2020.
% 
% function PXY = dividePolygon(timestamp,unclustered_detections,box_corners,box_label,NX,NY)
%     %Modified on 20200708 - Jorge Centenera - Fraunhofer FHR
%     %This function divides a Polygon.
%     %It is created to see how many radar detections fall inside a subpart of a
%     %car, representing the car as a bounding box.
% 
%     %%INPUTS
%     % 	+ polygon	:  	"Polygon x-coordinates" and "Polygon y-coordinates"
%     %                   a structure consisting of polygon.x (vector of 
%     %                   x-coordinates) and polygon.y (vector of y-coordinates).
%     %                   Both subfields are row vectors.
%     % 	+ NX        :   Number of divisions in x direction
%     % 					scalar
%     % 	+ NY        :   NY is the Number of divisions in y direction
%     % 					scalar
%     % 	+ draw      : 	1 if drawing is desired, 0 if not
%     % 					boolean
% 
%     %%OUTPUTS
%     % 	+ PXY	:  	a cell array where PX{i,j}.x and PX{i,j} are vectors of x 
%     %               and y coordinates of new polygon in (i,j) grid position
%     %    			cell array
%     
%     if box_label =="8"
%         disp("8"); %Iteration 4 is error
%     end
%     
%     X=unclustered_detections(1,:);
%     Y=unclustered_detections(2,:);
% 
%     polygon.x= box_corners(1,[3,4,8,7]);%Polygon x-coordinates
%     polygon.y= box_corners(2,[3,4,8,7]);%Polygon y-coordinates
%     
%     PXY = DIVIDEXY(polygon,NX,NY);
%     
%     subpolygon_counter=1;
%     for i=1:NX
%         for j=1:NY
%             if not(isempty(PXY{i,j}))
%                 PXY{i,j}.label=string(box_label)+"_"+string(subpolygon_counter);%Store the counter of subpolygon number
%                 %Calculate IN and ON points based on RADAR DETECTIONS X and Y
%                 [PXY{i,j}.IN, PXY{i,j}.ON] = inpolygon(X,Y,PXY{i,j}.x,PXY{i,j}.y);
%                 PXY{i,j}.edge_points=numel(X(PXY{i,j}.ON));%Store the number of points lying on the edge of the polygon area
%                 PXY{i,j}.in_points=numel(X(PXY{i,j}.IN));%Store the number of points lying inside of the polygon area 
%                 if PXY{i,j}.in_points>0 || PXY{i,j}.edge_points>0
%                     PXY{i,j}.mean_exists=1; %Store a flag indicating there is a representative (calculated as the mean)
%                     %Store the mean representative
%                     PXY{i,j}.representative_mean=mean([X(PXY{i,j}.IN),X(PXY{i,j}.ON);Y(PXY{i,j}.IN),Y(PXY{i,j}.ON);],2);
%                     PXY{i,j}.detections = [unclustered_detections(:,PXY{i,j}.IN),unclustered_detections(:,PXY{i,j}.ON)];
%                     PXY{i,j}.radial = sqrt((PXY{i,j}.representative_mean(1)^2) + (PXY{i,j}.representative_mean(2)^2));
%                     PXY{i,j}.orientation_rad = atan2(PXY{i,j}.representative_mean(2), PXY{i,j}.representative_mean(1));
%                     PXY{i,j}.timestamp = timestamp;
%                 else
%                     PXY{i,j}.mean_exists=0;%Store a flag indicating there is NOT any representative
%                 end
%             else
%                 disp("Empty");
%             end
%             %Increase subpolygons counter:
%             subpolygon_counter=subpolygon_counter+1;
%         end
%     end
% end
% 
% function PXY=DIVIDEXY(polygon,NX,NY)
% %Ayad Al-Rumaithi (2020). Divide Polygon (https://www.mathworks.com/matlabcentral/fileexchange/71635-divide-polygon), MATLAB Central File Exchange. Retrieved July 13, 2020.
% %Input
% %polygon: a structure consist of polygon.x (vector of x-coordinates) and polygon.y (vector of y-coordinates) 
% %NX: Number of divisions in x direction
% %NY: Number of divisions in y direction
% %Output
% %PXY: a cell array where PX{i,j}.x and PX{i,j} are vectors of x and y coordinates of new polygon in (i,j) grid position
%     DX=(max(polygon.x)-min(polygon.x))/NX;
%     DY=(max(polygon.y)-min(polygon.y))/NY;
%     i=0;
%     P=polygon;
%     for X=min(polygon.x)+DX:DX:max(polygon.x)-DX
%         i=i+1;
%         [PX{i}, P]=DIVIDEX(P,X);
%     end
%     PX{NX}=P;
%     for i=1:1:NX
%         j=0;
%         for Y=min(polygon.y)+DY:DY:max(polygon.y)-DY
%             j=j+1;
%             [PXY{i,j}, PX{i}]=DIVIDEY(PX{i},Y);     
%         end
%         PXY{i,NY}=PX{i};
%     end
% end
% 
% 
% function [polygon1 polygon2]=DIVIDEX(polygon,X)
% %Ayad Al-Rumaithi (2020). Divide Polygon (https://www.mathworks.com/matlabcentral/fileexchange/71635-divide-polygon), MATLAB Central File Exchange. Retrieved July 13, 2020.
%     polygon1=[];
%     polygon2=[];
%     if not(isempty(polygon))
%         m=length(polygon.x);
%         c1=0;
%         c2=0;
%         for i=1:m
%             j=i+1;
%             if i==m
%                 j=1;
%             end 
%             if polygon.x(i)<=X
%                 c1=c1+1;
%                 polygon1.x(c1)=polygon.x(i);
%                 polygon1.y(c1)=polygon.y(i);
%             end
%             if polygon.x(i)>=X
%                 c2=c2+1;
%                 polygon2.x(c2)=polygon.x(i);
%                 polygon2.y(c2)=polygon.y(i);  
%             end
%             if (polygon.x(i)>X && polygon.x(j)<X) || (polygon.x(i)<X && polygon.x(j)>X) 
%                 c1=c1+1;
%                 polygon1.x(c1)=X;
%                 polygon1.y(c1)=polygon.y(j)+(polygon.y(i)-polygon.y(j))/(polygon.x(i)-polygon.x(j))*(X-polygon.x(j));    
%                 c2=c2+1;
%                 polygon2.x(c2)=X;
%                 polygon2.y(c2)=polygon.y(j)+(polygon.y(i)-polygon.y(j))/(polygon.x(i)-polygon.x(j))*(X-polygon.x(j));           
%             end
%         end
%     end
% end
% 
% 
% function [polygon1 polygon2]=DIVIDEY(polygon,Y)
% %Ayad Al-Rumaithi (2020). Divide Polygon (https://www.mathworks.com/matlabcentral/fileexchange/71635-divide-polygon), MATLAB Central File Exchange. Retrieved July 13, 2020.
%     polygon1=[];
%     polygon2=[];
%     if not(isempty(polygon))
%         m=length(polygon.y);
%         c1=0;
%         c2=0;
%         for i=1:1:m
%             j=i+1;
%             if i==m
%                 j=1;
%             end 
%             if polygon.y(i)<=Y
%                 c1=c1+1;
%                 polygon1.x(c1)=polygon.x(i);
%                 polygon1.y(c1)=polygon.y(i);
%             end
%             if polygon.y(i)>=Y
%                 c2=c2+1;
%                 polygon2.x(c2)=polygon.x(i);
%                 polygon2.y(c2)=polygon.y(i);  
%             end
% 
%             if (polygon.y(i)>Y && polygon.y(j)<Y) || (polygon.y(i)<Y && polygon.y(j)>Y) 
%                 c1=c1+1;
%                 polygon1.x(c1)=polygon.x(j)+(polygon.x(i)-polygon.x(j))/(polygon.y(i)-polygon.y(j))*(Y-polygon.y(j));    
%                 polygon1.y(c1)=Y;
%                 c2=c2+1;
%                 polygon2.x(c2)=polygon.x(j)+(polygon.x(i)-polygon.x(j))/(polygon.y(i)-polygon.y(j))*(Y-polygon.y(j));           
%                 polygon2.y(c2)=Y;
%             end
%         end
%     end
% end