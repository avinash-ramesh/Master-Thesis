function [BEP, hFigure, hBEVPanel] = create_demo_display(sensors)%, lon_lat_timestamp, i)
    % Make a figure
    hFigure = figure('Position', [0, 0, 1200, 640], 'Name', 'Radar - Camera Fusion with NuScenes dataset');
    movegui(hFigure, [0 -1]); % Moves the figure to the left and a little down from the top
    
    % Add a panel for a bird's-eye plot
    hBEVPanel = uipanel(hFigure, 'Position', [0.5 0 0.5 1], 'Title', 'Bird''s-Eye Plot');

    % Create bird's-eye plot for the ego vehicle and sensor coverage
    hBEVPlot = axes(hBEVPanel);
    frontBackLim = 40; %60
    BEP = birdsEyePlot('Parent', hBEVPlot, 'Xlimits', [-frontBackLim frontBackLim], 'Ylimits', [-40 40]);%[-35 35]);

    % Plot the coverage areas for radars
	
    sensor_labels = {'RADAR\_FRONT\_LEFT', 'RADAR\_FRONT', 'RADAR\_FRONT\_RIGHT', 'RADAR\_BACK\_RIGHT', 'RADAR\_BACK\_LEFT'};
    sensor_colors = {'magenta','black','blue','green','cyan'};
    for i = 1:length(sensors)
        cap = coverageAreaPlotter(BEP,'FaceColor', sensor_colors{i},'EdgeColor', sensor_colors{i},'DisplayName', sensor_labels{i}); 
        plotCoverageArea(cap, sensors{i}.SensorLocation,...
            sensors{i}.MaxRange, sensors{i}.Yaw, sensors{i}.FieldOfView(1));
    end

    % Combine all radar detections into one entry and store it for later update
    detectionPlotter(BEP, 'DisplayName', 'radar', 'MarkerEdgeColor','red');

    axis(BEP.Parent, 'equal');
    xlim(BEP.Parent, [-frontBackLim frontBackLim]);
    ylim(BEP.Parent, [-40 40]);
end