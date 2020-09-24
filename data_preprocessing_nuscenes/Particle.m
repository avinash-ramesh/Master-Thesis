classdef Particle
    %Particle Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        X
        local_landmark_map
    end
    
    methods
        function self = Particle(x)
            self.X = x;
            self.local_landmark_map = containers.Map;
        end
        
        function self = update_landmark(self, id, landmark_X, landmark_P)
            landmark.X = landmark_X;
            landmark.P = landmark_P;
            self.local_landmark_map(id) = landmark;
        end
    end
end

