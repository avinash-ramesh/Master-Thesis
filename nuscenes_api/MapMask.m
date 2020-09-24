classdef MapMask
    %MapMask Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        img_file
        resolution
        foreground
        background
        base_mask
    end
    
    methods
        function self = MapMask(img_file, resolution)
            %MapMask Construct an instance of this class
            %   Detailed explanation goes here
            assert(and(1,exist(img_file)),"map mask "+img_file+" does not exist");
            assert(resolution >= 0.1, "Only supports down to 0.1 meter resolution.");
            self.img_file = img_file;
            self.resolution = resolution;
            self.foreground = 255;
            self.background = 0;
        end

        function raw_mask = mask(self)
            raw_mask = self.base_mask_fnc();
        end
        
        function final_img = base_mask_fnc(self)
            tmp_img = imread(self.img_file);
            native_resolution = 0.1;
            size_x = int64(size(tmp_img,1) / self.resolution * native_resolution);
            size_y = int64(size(tmp_img,2) / self.resolution * native_resolution);
            final_img = imresize(tmp_img, [size_x, size_y], 'nearest');
        end

        function record = transform_matrix(self)
            tmp_img = self.base_mask_fnc();
            record = [[1.0 / self.resolution, 0, 0, 0]; [0, -1.0/self.resolution, 0, size(tmp_img,1)];[0, 0, 1, 0]; [0, 0, 0, 1]];
        end

        function [px_coord, py_coord] = to_pixel_coords(self, x, y)
            assert(all(size(x) == size(y)));
            pts = [x; y; zeros('like',x); ones('like',x)];
            pixel_coords = int64(round(self.transform_matrix() * pts));
            px_coord = pixel_coords(1, :);
            py_coord = pixel_coords(2, :);
        end
    end
end

