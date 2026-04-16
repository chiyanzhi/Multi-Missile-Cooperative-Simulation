classdef Environment
    properties
        g = 9.81;
        sound_speed = 340;
    end
    
    methods
        function rho = getDensity(~, altitude)
            % 指数大气模型
            rho0 = 1.225; 
            H_scale = 8500; 
            rho = rho0 * exp(-altitude / H_scale);
        end
    end
end