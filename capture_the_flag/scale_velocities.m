function [scaled_vel] = scale_velocities(unscaled_vel)
    max_linear_vel = max(abs(unscaled_vel(1, :)));
    max_angular_vel = max(abs(unscaled_vel(2, :)));
    
    
    if(max_linear_vel < 0.4 && max_angular_vel < 8)
        scaled_vel(:,:) = unscaled_vel;
    else
        excess_linear = max_linear_vel/0.4;
        excess_angular = max_angular_vel/8;
        scaled_vel(:,:) = unscaled_vel./abs((max(excess_linear, excess_angular) ));
    end
end
