function[captured] = is_flag_captured(poses, flagpos)
    N = length(poses)
    for i = N/2+1:N
        if(norm(poses(:, i) - flagpos) < 0.03)
            captured = 1
        else
            captured = 0
        end
    end
end
