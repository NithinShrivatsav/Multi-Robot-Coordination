function [L] = update_wedge_graph(poseA, poseB, headingsA, wedge_delta, theta)
% poseB - pose of opposite team
% poseA - pose of sensing team
% assiming that headings ranges from 0 to 2*pi

    N = size(poseA, 2);
    L = zeros(N, N);
    ends = zeros(2, N);
    ends(1, :) = mod((headingsA - theta/2), 2*pi);
    ends(2, :) = mod((headingsA + theta/2), 2*pi);
    ends2 = atan2(sin(ends), cos(ends));
    for agent = 1:N
        neighbors1 = sqrt(sum((poseB(1:2, :) - repmat(poseA(1:2, agent), 1, N)).^2)) <= wedge_delta;
        % confusion here as to whether we should retain headings in [0,  2pi]
        rel_headings2 = atan2(poseB(2, :) - repmat(poseA(2, agent), 1, N), poseB(1, :) - repmat(poseA(1, agent), 1, N)); 
        rel_headings = rel_headings2;
        rel_headings(rel_headings < 0) = 2*pi + rel_headings(rel_headings < 0);
        neighbors2(1, :) = rel_headings > ends(1, agent);
        neighbors3(1, :) = rel_headings < ends(2, agent);
        neighbors2(2, :) = rel_headings2 > ends2(1, agent);
        neighbors3(2, :) = rel_headings2 < ends2(2, agent);
        L(agent, :) = neighbors1.*((neighbors2(1,:).*neighbors3(1, :)) | (neighbors2(2,:).*neighbors3(2, :)));        
        L(agent, agent) = 0;
    end
    L = -L;
end
