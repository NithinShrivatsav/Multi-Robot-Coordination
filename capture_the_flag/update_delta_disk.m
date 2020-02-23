function [L] = update_delta_disk(poses, delta)
% this is used to create a delta disk graph within a team of agents. pass the corresponding poses and delta as input. 
    N = size(poses, 2);
    L = zeros(N, N);
    for agent = 1:N
        neighbors = sqrt(sum((poses(1:2, :) - repmat(poses(1:2, agent), 1, N)).^2)) <= delta;
        L(agent, :) = neighbors;
        if(nnz(neighbors) >= 1)
            L(agent, agent) = -(nnz(neighbors) - 1);
        end
    end
    L = -L;
end
