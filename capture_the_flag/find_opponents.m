function[opponents] = find_opponents(L, agent, NP)
    N = size(L, 2);
    if(agent > NP)
        all_opps = [1:NP];
    else
        all_opps = [NP+1:N];
    end
    arr = L(agent, all_opps);
    opponents = find(arr~=0);
end
