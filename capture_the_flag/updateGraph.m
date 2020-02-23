function [L] = updateGraph(poses, headings, delta, wedge_delta, theta)   
% this is used to compute the graph that includes all agents from both teams. 
    N = size(poses, 2);
    Laa = update_delta_disk(poses(1:2, 1:(N/2)), delta);
    Lbb = update_delta_disk(poses(1:2, ((N/2)+1):N), delta);
    Lab = update_wedge_graph(poses(1:2, 1:(N/2)), poses(1:2, ((N/2)+1):N), headings(1:(N/2)), wedge_delta, theta);
    Lba = update_wedge_graph(poses(1:2, ((N/2)+1):N), poses(1:2, 1:(N/2)), headings(((N/2)+1):N), wedge_delta, theta);
    L = [Laa, Lab; Lba, Lbb]; 
end
