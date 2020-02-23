clear all, close all, clc
% ensure that if you have N agents, first N/2 are protectors are the rest
% are decoyers
N=12;                       % Number of agents
NP = 6;                    % Number of protectors (also change these in other defined functions)
ND = 6;                    % Number of "decoyers"
protectors = [1:6]';       % indices for two teams
decoyers = [7:12]';
dt=0.01;                    % numerical steplength
max_iter = 1000;  

disk_delta = 0.2;
wedge_delta = 0.2;
wedge_theta = 2*pi/3;

delta_angles = 0:0.01:2*pi;
disk_angles_x = repmat(disk_delta*cos(delta_angles),1, N);
disk_angles_y = repmat(disk_delta*sin(delta_angles),1, N);

% Initialize robotarium
rb = RobotariumBuilder();
rbtm = rb.set_number_of_agents(N).set_save_data(false).build();
[si_to_uni_dyn] = create_si_to_uni_mapping3();

% Initialize robots
xuni = rbtm.get_poses();                                    % States of real unicycle robots
x = xuni(1:2,:);                                            % x-y positions only
rbtm.set_velocities(1:N, zeros(2,N));                       % Assign dummy zero velocity
rbtm.step();                                                % Run robotarium step

disk_poses = repelem(x,1, length(delta_angles));
disk_poses_x_p = disk_poses(1,1:length(delta_angles)*NP) + disk_angles_x(1:length(delta_angles)*NP);
disk_poses_y_p = disk_poses(2,1:length(delta_angles)*NP) + disk_angles_y(1:length(delta_angles)*NP);
delta_handle_p = scatter(disk_poses_x_p, disk_poses_y_p, 1);
delta_handle_p.XDataSource = 'disk_poses_x_p';
delta_handle_p.YDataSource = 'disk_poses_y_p';

disk_poses_x_d = disk_poses(1,(length(delta_angles)*NP)+1:end) + disk_angles_x((length(delta_angles)*NP)+1:end);
disk_poses_y_d = disk_poses(2,(length(delta_angles)*NP)+1:end) + disk_angles_y((length(delta_angles)*NP)+1:end);
hold on
delta_handle_d = scatter(disk_poses_x_d, disk_poses_y_d, 1, 'red');
hold off
delta_handle_d.XDataSource = 'disk_poses_x_d';
delta_handle_d.YDataSource = 'disk_poses_y_d';

% (length(delta_angles)*NP)+1:length(delta_angles)*N
targets = [ 0.3*cos(0:2*pi/NP:2*pi*(1- 1/NP)) ; 0.3*sin( 0:2*pi/NP:2*pi*(1- 1/NP)) ];
errorToInitialPos(:, 1:NP) = x(:,1:NP) - targets;                % Error
errorNorm = [1,1]*(errorToInitialPos.^2);               % Norm of error
while max( errorNorm ) > 0.002
    % Update state variables        
    xuni = rbtm.get_poses();                            % States of real unicycle robots
    x = xuni(1:2,:);                                    % x-y positions
    % Update errors
    errorToInitialPos(:, 1:NP) = x(:,1:NP) - targets;                % Error
    
    errorNorm = [1,1]*(errorToInitialPos.^2);
    % u = zeros(2, N);
    % Conput control inputs
    % for i = protectors
    %   for j=delta_disk_neighbors(x, i , disk_delta)
    %            u(:, i) = u(:, i) + x(:, j) + x(:, i);
    %  end
    %  end
    
    u = -0.3.*errorToInitialPos;
    dx = si_to_uni_dyn(u, xuni(:, protectors));

    % Assing new control inputs to robots
    rbtm.set_velocities(protectors, dx);                       % Assign dummy zero velocity
    rbtm.step();% Run robotarium step
    
    L = updateGraph(x, xuni(3, :), disk_delta, wedge_delta, wedge_theta)
    
    disk_poses = repelem(x,1, length(delta_angles));
    disk_poses_x_p = disk_poses(1,1:length(delta_angles)*NP) + disk_angles_x(1:length(delta_angles)*NP);
    disk_poses_y_p = disk_poses(2,1:length(delta_angles)*NP) + disk_angles_y(1:length(delta_angles)*NP);
    disk_poses_x_d = disk_poses(1,(length(delta_angles)*NP)+1:end) + disk_angles_x((length(delta_angles)*NP)+1:end);
    disk_poses_y_d = disk_poses(2,(length(delta_angles)*NP)+1:end) + disk_angles_y((length(delta_angles)*NP)+1:end);
    refreshdata([delta_handle_p, delta_handle_d]);
end
disp('Initial positions reached')


for k = 1:max_iter    
    % Get new data and initialize new null velocities
    xuni = rbtm.get_poses();                                % Get new robots' states
    x = xuni(1:2,:);                                        % Extract single integrator states
    u=zeros(2,N);                                           % Initialize velocities to zero
    % FILL THIS PART!!!

    dx = si_to_uni_dyn(u, xuni);                            % Convert single integrator inputs into unicycle inputs
    rbtm.set_velocities(1:N, dx); rbtm.step();              % Set new velocities to robots and update
end


% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
rbtm.call_at_scripts_end();
