clear all, close all, clc

N = 6;
dt = 0.01;
max_iter = 500;

disk_delta  = 0.2;
wedge_delta = 0.3;
wedge_theta = 2*pi/3; % Not used just for reference

delta_angles = 0:0.01:2*pi;
disk_angles_x = repmat(disk_delta*cos(delta_angles),1, N);
disk_angles_y = repmat(disk_delta*sin(delta_angles),1, N);

wedge_line = 0:0.01:0.3;
wedge_line_left = repmat(wedge_line,1,N);
wedge_line_right = repmat(wedge_line,1,N);
wedge_angle_left = -pi/3; % left-most angle of wedge if theta is aligned with axis
wedge_angle_right = pi/3; % right-most angle of wedge if theta is aligned with axis
wedge_angles = wedge_angle_left:0.01:wedge_angle_right; % all intermediate angles to create wedge
wedge_angles_theta = repmat(wedge_angles,1,N); % A matrix of intermediate angles for efficient computation without loops

% Initialize robotarium
rb = RobotariumBuilder();
rbtm = rb.set_number_of_agents(N).set_save_data(false).build();
[si_to_uni_dyn] = create_si_to_uni_mapping3();

% Initialize robots
xuni = rbtm.get_poses();                                    % States of real unicycle robots
x = xuni(1:2,:);                                            % x-y positions only
theta = xuni(3,:);
rbtm.set_velocities(1:N, zeros(2,N));                       % Assign dummy zero velocity
rbtm.step();                                  

% Graph Specifications
% Cyclic graph
theta = pi/N;
R = [cos(theta) sin(theta),-sin(theta) cos(theta)];
A = diag(ones(N-1,1),1);
A(N,1) = 1;
L = diag(sum(A)) - A;
A1 = diag(ones(N-1,1),1) + diag(ones(N-1,1),-1);
A1(N,1) = 1;
A1(1,N) = 1;

% Target cycle definition
center = [0;0];
radius = 0.08;
interAgentDistance = radius*2*sin(pi/N);
plot(center(1),center(2),'*','markersize',12)
th = 0 : 2*pi/40 : 2*pi-2*pi/40+1;
plot(radius.*cos(th)+center(1),radius.*sin(th)+center(2),'b')

% Draw the delta-disk for protectors
disk_poses = repelem(x,1, length(delta_angles));
disk_poses_x_p = disk_poses(1,1:length(delta_angles)*N) + disk_angles_x(1:length(delta_angles)*N);
disk_poses_y_p = disk_poses(2,1:length(delta_angles)*N) + disk_angles_y(1:length(delta_angles)*N);
delta_handle_p = scatter(disk_poses_x_p, disk_poses_y_p, 1);
delta_handle_p.XDataSource = 'disk_poses_x_p';
delta_handle_p.YDataSource = 'disk_poses_y_p';

% Draw the wedge-graph for defender

wedge_poses = repelem(x,1,length(wedge_angles));
wedge_poses_theta = repelem(xuni(3,:),1,length(wedge_angles));
wedge_poses_theta_p = wedge_poses_theta(1,1:length(wedge_angles)*N) + wedge_angles_theta(1:length(wedge_angles)*N);
wedge_angles_x = wedge_delta.*cos(wedge_poses_theta_p);
wedge_angles_y = wedge_delta.*sin(wedge_poses_theta_p);

wedge_poses_x_p = wedge_poses(1,1:length(wedge_angles)*N) + wedge_angles_x(1:length(wedge_angles)*N); 
wedge_poses_y_p = wedge_poses(2,1:length(wedge_angles)*N) + wedge_angles_y(1:length(wedge_angles)*N);
wedge_handle_p = scatter(wedge_poses_x_p,wedge_poses_y_p,1,'green');
wedge_handle_p.XDataSource = 'wedge_poses_x_p';
wedge_handle_p.YDataSource = 'wedge_poses_y_p';

wedge_line_theta = repelem(xuni(3,:),1,length(wedge_line));
theta_left = repelem(-pi/3,1,length(wedge_line));
theta_left_final = repmat(theta_left,1,N);
theta_right = repelem(pi/3,1,length(wedge_line));
theta_right_final = repmat(theta_right,1,N);
wedge_line_theta_left = wedge_line_theta + theta_left_final;
wedge_line_theta_right = wedge_line_theta + theta_right_final;
wedge_poses_angle = repelem(x,1,length(wedge_line));

wedge_line_left_x = wedge_poses_angle(1,:) + wedge_line_left.*cos(wedge_line_theta_left);
wedge_line_left_y = wedge_poses_angle(2,:) + wedge_line_left.*sin(wedge_line_theta_left);

wedge_line_right_x = wedge_poses_angle(1,:) + wedge_line_right.*cos(wedge_line_theta_right);
wedge_line_right_y = wedge_poses_angle(2,:) + wedge_line_right.*sin(wedge_line_theta_right);

wedge_line_handle_left = scatter(wedge_line_left_x,wedge_line_left_y,1,'green');
wedge_line_handle_right = scatter(wedge_line_right_x,wedge_line_right_y,1,'green');

wedge_line_handle_left.XDataSource = 'wedge_line_left_x';
wedge_line_handle_left.YDataSource = 'wedge_line_left_y';

wedge_line_handle_right.XDataSource = 'wedge_line_right_x';
wedge_line_handle_right.YDataSource = 'wedge_line_right_y';


circularTargets = [ cos( 0:2*pi/N:2*pi*(1- 1/N) ) ; sin( 0:2*pi/N:2*pi*(1- 1/N) ) ];
errorToInitialPos = x - circularTargets;                % Error
errorNorm = [1,1]*(errorToInitialPos.^2);               % Norm of error
while max( errorNorm ) > 0.005
    % Update state variables        
    xuni = rbtm.get_poses();                            % States of real unicycle robots
    x = xuni(1:2,:);                                    % x-y positions

    % Update errors
    errorToInitialPos = x - circularTargets;
    errorNorm = [1,1]*(errorToInitialPos.^2);

    % Conput control inputs
    u = -0.3.*errorToInitialPos;
    dx = si_to_uni_dyn(u, xuni);

    % Assing new control inputs to robots
    rbtm.set_velocities(1:N, dx);                       % Assign dummy zero velocity
    rbtm.step();                                        % Run robotarium step
    
    disk_poses = repelem(x,1, length(delta_angles));
    disk_poses_x_p = disk_poses(1,1:length(delta_angles)*N) + disk_angles_x(1:length(delta_angles)*N);
    disk_poses_y_p = disk_poses(2,1:length(delta_angles)*N) + disk_angles_y(1:length(delta_angles)*N);
    
    wedge_poses = repelem(x,1,length(wedge_angles));
    wedge_poses_theta = repelem(xuni(3,:),1,length(wedge_angles));
    wedge_poses_theta_p = wedge_poses_theta(1,1:length(wedge_angles)*N) + wedge_angles_theta(1:length(wedge_angles)*N);
    wedge_angles_x = wedge_delta.*cos(wedge_poses_theta_p);
    wedge_angles_y = wedge_delta.*sin(wedge_poses_theta_p);
    wedge_poses_x_p = wedge_poses(1,1:length(wedge_angles)*N) + wedge_angles_x(1:length(wedge_angles)*N); 
    wedge_poses_y_p = wedge_poses(2,1:length(wedge_angles)*N) + wedge_angles_y(1:length(wedge_angles)*N);
    
    wedge_line_theta = repelem(xuni(3,:),1,length(wedge_line));
    theta_left = repelem(-pi/3,1,length(wedge_line));
    theta_left_final = repmat(theta_left,1,N);
    theta_right = repelem(pi/3,1,length(wedge_line));
    theta_right_final = repmat(theta_right,1,N);
    wedge_line_theta_left = wedge_line_theta + theta_left_final;
    wedge_line_theta_right = wedge_line_theta + theta_right_final;
    wedge_poses_angle = repelem(x,1,length(wedge_line));
    wedge_line_left_x = wedge_poses_angle(1,:) + wedge_line_left.*cos(wedge_line_theta_left);
    wedge_line_left_y = wedge_poses_angle(2,:) + wedge_line_left.*sin(wedge_line_theta_left);
    wedge_line_right_x = wedge_poses_angle(1,:) + wedge_line_right.*cos(wedge_line_theta_right);
    wedge_line_right_y = wedge_poses_angle(2,:) + wedge_line_right.*sin(wedge_line_theta_right);
    
    
    
    refreshdata([delta_handle_p, wedge_handle_p,wedge_line_handle_left,wedge_line_handle_right]);
end
disp('Initial positions reached')

% Correcting the orientation for cyclic pursuit
xuni = rbtm.get_poses();                            % States of real unicycle robots
x = xuni(1:2,:);
y = zeros(2,N);
y(1,:) = -1.*x(2,:);
y(2,:) = x(1,:);
x_theta = xuni(3,:);
        
x_theta_desired = atan2(y(2,:),y(1,:));
e =  x_theta_desired - x_theta; 
e_prime = atan2(sin(e),cos(e));
e_Norm = norm(e_prime); 
rbtm.set_velocities(1:N, zeros(2,N));                       % Assign dummy zero velocity
rbtm.step();                                                % Run robotarium step
w = zeros(2,N);

while max(e_Norm) > 0.005
    xuni = rbtm.get_poses();
    x_theta = xuni(3,:);
   
    e =  x_theta_desired - x_theta; 
    e_prime = atan2(sin(e),cos(e));
    e_Norm = norm(e_prime);
    
    w(2,:) = 0.3.*e_prime;
    
    rbtm.set_velocities(1:N, w);                       % Assign dummy zero velocity
    rbtm.step();  
    
    wedge_poses = repelem(x,1,length(wedge_angles));
    wedge_poses_theta = repelem(xuni(3,:),1,length(wedge_angles));
    wedge_poses_theta_p = wedge_poses_theta(1,1:length(wedge_angles)*N) + wedge_angles_theta(1:length(wedge_angles)*N);
    wedge_angles_x = wedge_delta.*cos(wedge_poses_theta_p);
    wedge_angles_y = wedge_delta.*sin(wedge_poses_theta_p);
    wedge_poses_x_p = wedge_poses(1,1:length(wedge_angles)*N) + wedge_angles_x(1:length(wedge_angles)*N); 
    wedge_poses_y_p = wedge_poses(2,1:length(wedge_angles)*N) + wedge_angles_y(1:length(wedge_angles)*N);
    
    wedge_line_theta = repelem(xuni(3,:),1,length(wedge_line));
    theta_left = repelem(-pi/3,1,length(wedge_line));
    theta_left_final = repmat(theta_left,1,N);
    theta_right = repelem(pi/3,1,length(wedge_line));
    theta_right_final = repmat(theta_right,1,N);
    wedge_line_theta_left = wedge_line_theta + theta_left_final;
    wedge_line_theta_right = wedge_line_theta + theta_right_final;
    wedge_poses_angle = repelem(x,1,length(wedge_line));
    wedge_line_left_x = wedge_poses_angle(1,:) + wedge_line_left.*cos(wedge_line_theta_left);
    wedge_line_left_y = wedge_poses_angle(2,:) + wedge_line_left.*sin(wedge_line_theta_left);
    wedge_line_right_x = wedge_poses_angle(1,:) + wedge_line_right.*cos(wedge_line_theta_right);
    wedge_line_right_y = wedge_poses_angle(2,:) + wedge_line_right.*sin(wedge_line_theta_right);
       
    refreshdata([delta_handle_p, wedge_handle_p, wedge_line_handle_left,wedge_line_handle_right]);
end

for k = 1:max_iter
    
    % Get new data and initialize new null velocities
    xuni = rbtm.get_poses();                                % Get new robots' states
    x = xuni(1:2,:);                                        % Extract single integrator states

    dx = zeros(2,N);                                           % Initialize velocities to zero
    
    for i = 1:N-1
        dx(:,i) = [0;0];
        if (norm(x(:,i+1)-x(:,i))-interAgentDistance)>0
            alpha = theta - (pi/18);
            R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
        else
            alpha = theta + (pi/18);
            R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
        end
        dx(:,i) = dx(:,i) + R*(x(:,i+1)-x(:,i));
        
    end
    dx(:,N) = [0;0];
    if (norm(x(:,1)-x(:,N))-interAgentDistance)>0
        alpha = theta - (pi/18);
        R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
    else
        alpha = theta + (pi/18);
        R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
    end
    dx(:,N) = dx(:,N) + R*(x(:,1)-x(:,N));
    
%     plot(x(1,1),x(2,1),'.')
    dx = si_to_uni_dyn(dx, xuni);                            % Convert single integrator inputs into unicycle inputs
    rbtm.set_velocities(1:N, dx); 
    rbtm.step();              % Set new velocities to robots and update
    
    disk_poses = repelem(x,1, length(delta_angles));
    disk_poses_x_p = disk_poses(1,1:length(delta_angles)*N) + disk_angles_x(1:length(delta_angles)*N);
    disk_poses_y_p = disk_poses(2,1:length(delta_angles)*N) + disk_angles_y(1:length(delta_angles)*N);
    
    wedge_poses = repelem(x,1,length(wedge_angles));
    wedge_poses_theta = repelem(xuni(3,:),1,length(wedge_angles));
    wedge_poses_theta_p = wedge_poses_theta(1,1:length(wedge_angles)*N) + wedge_angles_theta(1:length(wedge_angles)*N);
    wedge_angles_x = wedge_delta.*cos(wedge_poses_theta_p);
    wedge_angles_y = wedge_delta.*sin(wedge_poses_theta_p);
    wedge_poses_x_p = wedge_poses(1,1:length(wedge_angles)*N) + wedge_angles_x(1:length(wedge_angles)*N); 
    wedge_poses_y_p = wedge_poses(2,1:length(wedge_angles)*N) + wedge_angles_y(1:length(wedge_angles)*N);
    
    
    wedge_line_theta = repelem(xuni(3,:),1,length(wedge_line));
    theta_left = repelem(-pi/3,1,length(wedge_line));
    theta_left_final = repmat(theta_left,1,N);
    theta_right = repelem(pi/3,1,length(wedge_line));
    theta_right_final = repmat(theta_right,1,N);
    wedge_line_theta_left = wedge_line_theta + theta_left_final;
    wedge_line_theta_right = wedge_line_theta + theta_right_final;
    wedge_poses_angle = repelem(x,1,length(wedge_line));
    wedge_line_left_x = wedge_poses_angle(1,:) + wedge_line_left.*cos(wedge_line_theta_left);
    wedge_line_left_y = wedge_poses_angle(2,:) + wedge_line_left.*sin(wedge_line_theta_left);
    wedge_line_right_x = wedge_poses_angle(1,:) + wedge_line_right.*cos(wedge_line_theta_right);
    wedge_line_right_y = wedge_poses_angle(2,:) + wedge_line_right.*sin(wedge_line_theta_right);
       
    refreshdata([delta_handle_p, wedge_handle_p, wedge_line_handle_left,wedge_line_handle_right]);
    
end

rbtm.call_at_scripts_end();
