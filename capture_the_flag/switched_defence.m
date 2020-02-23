clear all, close all, clc

N = 7;                        % Number of agents for cyclic pursuit
NC = 3;                       % Number of agents for C formation defense
NT = N + NC                   % Total number of agents for defence
dt=0.01;                      % numerical steplength
max_iter = 5000;  

% state for switching
state = 1;

% Delta-Disk parameters
disk_delta = 0.2;
wedge_delta = 0.3;
wedge_theta = 2*pi/3;

delta_angles = 0:0.01:2*pi;
disk_angles_x = repmat(disk_delta*cos(delta_angles),1, NT);
disk_angles_y = repmat(disk_delta*sin(delta_angles),1, NT);

% Initialize robotarium
rb = RobotariumBuilder();
rbtm = rb.set_number_of_agents(NT).set_save_data(false).build();
[si_to_uni_dyn] = create_si_to_uni_mapping3();

% Initialize robots
xT = rbtm.get_poses();                                      % States of real unicycle robots                                           % States of all robots
x = xT(1:2,:);                                              % x-y positions of robots in cyclic pursuit
rbtm.set_velocities(1:NT, zeros(2,NT));                     % Assign dummy zero velocity
rbtm.step();                                                % Run robotarium step

% Draw the delta-disk for the robots
disk_poses = repelem(x,1, length(delta_angles));
disk_poses_x_p = disk_poses(1,1:length(delta_angles)*NT) + disk_angles_x(1:length(delta_angles)*NT);
disk_poses_y_p = disk_poses(2,1:length(delta_angles)*NT) + disk_angles_y(1:length(delta_angles)*NT);
delta_handle_p = scatter(disk_poses_x_p, disk_poses_y_p, 1);
delta_handle_p.XDataSource = 'disk_poses_x_p';
delta_handle_p.YDataSource = 'disk_poses_y_p';

% Cyclic graph
theta = pi/NT;
R = [cos(theta) sin(theta),-sin(theta) cos(theta)];
% A = diag(ones(N-1,1),1);
% A(N,1) = 1;
% L = diag(sum(A)) - A;
A1 = diag(ones(NT-1,1),1) + diag(ones(NT-1,1),-1);
A1(NT,1) = 1;
A1(1,NT) = 1;
L1 = diag(sum(A1)) - A1;

% Parameters of C Formation
LC = completeGL(3);
AC = ones(3,3);
AC(1,1) = 0;
AC(2,2) = 0;
AC(3,3) = 0;
WC = [0 0.5 0.5;0.5 0 0.5;0.5 0.5 0];

% Target cycle definition
center = [0;0];
radius = 0.08;
interAgentDistance = radius*2*sin(pi/NT);
kp1 = 7;
kp2 = 0.08;
plot(center(1),center(2),'*','markersize',12)
th = 0 : 2*pi/40 : 2*pi-2*pi/40+1;
plot(radius.*cos(th)+center(1),radius.*sin(th)+center(2),'b')


% Reach initial positions on a circle
circularTargets = [ cos( 0:2*pi/NT:(2*pi*(1- 1/NT)) )/4 ; sin( 0:2*pi/NT:(2*pi*(1- 1/NT)) )/4 ];
errorToInitialPos = x - circularTargets;                % Error
errorNorm = [1,1]*(errorToInitialPos.^2);               % Norm of error
while max( errorNorm ) > 0.05
    % Update state variables        
    xT = rbtm.get_poses();                            % States of real unicycle robots
    x = xT(1:2,:);                                    % x-y positions

    % Update errors
    errorToInitialPos = x - circularTargets;
    errorNorm = [1,1]*(errorToInitialPos.^2);

    % Conput control inputs
    u = -0.3.*errorToInitialPos;
    dx = si_to_uni_dyn(u, xT);

    % Assing new control inputs to robots
    rbtm.set_velocities(1:NT, dx);                      % Assign dummy zero velocity
    rbtm.step();                                        % Run robotarium step
    
    L = update_delta_disk(x, disk_delta);
    
    disk_poses = repelem(x,1, length(delta_angles));
    disk_poses_x_p = disk_poses(1,1:length(delta_angles)*NT) + disk_angles_x(1:length(delta_angles)*NT);
    disk_poses_y_p = disk_poses(2,1:length(delta_angles)*NT) + disk_angles_y(1:length(delta_angles)*NT);
    disk_poses_x_d = disk_poses(1,(length(delta_angles)*NT)+1:end) + disk_angles_x((length(delta_angles)*NT)+1:end);
    disk_poses_y_d = disk_poses(2,(length(delta_angles)*NT)+1:end) + disk_angles_y((length(delta_angles)*NT)+1:end);
    refreshdata([delta_handle_p]);
end
disp('Initial positions reached')

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
end


  
for k = 1:max_iter
    
%     % Get new data and initialize new null velocities
%     xT = rbtm.get_poses();                                  % Get new robots' states
%     x = xT(1:2,:);                                          % Extract single integrator states
%     dx = zeros(2,NT);                                       % Initialize velocities to zero for all robots   
    

    switch state
        case 1
            % Get new data and initialize new null velocities
            xT = rbtm.get_poses();                                  % Get new robots' states
            x = xT(1:2,:);                                          % Extract single integrator states
            dx = zeros(2,NT);                                       % Initialize velocities to zero for all robots   
            interAgentDistance = radius*2*sin(pi/NT);
            for i = 1:NT-1
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
            dx(:,NT) = [0;0];
            if (norm(x(:,1)-x(:,NT))-interAgentDistance)>0
                alpha = theta - (pi/18);
                R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
            else
                alpha = theta + (pi/18);
                R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
            end
            dx(:,NT) = dx(:,NT) + R*(x(:,1)-x(:,NT));
            
            disk_poses = repelem(x,1, length(delta_angles));
            disk_poses_x_p = disk_poses(1,1:length(delta_angles)*NT) + disk_angles_x(1:length(delta_angles)*NT);
            disk_poses_y_p = disk_poses(2,1:length(delta_angles)*NT) + disk_angles_y(1:length(delta_angles)*NT);
            disk_poses_x_d = disk_poses(1,(length(delta_angles)*NT)+1:end) + disk_angles_x((length(delta_angles)*NT)+1:end);
            disk_poses_y_d = disk_poses(2,(length(delta_angles)*NT)+1:end) + disk_angles_y((length(delta_angles)*NT)+1:end);
            refreshdata([delta_handle_p]);
            L = update_delta_disk(x, disk_delta)
            
            dx = si_to_uni_dyn(dx, xT);                            % Convert single integrator inputs into unicycle inputs
            rbtm.set_velocities(1:NT, dx); rbtm.step();            % Set new velocities to robots and update
            if k == 3000
                state = 2;
            end     
            
            
        case 2
            radius_new = 0.06;
            center_new = [0;0];
            interAgentDistance_new = radius_new*2*sin(pi/N);
            plot(center(1),center(2),'.','markersize',12)
            th = 0 : 2*pi/40 : 2*pi-2*pi/40+1;
            plot(radius_new.*cos(th)+center_new(1),radius_new.*sin(th)+center_new(2),'g')

            % Get new data and initialize new null velocities
            xT = rbtm.get_poses();                                  % Get new robots' states
            x = xT(1:2,:);                                          % Extract single integrator states
            dx = zeros(2,NT);                                       % Initialize velocities to zero for all robots   
%             dx = zeros(2,NT);
%             dx = si_to_uni_dyn(dx, xT);                            % Convert single integrator inputs into unicycle inputs
%             rbtm.set_velocities(1:NT, dx); rbtm.step();            % Set new velocities to robots and update
            interAgentDistance = radius*2*sin(pi/N);
                     
            for i = 1:N -1
                dx(:,i) = [0;0];
                if (norm(x(:,i+1)-x(:,i))-interAgentDistance)>0
                    alpha = theta - (pi/10);
                    R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
                else
                    alpha = theta + (pi/10);
                    R  = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
                end
                dx(:,i) = dx(:,i) + R*(x(:,i+1)-x(:,i)) - (norm(x(:,i)-center)-0.08)*(x(:,i) - center);
            end
            if (norm(x(:,1)-x(:,N))-interAgentDistance)>0
                alpha = theta - (pi/10);
                R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
            else
                alpha = theta + (pi/10);
                R  = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
            end
            dx(:,N) = dx(:,N) + R*(x(:,1)-x(:,N))- (norm(x(:,N)-center)-0.08)*(x(:,N) - center);
            
            for c = N+1:N+NC
                dx(:,c) = [0;0];
                for d = N+1:N+NC
                    dx(:,c) = dx(:,c) + AC(c-N,d-N)*(norm(x(:,d)-x(:,c))-(WC(c-N,d-N)/4))*(x(:,d)-x(:,c));
                end
            end
            dx(:,N+1) = dx(:,N+1) + ((norm(center-x(:,N+1))-0.6)*(center-x(:,N+1)));
            
            disk_poses = repelem(x,1, length(delta_angles));
            disk_poses_x_p = disk_poses(1,1:length(delta_angles)*NT) + disk_angles_x(1:length(delta_angles)*NT);
            disk_poses_y_p = disk_poses(2,1:length(delta_angles)*NT) + disk_angles_y(1:length(delta_angles)*NT);
            disk_poses_x_d = disk_poses(1,(length(delta_angles)*NT)+1:end) + disk_angles_x((length(delta_angles)*NT)+1:end);
            disk_poses_y_d = disk_poses(2,(length(delta_angles)*NT)+1:end) + disk_angles_y((length(delta_angles)*NT)+1:end);
            refreshdata([delta_handle_p]);
           L = update_delta_disk(x, disk_delta)
            
            dx = si_to_uni_dyn(dx, xT);                            % Convert single integrator inputs into unicycle inputs
            rbtm.set_velocities(1:NT, dx); rbtm.step();            % Set new velocities to robots and update
            
    end     
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
rbtm.call_at_scripts_end();