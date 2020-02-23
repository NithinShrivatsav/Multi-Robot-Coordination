%% Initializations 
clear;
N = 12;
rbtm = Robotarium('NumberOfRobots', N, 'ShowFigure', true);
NP = 6;                    % Number of protectors (also change these in other defined functions)
ND = 6;                    % Number of "decoyers"
protectors = [1:6]';       % indices for two teams
decoyers = [7:12]';
iterations = 1000;
disk_delta  = 0.4;
wedge_delta = 0.4;
wedge_theta = 2*pi/3;       
flag = [1.1;0.1];
flagpos = [0.6; 0];
delta_angles = 0:0.01:2*pi;
disk_angles_x = repmat(disk_delta*cos(delta_angles),1, N);
disk_angles_y = repmat(disk_delta*sin(delta_angles),1, N);

wedge_line = 0:0.01:0.4;
wedge_line_left = repmat(wedge_line,1,N);
wedge_line_right = repmat(wedge_line,1,N);
wedge_angle_left = -pi/3; % left-most angle of wedge if theta is aligned with axis
wedge_angle_right = pi/3; % right-most angle of wedge if theta is aligned with axis
wedge_angles = wedge_angle_left:0.01:wedge_angle_right; % all intermediate angles to create wedge
wedge_angles_theta = repmat(wedge_angles,1,N); % A matrix of intermediate angles for efficient computation without loops

[si_to_uni_dyn] = create_si_to_uni_mapping3();

%% initialialize robots with zero velocity
xuni = rbtm.get_poses();                                    % States of real unicycle robots
x = xuni(1:2,:);                                            % x-y positions only
rbtm.set_velocities(1:N, zeros(2,N));                       % Assign dummy zero velocity
rbtm.step();

%% Graph Specifications - cyclic graph
theta = pi/N;
R = [cos(theta) sin(theta),-sin(theta) cos(theta)];
A = diag(ones(N-1,1),1);
A(N,1) = 1;
L = diag(sum(A)) - A;
A1 = diag(ones(N-1,1),1) + diag(ones(N-1,1),-1);
A1(N,1) = 1;
A1(1,N) = 1;

%% Target cycle definition
radius = 0.2;
interAgentDistance = radius*2*sin(pi/N);

%% Draw the delta-disk for protectors and decoyers
disk_poses = repelem(x,1, length(delta_angles));
disk_poses_x_p = disk_poses(1,1:length(delta_angles)*NP) + disk_angles_x(1:length(delta_angles)*NP);
disk_poses_y_p = disk_poses(2,1:length(delta_angles)*NP) + disk_angles_y(1:length(delta_angles)*NP);
delta_handle_p = scatter(disk_poses_x_p, disk_poses_y_p, 1, 'MarkerEdgeColor',[0.7 .7 .9]);
delta_handle_p.XDataSource = 'disk_poses_x_p';
delta_handle_p.YDataSource = 'disk_poses_y_p';

disk_poses_x_d = disk_poses(1,(length(delta_angles)*NP)+1:end) + disk_angles_x((length(delta_angles)*NP)+1:end);
disk_poses_y_d = disk_poses(2,(length(delta_angles)*NP)+1:end) + disk_angles_y((length(delta_angles)*NP)+1:end);
delta_handle_d = scatter(disk_poses_x_d, disk_poses_y_d, 1, 'MarkerEdgeColor',[0.9 .7 .7]);
delta_handle_d.XDataSource = 'disk_poses_x_d';
delta_handle_d.YDataSource = 'disk_poses_y_d';

%% Draw wedge for protectors
wedge_poses = repelem(x,1,length(wedge_angles));
wedge_poses_theta = repelem(xuni(3,:),1,length(wedge_angles));
wedge_poses_theta_p = wedge_poses_theta(1,1:length(wedge_angles)*N) + wedge_angles_theta(1:length(wedge_angles)*N);
wedge_angles_x = wedge_delta.*cos(wedge_poses_theta_p);
wedge_angles_y = wedge_delta.*sin(wedge_poses_theta_p);

wedge_poses_x_p = wedge_poses(1,1:length(wedge_angles)*N) + wedge_angles_x(1:length(wedge_angles)*N); 
wedge_poses_y_p = wedge_poses(2,1:length(wedge_angles)*N) + wedge_angles_y(1:length(wedge_angles)*N);
wedge_handle_p = scatter(wedge_poses_x_p,wedge_poses_y_p,1,'MarkerEdgeColor',[0.7 .9 .7]);
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

wedge_line_handle_left = scatter(wedge_line_left_x,wedge_line_left_y,1,'MarkerEdgeColor',[0.9 .7 .7]);
wedge_line_handle_right = scatter(wedge_line_right_x,wedge_line_right_y,1,'MarkerEdgeColor',[0.9 .7 .7]);

wedge_line_handle_left.XDataSource = 'wedge_line_left_x';
wedge_line_handle_left.YDataSource = 'wedge_line_left_y';

wedge_line_handle_right.XDataSource = 'wedge_line_right_x';
wedge_line_handle_right.YDataSource = 'wedge_line_right_y';

rectangle('Position',[-1.0 -1 2.6 2], 'LineWidth', 3, 'EdgeColor', 'b');
rectangle('Position',[-1.6 -1 0.6 2], 'LineWidth', 3, 'EdgeColor', 'r' );

rectangle('Position', [0.6 0 0.01 0.10], 'FaceColor', 'b', 'LineWidth', 1, 'EdgeColor', 'b')
patch([0.6 0.6 0.54], [0.05 0.1 0.05], 'b')


%% Achieve initial conditions - protectors and decoyers
si_barrier_cert = create_si_barrier_certificate('SafetyRadius', 0.1);
circularTargetsProtectors = [flagpos(1) + 0.3*cos(0:2*pi/NP:2*pi*(1- 1/NP)) ; 0.3*sin( 0:2*pi/NP:2*pi*(1- 1/NP) )];
errorToInitialPosProtector(:, 1:NP) = x(:,1:NP) - circularTargetsProtectors;                % Error
errorNormP = [1,1]*(errorToInitialPosProtector.^2);               % Norm of error

targetsDecoyers = [-1.3 * ones(1, ND); linspace(-0.5, 0.5, ND)];
errorToInitialPosDecoyer(:, 1:ND) = x(:,decoyers) - targetsDecoyers;                % Error
errorNormD = [1,1]*(errorToInitialPosDecoyer.^2);               % Norm of error

while max(errorNormP + errorNormD) > 0.002
    % Update state variables        
    xuni = rbtm.get_poses();                            % States of real unicycle robots
    x = xuni(1:2,:);                                    % x-y positions

    % Update errors
    errorToInitialPosProtector(:, 1:NP) = x(:,1:NP) - circularTargetsProtectors;                % Error
    errorNormP = [1,1]*(errorToInitialPosProtector.^2);
    errorToInitialPosDecoyer(:, 1:ND) = x(:,NP+1:N) - targetsDecoyers;                % Error
    errorNormD = [1,1]*(errorToInitialPosDecoyer.^2);

    % Conput control inputs
    u_protector = -0.3.*errorToInitialPosProtector;
    u_decoyer = -0.3.*errorToInitialPosDecoyer;
    du = [u_protector u_decoyer];
    du = si_barrier_cert(du, xuni);
    dx = si_to_uni_dyn(du, xuni);

    % Assing new control inputs to robots
    dx = scale_velocities(dx);
    rbtm.set_velocities(1:N, dx);                       % Assign dummy zero velocity

    rbtm.step();% Run robotarium step

    disk_poses = repelem(x,1, length(delta_angles));
    disk_poses_x_p = disk_poses(1,1:length(delta_angles)*NP) + disk_angles_x(1:length(delta_angles)*NP);
    disk_poses_y_p = disk_poses(2,1:length(delta_angles)*NP) + disk_angles_y(1:length(delta_angles)*NP);
    disk_poses_x_d = disk_poses(1,(length(delta_angles)*NP)+1:end) + disk_angles_x((length(delta_angles)*NP)+1:end);
    disk_poses_y_d = disk_poses(2,(length(delta_angles)*NP)+1:end) + disk_angles_y((length(delta_angles)*NP)+1:end);

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
    
    refreshdata([delta_handle_p, delta_handle_d, wedge_handle_p,wedge_line_handle_left,wedge_line_handle_right]);
end

xuni = rbtm.get_poses();                            % States of real unicycle robots
rbtm.set_velocities(1:N, zeros(2,N));                       % Assign dummy zero velocity
rbtm.step();                                                % Run robotarium step
disp('Initial positions reached');

% Correcting the orientation for cyclic pursuit
x = xuni(1:2,:);
y = zeros(2,NP);
y(1,:) = -1.*(x(2,1:NP) - flagpos(2));
y(2,:) = (x(1,1:NP) - flagpos(1));
x_theta = xuni(3,1:NP);

x_theta_desired = atan2(y(2,:),y(1,:));
e =  x_theta_desired - x_theta; 
e_prime = atan2(sin(e),cos(e));
e_Norm = norm(e_prime); 
w = zeros(2,N);

while max(e_Norm) > 0.05
    xuni = rbtm.get_poses();
    x_theta = xuni(3,1:NP);
   
    e =  x_theta_desired - x_theta; 
    e_prime = atan2(sin(e),cos(e));
    e_Norm = norm(e_prime);
    
    w(2,1:NP) = 0.3.*e_prime;
    w = scale_velocities(w);
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

disp('Initial positions and orientations reached');


%% Switching States of Decoyers and Protectors
statesDecoyers = 0;
statesProtectors = 0;

%% Weight Matrix for Decoyers formations
d = 0.4;
Wd = [0 d d 0 0 0; d 0 d d 0 0; d d 0 d d 0; 0 d d 0 d d;0 0 0 d 0 d;0 0 0 d d 0 ];
Wdl = [0 d d; d 0 d; d d 0];

%% Iterate for the previously specified number of iterations



for t = 1:iterations    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    
    
    
    
    xuni = rbtm.get_poses();                                % Get new robots' states
    x = xuni(1:2,:);                                        % Extract single integrator states
    dx = zeros(2,N);                                           % Initialize velocities to zero

    L = updateGraph(x, xuni(3, :), disk_delta, wedge_delta, wedge_theta);
    
    %% Protectors
    
    for i = 1:NP-1
        if (norm(x(:,i+1)-x(:,i))-interAgentDistance)>0
            alpha = theta - (pi/18);
            R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
        else
            alpha = theta + (pi/18);
            R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
        end
        dx(:,i) = dx(:,i) + R*(x(:,i+1)-x(:,i))-((norm(x(:,i)-flagpos)-radius)*(x(:,i)-flagpos));        
    end
    if (norm(x(:,1)-x(:,NP))-interAgentDistance)>0
        alpha = theta - (pi/18);
        R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
    else
        alpha = theta + (pi/18);
        R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
    end
    dx(:,NP) = dx(:,NP) + R*(x(:,1)-x(:,NP))-((norm(x(:,NP)-flagpos)-radius)*(x(:,NP)-flagpos));    
    
    %k = zeros(2,1);
    
    
    %% Decoyers
    switch (statesDecoyers)
        case 0
            checkLeader1 = 0;
            checkLeader2 = 0;
        for i = NP+1 :N
            
            for j = topological_neighbors(L,i)
                dx(:,i) = dx(:,i) + (x(:,j) - x(:,i));
            end
            
            if (i==NP+1 && length(topological_neighbors(L,i))>=4)
                checkLeader1 = 1;
            end
            if (i==NP+6 && length(topological_neighbors(L,i))>=4)
                checkLeader2 = 1;
            end
        end
        
        if (checkLeader1 == 1 && checkLeader2 == 1)
            statesDecoyers = 1;
        end
            
        case 1  
            xgl1 = [0.1;0.5];
            x1greached = 0;
            if norm(xgl1 - x(:,NP+2)) >=0.01
                dx(:,NP+2) = dx(:,NP+2) + 0.5 * (xgl1 - x(:,NP+2)) / norm(xgl1 - x(:,NP+2));
            else
                x1greached = 1;
            end
            x2greached = 0;
            xgl2 = [0.1;-0.5];
            if norm(xgl1 - x(:,NP+2))>=0.01
                dx(:,NP+5) = dx(:,NP+5) + 0.5*(xgl2 - x(:,NP+5)) / norm(xgl1 - x(:,NP+2));
            else
                x2greached = 1;
            end
            % Team 1
            for i = NP+1 : NP+3
                for j = NP+1 : NP+3
                    if i~=j
                        deltaSeparation = 0.25;
                        weight = (1 - deltaSeparation / norm ( x(:,i) -  x(:,j) ) )/(disk_delta - norm ( x(:,i) -  x(:,j) ))^3;
                        dx(:,i) = dx(:,i) + 0.1 * weight *( (x(:,j) - x(:,i)));
                    end
                end
            end
            
            for i = NP+4 : NP+6
                for j = NP+4 : NP+6
                if i~=j
                        deltaSeparation = 0.25;
                        weight = (1 - deltaSeparation / norm ( x(:,i) -  x(:,j) ) )/(disk_delta - norm ( x(:,i) -  x(:,j) ))^3;
                        dx(:,i) = dx(:,i) + 0.1 * weight *( (x(:,j) - x(:,i)));
                end
                end
            end
            if(x1greached && x2greached)
                statesDecoyers = 2;
            end
            
        
        case 2
            for i = [NP+1; NP+3]'
                if(isempty(find(L(i, 1:NP))))
                    %x_theta = xuni(3,i);
                   	%x_location_desired = -x(1:2,i) + c;
                    %x_theta_desired = atan2(x_location_desired(2,:),x_location_desired(1,:));
                    %e = (x_theta_desired - x_theta);
                    %e_prime = atan2(sin(e),cos(e));
                    %e_norm = norm(e_prime);
                    %w(2,:) = 0.3.*e_prime;
                    %w = w./10;
                    dx(:, i) = flagpos - x(:, i);
                else
                    if(length(find(L(i, 1:NP))) == 1)
                        disp('case 4 else')
                        j = find(L(i, 1:NP));
                        dx(:, i) = (norm(x(:, j) - x(:, i)) - 0.15)*(x(:, j) - x(:, i));
                    end
                end
            end
    end

    du = si_barrier_cert(dx, xuni);
    dxu = si_to_uni_dyn(du, xuni);                            % Convert single integrator inputs into unicycle inputs
    dxu = scale_velocities(dxu);
    rbtm.set_velocities(1:N, dxu); 
    rbtm.step();              % Set new velocities to robots and update

    disk_poses = repelem(x,1, length(delta_angles));
    disk_poses_x_p = disk_poses(1,1:length(delta_angles)*NP) + disk_angles_x(1:length(delta_angles)*NP);
    disk_poses_y_p = disk_poses(2,1:length(delta_angles)*NP) + disk_angles_y(1:length(delta_angles)*NP);
    disk_poses_x_d = disk_poses(1,(length(delta_angles)*NP)+1:end) + disk_angles_x((length(delta_angles)*NP)+1:end);
    disk_poses_y_d = disk_poses(2,(length(delta_angles)*NP)+1:end) + disk_angles_y((length(delta_angles)*NP)+1:end);

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
    
    refreshdata([delta_handle_p, delta_handle_d, wedge_handle_p,wedge_line_handle_left,wedge_line_handle_right]);

    %% Send velocities to agents
    
    % Set velocities of agents 1,...,N
%    rbtm.set_velocities(1:N, dxu);
    
    % Send the previously set velocities to the agents.  This function must be called!
end

% We should call r.call_at_scripts_end() after our experiment is over!
rbtm.debug();
