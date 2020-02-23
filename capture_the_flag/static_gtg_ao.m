%% Initializations 
clear all;
disp(fix(clock))
N = 10;
rbtm = Robotarium('NumberOfRobots', N, 'ShowFigure', true);
NP = 6;                    % Number of protectors (also change these in other defined functions)
ND = 4;                    % Number of "decoyers"
protectors = [1:6]';       % indices for two teams
decoyers = [7:10]';
iterations = 3000;
jailPosition = [-0.5;0.5];
collision_flag=zeros(1,6);
disk_delta  = 0.5;
wedge_delta = 0.4;
wedge_theta = 2*pi/3;       
flagpos = [0.6; 0];
delta_angles = 0:0.01:2*pi;
disk_angles_x = repmat(disk_delta*cos(delta_angles),1, N);
disk_angles_y = repmat(disk_delta*sin(delta_angles),1, N);
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
radius = 0.3;
interAgentDistance = radius*2*sin(pi/NP);

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


attack_flag = 0;
%% Draw wedge for protectors
rectangle('Position',[-1.0 -1 2.6 2], 'LineWidth', 3, 'EdgeColor', 'b');
rectangle('Position',[-1.6 -1 0.6 2], 'LineWidth', 3, 'EdgeColor', 'r' );

rectangle('Position', [0.6 0 0.01 0.10], 'FaceColor', 'b', 'LineWidth', 1, 'EdgeColor', 'b')
patch([0.6 0.6 0.54], [0.05 0.1 0.05], 'b')


%% Achieve initial conditions - protectors and decoyers

si_barrier_cert = create_si_barrier_certificate('SafetyRadius', 1.5*rbtm.robot_diameter);
circularTargetsProtectors = [flagpos(1) + 0.5*cos(0:2*pi/NP:2*pi*(1- 1/NP)) ; flagpos(2) + 0.5*sin( 0:2*pi/NP:2*pi*(1- 1/NP) )];
errorToInitialPosProtector(:, 1:NP) = x(:,1:NP) - circularTargetsProtectors;                % Error
errorNormP = [1,1]*(errorToInitialPosProtector.^2);               % Norm of error

targetsDecoyers = [-1.3 * ones(1, ND); linspace(-0.8, 0.8, ND)];
errorToInitialPosDecoyer(:, 1:ND) = x(:,decoyers) - targetsDecoyers;                % Error
errorNormD = [1,1]*(errorToInitialPosDecoyer.^2);               % Norm of error

while max(max(errorNormP), max(errorNormD)) > 0.01
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
    refreshdata([delta_handle_p, delta_handle_d])%, wedge_handle_p,wedge_line_handle_left,wedge_line_handle_right]);
end

xuni = rbtm.get_poses();                            % States of real unicycle robots
rbtm.set_velocities(1:N, zeros(2,N));                       % Assign dummy zero velocity
rbtm.step();                                                % Run robotarium step
disp(fix(clock))
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

while max(e_Norm) > 0.01
    xuni = rbtm.get_poses();
    x_theta = xuni(3,1:NP);
    
    e =  x_theta_desired - x_theta; 
    e_prime = atan2(sin(e),cos(e));
    e_Norm = norm(e_prime);
    
    w(2,1:NP) = 0.3.*e_prime;
    w = scale_velocities(w);
    rbtm.set_velocities(1:N, w);                       % Assign dummy zero velocity
    rbtm.step();  
end
disp(fix(clock))
disp('Initial positions and orientations reached');


%% Switching States of Decoyers and Protectors
statesDecoyers = 1;
statesProtectors = 0;

%% Weight Matrix for Decoyers formations
d = 0.4;
Wd = [0 d d 0 0 0; d 0 d d 0 0; d d 0 d d 0; 0 d d 0 d d;0 0 0 d 0 d;0 0 0 d d 0 ];
Wdl = [0 d d; d 0 d; d d 0];

%% Iterate for the previously specified number of iterations


%for t = 1:iterations 
while(statesDecoyers < 3)
    %% Common

    xuni = rbtm.get_poses();                                % Get new robots' states
    x = xuni(1:2,:);                                        % Extract single integrator states
    dx = zeros(2,N);                                           % Initialize velocities to zero

    L = updateGraph(x, disk_delta);
   
    check_collision(x,flag,NP,N);
    %% Protectors - do nothing
    
    %% Decoyers
    switch (statesDecoyers)
 
        case 0
            disp('case 0 working')
            checkLeader1 = 0;
            checkLeader2 = 0;
            for i = NP+1 :N
                for j = topological_neighbors(L,i)
                    dx(:,i) = dx(:,i) + (x(:,j) - x(:,i));
                end
                if (i==NP+2 && length(topological_neighbors(L,i)) >= 3)
                    checkLeader1 = 1;
                end
            end 
            du = si_barrier_cert(dx, xuni);
            dxu = si_to_uni_dyn(dx, xuni);                            % Convert single integrator inputs into unicycle inputs
            dxu = scale_velocities(dxu);
            if (checkLeader1 == 1)
                disp(fix(clock))
                disp('decoyer aligning orientation');                
                statesDecoyers = 1;
            end
            
        %% add an additional case for the orientation alignment (Nitin)
        case 1
            x_theta = xuni(3,NP+2);
            vector_direction = [0.6-x(1,NP+2);0-x(2,NP+2)];
            x_theta_desired = atan2(vector_direction(2),vector_direction(1));
            e = x_theta_desired - x_theta;
            e_prime = atan2(sin(e),cos(e));
            e_Norm = norm(e_prime); 
            dx(2,NP+2) = 0.3.*e_prime;
            dxu = si_barrier_cert(dx, xuni);
            dxu = scale_velocities(dxu);
            dxu = dxu./3;
            
            if e_Norm < 0.2
                disp(fix(clock))
                disp('decoyer attacking')
                statesDecoyers = 2;
            end

        case 2
            if norm([0.6; 0] - x(:, NP+2)) > 0.05
                dx(:, NP+2) = [0.6; 0] - x(:, NP+2);
                opp = find_opponents(L, NP+2, NP);
                for j = opp
                    disp(fix(clock))
                    disp('decoyer avoiding obstacles')
                    err = x(:, j) - x(:, NP+2);
                    K = 000.02/(norm(err)*(norm(err)^2 + 0.00001));
                    dx(:, NP+2) = dx(:, NP+2) + K*(-err);
                end
            else
                statesDecoyers = 3;
            end 
            du = si_barrier_cert(dx, xuni);
            dxu = si_to_uni_dyn(du, xuni);                            % Convert single integrator inputs into unicycle inputs
            dxu = scale_velocities(dxu);
    end
    
              % Set new velocities to robots and update

    disk_poses = repelem(x,1, length(delta_angles));
    disk_poses_x_p = disk_poses(1,1:length(delta_angles)*NP) + disk_angles_x(1:length(delta_angles)*NP);
    disk_poses_y_p = disk_poses(2,1:length(delta_angles)*NP) + disk_angles_y(1:length(delta_angles)*NP);
    disk_poses_x_d = disk_poses(1,(length(delta_angles)*NP)+1:end) + disk_angles_x((length(delta_angles)*NP)+1:end);
    disk_poses_y_d = disk_poses(2,(length(delta_angles)*NP)+1:end) + disk_angles_y((length(delta_angles)*NP)+1:end);
    refreshdata([delta_handle_p, delta_handle_d])
    rbtm.set_velocities(1:N, dxu); 
    rbtm.step();
end


rbtm.debug();
