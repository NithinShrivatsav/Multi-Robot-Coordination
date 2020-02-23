%% Initializations 
clear all;
N = 10;
rbtm = Robotarium('NumberOfRobots', N, 'ShowFigure', true);
NP = 6;                    % Number of protectors (also change these in other defined functions)
ND = 4;                    % Number of "decoyers"
protectors = [1:6]';       % indices for two teams
decoyers = [7:10]';
iterations = 1000;
jailPosition = [-0.5;0.5];
collision_flag=zeros(1,6);
disk_delta  = 0.4;
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

targetsDecoyers = [-1.3 * ones(1, ND); linspace(-0.3, 0.3, ND)];
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
    %% Common

    xuni = rbtm.get_poses();                                % Get new robots' states
    x = xuni(1:2,:);                                        % Extract single integrator states
    dx = zeros(2,N);                                           % Initialize velocities to zero

    L = updateGraph(x, disk_delta);
   
    check_collision(x,flag,NP,N);
    

    %% Protectors
    statesProtectors = 0;
    switch(statesProtectors)
        case 0
            % Cyclic Pursuit with 6 agents
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
        case 1
            
            % Cyclic Pursuit for 3 agents
            for i=1: length(cycleProtectors) - 1
              if (norm(x(:,cycleProtectors(i+1))-x(:,cycleProtectors(i)))-distanceBetweenAgents)>0
                   alpha = theta - (pi/18);
                   R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
               else
                   alpha = theta + (pi/18);
                   R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
              end
              dx(:,cycleProtectors(i)) = dx(:,cycleProtectors(i)) + R*(x(:,cycleProtectors(i+1))-x(:,cycleProtectors(i)))-((norm(x(:,cycleProtectors(i))-flagpos)-distanceToCenter)*(x(:,cycleProtectors(i))-flagpos));
            end
            if (norm(x(:,cycleProtectors(1))-x(:,cycleProtectors(NP_new)))-distanceBetweenAgents)>0
                alpha = theta - (pi/18);
                R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
            else
                alpha = theta + (pi/18);
                R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
            end
            dx(:,cycleProtectors(length(cycleProtectors))) = dx(:,cycleProtectors(length(cycleProtectors))) + R*(x(:,cycleProtectors(1))-x(:,cycleProtectors(length(cycleProtectors))))-((norm(x(:,cycleProtectors(length(cycleProtectors)))-flagpos)-distanceToCenter)*(x(:,cycleProtectors(length(cycleProtectors)))-flagpos));
            
            % Defence Protectors
            deltaSeparation = 0.15;
             % w12 = (1 - (deltaSeparation / norm ( x(:,defenceProtectors(2)) -  x(:,defenceProtectors(1)) )) )/(disk_delta - norm ( x(:,defenceProtectors(1)) -  x(:,defenceProtectors(2)) ))^3;
             % w23 = (1 - (deltaSeparation / norm ( x(:,defenceProtectors(2)) -  x(:,defenceProtectors(3)) )) )/(disk_delta - norm ( x(:,defenceProtectors(2)) -  x(:,defenceProtectors(3)) ))^3;
 
            w12 = norm ( x(:,defenceProtectors(2)) -  x(:,defenceProtectors(1)) ) - 0.2;
            w23 = norm ( x(:,defenceProtectors(2)) -  x(:,defenceProtectors(3)) ) - 0.2;
 
            dx(:,defenceProtectors(2)) = dx(:,defenceProtectors(2)) +  ( barrierGoal - x(:,centreDefenceProtector) ) + 0.5 * w12 * ( x(:,defenceProtectors(1)) - x(:,defenceProtectors(2)) ) + 0.5 * w23 * ( x(:,defenceProtectors(3)) - x(:,defenceProtectors(2)) );
            dx(:,defenceProtectors(1)) = dx(:,defenceProtectors(1))  + w12 * ( x(:,defenceProtectors(2)) - x(:,defenceProtectors(1)) );
            dx(:,defenceProtectors(3)) = dx(:,defenceProtectors(3))  + w23 * ( x(:,defenceProtectors(2)) - x(:,defenceProtectors(3)) );
    end  
    
    
    %% Decoyers
    switch (statesDecoyers)
        case 0
            disp('case 0 working')
            checkLeader1 = 0;
            checkLeader2 = 0;
            consensus_flag = zeros(1, ND);
            for i = NP+1 :N
                for j = topological_neighbors(L,i)
                    dx(:,i) = dx(:,i) + (x(:,j) - x(:,i));
                end
                if (length(topological_neighbors(L,i)) >= ND-1)
                    consensus_flag(i-NP) = 1;
                end
            end        
            if (length(find(consensus_flag == 0)) < 1)
                statesDecoyers = 1
            end

        case 1
            subteam_reached_flag = [0 1];
            if norm([0.6; 0] - x(:, NP+4)) > 0.02
                dx(:, NP+4) = [0.45; 0.8] - x(:, NP+4);
            end                
            subteam_goals = [0.0, 0.1; -0.6, 0.5];
            leaders = [NP+2];
            for i = 1:length(leaders)                
                if norm(subteam_goals(:,i) - x(:,leaders(i))) >=0.01
                    dx(:,leaders(i)) = dx(:,leaders(i)) + 0.5*(subteam_goals(:, i) - x(:,leaders(i)))/norm(subteam_goals(:, i) - x(:,leaders(i)));
                else
                    subteam_reached_flag(i) = 1;
                end
            end
            % Team 1
            for i = NP+1 : NP+3
                for j = NP+1 : NP+3
                    if i~=j
                        deltaSeparation = 0.18;
                        weight = (1 - (deltaSeparation / norm ( x(:,i) -  x(:,j) )) )/(disk_delta - norm ( x(:,i) -  x(:,j) ))^3;
                        dx(:,i) = dx(:,i) + 0.1 * weight * ((x(:,j) - x(:,i)));
                    end
                end
            end

            statesDecoyers = 1 + prod(subteam_reached_flag);
            if(statesDecoyers > 1)
                decoy_time = cputime;
            end

        case 2
            disp('reached the destination for attack');
            for i = [NP+4]'
                if(isempty(find(L(i, 1:NP))))
                    dx(:, i) = flagpos - x(:, i);
                else
                    decoypoint = [-1; 0]
                    if(length(find(L(i, 1:NP))) == 1)
                        disp('case 4 else')
                        j = find(L(i, 1:NP));
                        dx(:, i) = (norm(x(:, j) - x(:, i)) - 0.15)*(x(:, j) - x(:, i)) -0.2*(decoypoint - x(:, i))/norm(decoypoint - x(:, i));
                    end
                end
            end
%{
            if(cputime - decoy_time > 10)
                leaders = [NP+2, NP+5];
                dest = flagpos;
                uao = zeros(2, N);
                ug = zeros(2, N);
                % FILL THIS PART!!!
                for i = leaders
                    er = dest - x(:, i);
                    Kg = 2*(1 - exp(-((norm(er))^2)))/norm(er);
                    ug(:, i) = Kg*er;
                    for j = find_opponents(L, i)
                         err =  x(:, i) - x(:, j);
                         Kao = 000.001/norm(err)*(norm(err)^2 + 0.0001);
                         uao(:, i) =   uao(:, i)+ Kao*(err);
                    end
                    dx(:, i) = ug(:, i)+uao(:, i);
                end
            end
%}
    end
%   collision_flag=check_collision(x,collision_flag,NP,N);
%   for kk=1:6
%            if(collision_flag(kk)>0)
%                disp('collision was detected');
%                dx(:,NP+kk)= jailPosition - x(:,NP+kk);
%            end
%       
%   end
   
   %if (is_flag_captured(x, flagpos))
    %   rectangle('Position',[-1.0 -1 2.6 2], 'LineWidth', 3, 'FaceColor', 'r');
     %  disp ('FLAG CAPTURED');
      % break;
   %end
    
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
    refreshdata([delta_handle_p, delta_handle_d])%, wedge_handle_p,wedge_line_handle_left,wedge_line_handle_right]);

end

rbtm.debug();
