%% STATIC GO TO GOAL
% Capture the Flag Game
% Attacker tries to capture the flag while avoiding obstacles in the
% presence of non reactive defenders in a static circular formation
% 11/30/2018

%% Set up Robotarium object
clear all, clc
N = 10;
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);
disp(fix(clock))

%Run the simulation for a specific number of iterations
iterationsInitialPosition = 800;
iterationsInitialOrientation = 800;
iterations = 1000;

%% Draw Arena
rectangle('Position',[-1.0 -1 2.6 2], 'LineWidth', 3, 'EdgeColor', 'b');
rectangle('Position',[-1.6 -1 0.6 2], 'LineWidth', 3, 'EdgeColor', 'r' );

rectangle('Position', [0.6 0 0.01 0.10], 'FaceColor', 'b', 'LineWidth', 1, 'EdgeColor', 'b')
patch([0.6 0.6 0.54], [0.05 0.1 0.05], 'b')


%% Initializations
NP = 6;                    % Number of protectors
ND = 4;                    % Number of decoyers
flagpos = [0.6; 0];        % Position of Flag 
protectors = (1:6)';       % indices for two teams
decoyers = (7:10)';
jailPosition = [-0.5;0.5]; % Position where the agents go to once tagged
collision_flag=zeros(1,6);
disk_delta  = 0.6;
delta_angles = 0:0.01:2*pi;
disk_angles_x = repmat(disk_delta*cos(delta_angles),1, N);
disk_angles_y = repmat(disk_delta*sin(delta_angles),1, N);





%% Draw the delta-disk for protectors and decoyers
x = r.get_poses();
disk_poses = repelem(x,1, length(delta_angles));
disk_poses_x_p = disk_poses(1,1:length(delta_angles)*NP) + disk_angles_x(1:length(delta_angles)*NP);
disk_poses_y_p = disk_poses(2,1:length(delta_angles)*NP) + disk_angles_y(1:length(delta_angles)*NP);
delta_handle_p = scatter(disk_poses_x_p, disk_poses_y_p, 1, 'MarkerEdgeColor',[0.7 .7 .9],'LineWidth',0.2);
delta_handle_p.XDataSource = 'disk_poses_x_p';
delta_handle_p.YDataSource = 'disk_poses_y_p';

disk_poses_x_d = disk_poses(1,(length(delta_angles)*NP)+1:end) + disk_angles_x((length(delta_angles)*NP)+1:end);
disk_poses_y_d = disk_poses(2,(length(delta_angles)*NP)+1:end) + disk_angles_y((length(delta_angles)*NP)+1:end);
delta_handle_d = scatter(disk_poses_x_d, disk_poses_y_d, 1, 'MarkerEdgeColor',[0.9 .7 .7], 'LineWidth',0.2 );
delta_handle_d.XDataSource = 'disk_poses_x_d';
delta_handle_d.YDataSource = 'disk_poses_y_d';
r.step();



%% Target Initial Conditions
% Protectors
radius = 0.4;
interAgentDistance = 2*radius*sin(pi/NP);
theta = pi/NP;
targetAgentsInitialOrientation = zeros(2,N);
targetAgentsInitialPosition = zeros(2,N);

targetAgentsInitialPosition(:, 1:NP) = [flagpos(1) + radius * cos(0:2*pi/NP:2*pi*(1- 1/NP)) ; flagpos(2) + radius * sin( 0:2*pi/NP:2*pi*(1- 1/NP) )];
for i = 1:NP-1
    targetAgentsInitialOrientation(:,i) = targetAgentsInitialPosition(:,i+1);
    % y(:,i) = flagpos;
end
targetAgentsInitialOrientation(:,NP) = targetAgentsInitialPosition(:,1);

% Decoyers
targetAgentsInitialPosition(:, NP+1:N) = [-1.3 * ones(1, ND); linspace(-0.8, 0.8, ND)];

for i = NP+1:N
    targetAgentsInitialOrientation(:,i) = flagpos;
end

%% Set up constants for experiments
% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
dx = zeros(2, N);

lambda = 0.05;                  % Projection Distance
safety = 1.5*r.robot_diameter;  % Safety Radius

%% Tools to map single-integrator -> unicycle

% Get the tools we need to map from single-integrator
[si_to_uni_dyn, uni_to_si_states] = create_si_to_uni_mapping('ProjectionDistance', lambda);

% Grab barrier certificates for unicycle dynamics
uni_barrier_cert = create_uni_barrier_certificate('SafetyRadius', safety, 'ProjectionDistance', lambda);

% Grab a position controller for single-integrator systems
si_pos_controller = create_si_position_controller();

% Grab a position controller for unicycle systems
uni_pos_controller = create_unicycle_position_controller();

%Iterate for the previously specified number of iterations
for t = 1:iterationsInitialPosition
    
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    %% Algorithm
      
        
    % Convert to single-integrator domain 
    x_int = uni_to_si_states(x);
    
    %Currently in integrator dynamics
    dx = si_pos_controller(x_int, targetAgentsInitialPosition);
    
    % Threshold velocities for safety
    dxmax = 0.2;
    for i = 1:N
        if norm(dx(:,i)) > dxmax
            dx(:,i) = dx(:,i)/norm(dx(:,i))*dxmax;
        end
    end
    
    
    % Map to unicycle dynamics
    dx = si_to_uni_dyn(dx, x);    
    
    %Ensure the robots don't collide
    dx = uni_barrier_cert(dx, x);    
    
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dx);
    
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();
    
    disk_poses = repelem(x,1, length(delta_angles));
    disk_poses_x_p = disk_poses(1,1:length(delta_angles)*NP) + disk_angles_x(1:length(delta_angles)*NP);
    disk_poses_y_p = disk_poses(2,1:length(delta_angles)*NP) + disk_angles_y(1:length(delta_angles)*NP);
    disk_poses_x_d = disk_poses(1,(length(delta_angles)*NP)+1:end) + disk_angles_x((length(delta_angles)*NP)+1:end);
    disk_poses_y_d = disk_poses(2,(length(delta_angles)*NP)+1:end) + disk_angles_y((length(delta_angles)*NP)+1:end);
    refreshdata([delta_handle_p, delta_handle_d]);
    
    if abs(max(targetAgentsInitialPosition-x(1:2, 1:N),1)) < 0.15 
        break;
    end
    
end

disp('INITIAL POSITIONS REACHED');
disp(fix(clock));

%Iterate for the previously specified number of iterations
for t = 1:iterationsInitialOrientation
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    
    
    %% Algorithm
  
        
    %Currently in integrator dynamics
    dx = uni_pos_controller(x(:,1:N), targetAgentsInitialOrientation);
    
    % Threshold velocities for safety
    dxmax = 0.2;
    for i = 1:N
        if norm(dx(:,i)) > dxmax
            dx(:,i) = dx(:,i)/norm(dx(:,i))*dxmax;
        end
    end
    
    % Map to unicycle dynamics
    %Ensure the robots don't collide
    dx = uni_barrier_cert(dx, x);    
    dx(1,:) = zeros(1,N);
    
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dx);
    r.step();
    
    disk_poses = repelem(x,1, length(delta_angles));
    disk_poses_x_p = disk_poses(1,1:length(delta_angles)*NP) + disk_angles_x(1:length(delta_angles)*NP);
    disk_poses_y_p = disk_poses(2,1:length(delta_angles)*NP) + disk_angles_y(1:length(delta_angles)*NP);
    disk_poses_x_d = disk_poses(1,(length(delta_angles)*NP)+1:end) + disk_angles_x((length(delta_angles)*NP)+1:end);
    disk_poses_y_d = disk_poses(2,(length(delta_angles)*NP)+1:end) + disk_angles_y((length(delta_angles)*NP)+1:end);
    refreshdata([delta_handle_p, delta_handle_d]);
    
    % Send the previously set velocities to the agents.  This function must be called!
    if abs(max(targetAgentsInitialOrientation-x(1:2, 1:N),1))<0.15
         break;
    end
    
    
end

disp('INITIAL ORIENTATIONS REACHED');
disp(fix(clock));

%% Draw the connectivity for protectors and decoyers
k = 1;

linePlotHandleProtectors = zeros(N,1);
linePlotHandleDecoyers = zeros(N,1);
linePlotHandlePD = zeros(N,1);
        
for i = 1:N
    for j = i+1:N
        linePlotHandleProtectors(k) = plot([x(1,i) x(1,i)], [x(2,i) x(2,i)], 'b', 'LineWidth', 2);
        k  = k+1;
    end
end
k  = 1;
for i = 1:N
    for j = i+1:N
        linePlotHandleDecoyers(k) = plot([x(1,i) x(1,i)], [x(2,i) x(2,i)], 'r', 'LineWidth', 2);
        k  = k+1;
    end
end

statesProtectors = 0;

%Iterate for the previously specified number of iterations
for t = 1:iterations
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    % Convert to single-integrator domain 
%     x_int = uni_to_si_states(x);
    x_int = x(1:2,:);
    dx = zeros(2,N);
    
    %% Algorithm
  
    L = updateGraph(x_int, disk_delta);
    
    LP = zeros(N);
    LD = zeros(N);
    LPD = zeros(N);
    
    LP(1:NP,1:NP) = L(1:NP,1:NP);
    LD(NP+1:N,NP+1:N) = L(NP+1:N,NP+1:N);
    LPD(1:NP,NP+1:N) = L(1:NP,NP+1:N);
    
    Lpd = zeros(N);
    Lpd(1:NP,NP+1:N) = L(1:NP,NP+1:N);
    [protectorsInvolved, decoyersInvolved] = find(Lpd);
    barrierGoal(1,1) = sum( x(1,decoyersInvolved) ) / length(decoyersInvolved);
    barrierGoal(2,1) = sum( x(2,decoyersInvolved) ) / length(decoyersInvolved);
    
    % Chcek if there is any collision   
%     check_collision(x_int,flag,NP,N)
%     if  ~(isempty (protectorsInvolved) ) 
%         % Checking which agent sees maximum number of decoyers
%         numberOfDecoyersEngaged = zeros(NP,1);
%         for iter = 1:NP
%             numberOfDecoyersEngaged(iter) = abs ( sum (Lpd(iter,:)) );
%         end
%         
%         [MaxDecoyers, MaxEngagedProtector] = max (numberOfDecoyersEngaged);
%         % Do connectivity maintenance with MaxEngagedProtector, MaxEngagedProtector + 1, MaxEngagedProtector - 1
%         % Also run a check for if MaxEngagedProtector = NP then
%         
%         centreDefenceProtector = MaxEngagedProtector;
%         % Do cyclic pursuit
%         % Find agents not doing defence and run cyclic pursuit with them
%         
%         if MaxEngagedProtector == NP
%             defenceProtectors = [ MaxEngagedProtector ; 1 ; MaxEngagedProtector-1];
%         elseif MaxEngagedProtector == 1
%                 defenceProtectors = [MaxEngagedProtector ; MaxEngagedProtector+1; NP];
%         else
%                 defenceProtectors = [MaxEngagedProtector;MaxEngagedProtector+1;MaxEngagedProtector-1];
%         end
%         
%         p = ismember(protectors,defenceProtectors);
%         cycleProtectors = protectors(~p); 
%         NP_new = length(cycleProtectors);
%         
%         % Set statesProtectors to case 1
%         statesProtectors = 1;
%         
%     else
%         protectorsInvolved = 1:NP;
%         statesProtectors = 0;
%     end
        
    distanceToCenter = 0.2;
    distanceBetweenAgents = 2*distanceToCenter*sin(pi/(NP/2));
    switch(statesProtectors)
       
        
       case 0
           % Run the cyclic pursuit here with 6 agents
                    
           
           for i = 1:NP-1
               if (norm(x_int(:,i+1)-x_int(:,i))-interAgentDistance)>0
                   alpha = theta - (pi/18);
                   R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
               else
                   alpha = theta + (pi/18);
                   R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
               end
               dx(:,i) = dx(:,i) + R*(x_int(:,i+1)-x_int(:,i))-((norm(x_int(:,i)-flagpos)-radius)*(x_int(:,i)-flagpos));        
            end
            if (norm(x_int(:,1)-x_int(:,NP))-interAgentDistance)>0
                alpha = theta - (pi/18);
                R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
            else
                alpha = theta + (pi/18);
                R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
            end
            dx(:,NP) = dx(:,NP) + R*(x_int(:,1)-x_int(:,NP))-((norm(x_int(:,NP)-flagpos)-radius)*(x_int(:,NP)-flagpos));
            dx(:,1:NP) = si_to_uni_dyn(dx(:,1:NP), x(:,1:NP)); 
            
            % Check
            if  ~(isempty (protectorsInvolved) ) 

                numberOfDecoyersEngaged = zeros(NP,1);
                for iter = 1:NP
                    numberOfDecoyersEngaged(iter) = abs ( sum (Lpd(iter,:)) );
                end
                [MaxDecoyers, MaxEngagedProtector] = max (numberOfDecoyersEngaged);
                centreDefenceProtector = MaxEngagedProtector;
                if MaxEngagedProtector == NP
                defenceProtectors = [ MaxEngagedProtector ; 1 ; MaxEngagedProtector-1];
                elseif MaxEngagedProtector == 1
                        defenceProtectors = [MaxEngagedProtector ; MaxEngagedProtector+1; NP];
                else
                        defenceProtectors = [MaxEngagedProtector;MaxEngagedProtector+1;MaxEngagedProtector-1];
                end

                p = ismember(protectors,defenceProtectors);
                cycleProtectors = protectors(~p); 
                NP_new = length(cycleProtectors);
                statesProtectors = 1;
            end
            
       case 1
          for i=1: length(cycleProtectors) - 1
              if (norm(x_int(:,cycleProtectors(i+1))-x_int(:,cycleProtectors(i)))-distanceBetweenAgents)>0
                   alpha = theta - (pi/18);
                   R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
               else
                   alpha = theta + (pi/18);
                   R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
              end
              dx(:,cycleProtectors(i)) = dx(:,cycleProtectors(i)) + R*(x_int(:,cycleProtectors(i+1))-x_int(:,cycleProtectors(i)))-((norm(x_int(:,cycleProtectors(i))-flagpos)-distanceToCenter)*(x_int(:,cycleProtectors(i))-flagpos));
          end
          if (norm(x_int(:,cycleProtectors(1))-x_int(:,cycleProtectors(NP_new)))-distanceBetweenAgents)>0
                alpha = theta - (pi/18);
                R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
          else
                alpha = theta + (pi/18);
                R = [cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
          end
          dx(:,cycleProtectors(length(cycleProtectors))) = dx(:,cycleProtectors(length(cycleProtectors))) + R*(x_int(:,cycleProtectors(1))-x_int(:,cycleProtectors(length(cycleProtectors))))-((norm(x_int(:,cycleProtectors(length(cycleProtectors)))-flagpos)-distanceToCenter)*(x_int(:,cycleProtectors(length(cycleProtectors)))-flagpos));      
          
%           w12 = norm ( x(:,defenceProtectors(1)) -  x(:,defenceProtectors(2)) ) - 0.2;
%           w13 = norm ( x(:,defenceProtectors(1)) -  x(:,defenceProtectors(3)) ) - 0.2;
%  
%           dx(:,defenceProtectors(1)) = dx(:,defenceProtectors(1)) +  ( barrierGoal - x_int(:,centreDefenceProtector) ) + 0.5 * w12 * ( x_int(:,defenceProtectors(2)) - x_int(:,defenceProtectors(1)) ) + 0.5 * w13 * ( x_int(:,defenceProtectors(3)) - x_int(:,defenceProtectors(1)) );
%           dx(:,defenceProtectors(2)) = dx(:,defenceProtectors(2))  + w12 * ( x_int(:,defenceProtectors(1)) - x_int(:,defenceProtectors(2)) ) ;
%           dx(:,defenceProtectors(3)) = dx(:,defenceProtectors(3))  + w13 * ( x_int(:,defenceProtectors(1)) - x_int(:,defenceProtectors(3)) );

          
          dx(:,defenceProtectors(1)) = dx(:,defenceProtectors(1)) + ( barrierGoal - x_int(:,centreDefenceProtector) ) + 0.5 * norm(x_int(:,defenceProtectors(2))-x_int(:,defenceProtectors(1))-0.2) * ( x_int(:,defenceProtectors(2)) - x_int(:,defenceProtectors(1)) ) + 0.5 * norm(x_int(:,defenceProtectors(3))-x_int(:,defenceProtectors(1))-0.2) * ( x_int(:,defenceProtectors(3)) - x_int(:,defenceProtectors(1)) );
          dx(:,defenceProtectors(2)) = dx(:,defenceProtectors(2))  + 0.5 * norm(x_int(:,defenceProtectors(1))-x_int(:,defenceProtectors(2))-0.2) * ( x_int(:,defenceProtectors(1)) - x_int(:,defenceProtectors(2)) ) + 0.5 * norm(x_int(:,defenceProtectors(3))-x_int(:,defenceProtectors(2))-0.2) * ( x_int(:,defenceProtectors(3)) - x_int(:,defenceProtectors(2)) );
          dx(:,defenceProtectors(3)) = dx(:,defenceProtectors(3))  + 0.5 * norm(x_int(:,defenceProtectors(1))-x_int(:,defenceProtectors(3))-0.2) * ( x_int(:,defenceProtectors(1)) - x_int(:,defenceProtectors(3)) ) + 0.5 * norm(x_int(:,defenceProtectors(2))-x_int(:,defenceProtectors(3))-0.2) * ( x_int(:,defenceProtectors(2)) - x_int(:,defenceProtectors(3)) );    
          dist = norm(barrierGoal - x_int(:,centreDefenceProtector))
          dx(:,1:NP) = si_to_uni_dyn(dx(:,1:NP), x(:,1:NP));
          if (norm(barrierGoal - x_int(:,centreDefenceProtector)))<0.4
              statesProtectors = 2;
          end
        case 2
            dx(:,1:NP) = si_pos_controller(x_int(:,1:NP), targetAgentsInitialPosition(:,1:NP));
            if abs(max(targetAgentsInitialPosition-x(1:2, 1:N),1)) < 0.15 
                statesProtectors = 3;
            end
        case 3
            dx(:,1:NP) = uni_pos_controller(x_int(:,1:NP), targetAgentsInitialOrientation);
            if abs(max(targetAgentsInitialOrientation-x(1:2, 1:N),1)) < 0.15
               statesProtectors = 0;
            end          
    end

    
    % One decoyer attacks
    if norm(flagpos - x_int(:,NP+2))>0.1
        dx(:,NP+2) = 1.5 * (flagpos - x_int(:,NP+2));
        opp = find_opponents(L, NP+2, NP);
                for j = opp
                    err = (x_int(:, j) - x_int(:, NP+2));
                    K = 000.02/(norm(err)*(norm(err)^2 + 0.00001));
                    % This is single-integrator dynamics, need to convert
                    % to unicycle dynamics
                    dx(:, NP+2) = dx(:, NP+2) + K *(-err);
                end
    else
        break;
    end
    
    %Currently in integrator dynamics
    dx(:,NP+2) = dx(:,NP+2) +  uni_pos_controller(x(:,NP+2), targetAgentsInitialOrientation(:,NP+2));
    
    dxmax = 0.1;
    dx(:,NP+2) = dx(:,NP+2)/norm(dx(:,NP+2))*dxmax;
    
    % Threshold velocities for safety
    
%     for i = 1:N
%         if norm(dx(:,i)) > dxmax
%             dx(:,i) = dx(:,i)/norm(dx(:,i))*dxmax;
%         end
%     end
    
    % Map to unicycle dynamics
    % dx = si_to_uni_dyn(dx, x);    
%     disp(dx);
    collision_flag=check_collision(x_int,collision_flag,NP,N);
    for kk=1:6
            if(collision_flag(kk)>0)
                dx(:,NP+kk)= jailPosition - x_int(:,NP+kk);
            end
    end
    %Ensure the robots don't collide
    dx = uni_barrier_cert(dx, x);    
%     dx(:,1:NP) = zeros(2,NP);
    
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dx);
    r.step();
    
    k = 1;    
    for i = 1:NP
        for j = i+1:NP
            if LP(i,j) == -1
                set (linePlotHandleProtectors(k), 'xdata', [x(1,i) x(1,j)], 'ydata',  [x(2,i) x(2,j)]);
            else    
                set (linePlotHandleProtectors(k), 'xdata', [x(1,i) x(1,i)], 'ydata',  [x(2,i) x(2,i)]);
            end
            k  = k+1;
        end
    end
    k  = 1;
    for i = NP+1:N
        for j = i+1:N
            if LD(i,j) == -1
                set (linePlotHandleDecoyers(k), 'xdata', [x(1,i) x(1,j)], 'ydata',  [x(2,i) x(2,j)]);
                
            else    
                set (linePlotHandleDecoyers(k), 'xdata', [x(1,i) x(1,i)], 'ydata',  [x(2,i) x(2,i)]);
                
            end
            
            k  = k+1;
        end
    end
    disk_poses = repelem(x,1, length(delta_angles));
    disk_poses_x_p = disk_poses(1,1:length(delta_angles)*NP) + disk_angles_x(1:length(delta_angles)*NP);
    disk_poses_y_p = disk_poses(2,1:length(delta_angles)*NP) + disk_angles_y(1:length(delta_angles)*NP);
    disk_poses_x_d = disk_poses(1,(length(delta_angles)*NP)+1:end) + disk_angles_x((length(delta_angles)*NP)+1:end);
    disk_poses_y_d = disk_poses(2,(length(delta_angles)*NP)+1:end) + disk_angles_y((length(delta_angles)*NP)+1:end);
    refreshdata([delta_handle_p, delta_handle_d]);
    
    
end

disp('SUCCESSFUL COMPLETION OF EXPERIMENTS');
disp(fix(clock));

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();