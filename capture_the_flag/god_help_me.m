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
radius = 0.25;
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

for t=1:iterations 
    xuni = rbtm.get_poses();                                % Get new robots' states
    x = xuni(1:2,:);                                        % Extract single integrator states
    dx = zeros(2,N);                                           % Initialize velocities to zero
    w = zeros(2,N);

    L = updateGraph(x, disk_delta);
    
    %% Protectors
    
    %% Protectors
    Lpd = zeros(N);
    Lpd(1:NP,NP+1:N) = L(1:NP,NP+1:N);
    [protectorsInvolved, decoyersInvolved] = find(Lpd);
    barrierGoal(1,1) = sum( x(1,decoyersInvolved) ) / length(decoyersInvolved);
    barrierGoal(2,1) = sum( x(2,decoyersInvolved) ) / length(decoyersInvolved);
    
    if  ~(isempty (protectorsInvolved) ) 
        % Checking which agent sees maximum number of decoyers
        numberOfDecoyersEngaged = zeros(NP,1);
        for iter = 1:NP
            numberOfDecoyersEngaged(iter) = abs ( sum (Lpd(iter,:)) );
        end
        
        [MaxDecoyers, MaxEngagedProtector] = max (numberOfDecoyersEngaged);
        % Do connectivity maintenance with MaxEngagedProtector, MaxEngagedProtector + 1, MaxEngagedProtector - 1
        % Also run a check for if MaxEngagedProtector = NP then
        
        centreDefenceProtector = MaxEngagedProtector;
        % Do cyclic pursuit
        % Find agents not doing defence and run cyclic pursuit with them
        
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
        
        % Set statesProtectors to case 1
        statesProtectors = 2;
        
    else
        protectorsInvolved = 1:NP;
        statesProtectors = 0;
        % Do cyclic pursuit with 6 agents
        % cycleProtectors =
        % Set statesProtectors to case 0
    end
        
    distanceToCenter = 0.2;
    distanceBetweenAgents = 2*distanceToCenter*sin(pi/(NP/2));
    switch(statesProtectors)
       
        
       case 0
           % Run the cyclic pursuit here with 6 agents
                    
           
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
          % Make the 3 cyclic pursuit agents do a consensus until each of them
          % see 2 neighbors
          % If they see 2 neighbors they stop
          
          % Make the 3 defence agents run a consensus on headings 
          % with respect to the position of the attackers
          % Switch from this state to the next state when both the
          % conditions are met
          distanceToCenter = 0.2;
          distanceBetweenAgents = 2*distanceToCenter*sin(pi/(NP));
          
          for i= 1:NP
              for j= 1:NP
                  if i~=j
                      dx(:,i) = dx(:,i) + (norm(flagpos-x(:,i))-distanceToCenter).*(flagpos-x(:,i)) + (norm(x(:,j)-x(:,i))-distanceBetweenAgents).*(x(:,j)-x(:,i));
                  end
              end
          end
          
                
          
          
       %   e_prime = zeros(3,1);
        %  x_theta = zeros(3,1);
          
         % x_location_desired = barrierGoal ; 
          
          %for iterateOver = defenceProtectors
           %x_theta(iterateOver) = xuni(3,iterateOver);
          % x_theta_desired = atan2(x_location_desired(2),x_location_desired(1));
           %e = (x_theta_desired - x_theta(iterateOver));
           %e_prime(iterateOver) = atan2(sin(e),cos(e));
           %w(2,iterateOver) = 0.3.*e_prime(iterateOver);
           %w(2,iterateOver) = w(2,iterateOver)./10;
            
          %end
          %flag1 = 0;
          %flag2 = 0;
          
          %if abs(( max(e_prime) )<0.01)
          %    flag1 = 1;
          %end
          
          if norm(x(:,cycleProtectors(1))-flagpos)<0.15 && norm(cycleProtectors(2))<0.15 && norm(cycleProtectors(3))<0.15
             statesProtectors = 2;
          end
          
          %if (flag1 == 1 && flag2 ==1)
%               statesProtectors = 2;
          %end
          
       case 2
          
          % In this case run cyclic pursuit with the 3 designated agents
          
          % The defence agents try to move towards the attackers (GO TO GOAL)
          % Here you might also have to run consensus on headings with
          % regards to attackers at every stage, in which case we can
          % separate the previous case into 2 parts and proceed where every
          % time headings get changed and then they move towards it. Not
          % sure about this part though
          
          % Cyclic Pursuit Agents
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
         
          
          
          % Defence Agents
          % CHECK TOPOLOGICAL NEIGHBOURS IMPLEMENTATION ELSE USE THE IDEA
          % OF TEAMS
          
%           for i = 1:length(defenceProtectors)
%               for j = 1:length(defenceProtectors)
%                   if i~=j 
% %                     deltaSeparation = 0.15;
% %                     weight = (1 - deltaSeparation / norm ( x(:,i) -  x(:,j) ) )/(disk_delta - norm ( x(:,i) -  x(:,j) ))^3;
% %                     weight = norm(x(:,j)-x(:,i)-0.2)
%                       dx(:,defenceProtectors(i)) = dx(:,defenceProtectors(i)) + 0.5 * (norm(x(:,defenceProtectors(j))-x(:,defenceProtectors(i)))-0.2) * ( x(:,defenceProtectors(j)) - x(:,defenceProtectors(i)) );
%                   end
%                   if i==1
%                       
%               end
%           end
          
          dx(:,defenceProtectors(1)) = dx(:,defenceProtectors(1)) + ( barrierGoal - x(:,centreDefenceProtector) ) + 0.5 * norm(x(:,defenceProtectors(2))-x(:,defenceProtectors(1))-0.2) * ( x(:,defenceProtectors(2)) - x(:,defenceProtectors(1)) ) + 0.5 * norm(x(:,defenceProtectors(3))-x(:,defenceProtectors(1))-0.2) * ( x(:,defenceProtectors(3)) - x(:,defenceProtectors(1)) );
          dx(:,defenceProtectors(2)) = dx(:,defenceProtectors(2))  + 0.5 * norm(x(:,defenceProtectors(1))-x(:,defenceProtectors(2))-0.2) * ( x(:,defenceProtectors(1)) - x(:,defenceProtectors(2)) ) + 0.5 * norm(x(:,defenceProtectors(3))-x(:,defenceProtectors(2))-0.2) * ( x(:,defenceProtectors(3)) - x(:,defenceProtectors(2)) );
          dx(:,defenceProtectors(3)) = dx(:,defenceProtectors(3))  + 0.5 * norm(x(:,defenceProtectors(1))-x(:,defenceProtectors(3))-0.2) * ( x(:,defenceProtectors(1)) - x(:,defenceProtectors(3)) ) + 0.5 * norm(x(:,defenceProtectors(2))-x(:,defenceProtectors(3))-0.2) * ( x(:,defenceProtectors(2)) - x(:,defenceProtectors(3)) );
          
          
%           dx(:,defenceProtectors(1)) = dx(:,defenceProtectors(1)) + ( barrierGoal - x(:,defenceProtectors(1)) ); 
           
          
    end
    du(:,1:NP) = si_barrier_cert(dx(:,1:NP), xuni(:,1:NP));
%     du = dx;
    dxu(:,1:NP) = si_to_uni_dyn(du(:,1:NP), xuni(:,1:NP));                            % Convert single integrator inputs into unicycle inputs
    dxu = scale_velocities(dxu);
    rbtm.set_velocities(1:NP, dxu(:,1:NP)); 
    rbtm.step();              % Set new velocities to robots and update
    
    %% Decoyers
    
    xuni = rbtm.get_poses();                                % Get new robots' states
    x = xuni(1:2,:);                                        % Extract single integrator states
    dx = zeros(2,N);
    switch (statesDecoyers)
 
        case 0
            disp('case 0 working')
            checkLeader1 = 0;
            checkLeader2 = 0;
            for i = NP+1 :N
                for j = topological_neighbors(L,i)
                    dx(:,i) = dx(:,i) + (norm(x(:,j) - x(:,i))-0.2)*(x(:,j) - x(:,i));
                end
                if (i==NP+2 && length(topological_neighbors(L,i)) >= 3)
                    checkLeader1 = 1;
                end
            end
            du = si_barrier_cert(dx, xuni);
            dxu = si_to_uni_dyn(du, xuni);                            % Convert single integrator inputs into unicycle inputs
            
            if (checkLeader1 == 1)
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
            du(:,NP+2) = si_barrier_cert(dx(:,NP+2), xuni(:,NP+2));
            dxu(:,NP+2) = scale_velocities(dx(:,NP+2));
            dxu(:,NP+2) = dxu(:,NP+2)./3;
            
            if e_Norm < 0.2
                statesDecoyers = 2;
            end
            
        case 2
            if norm([0.6; 0] - x(:, NP+2)) > 0.02
                dx(:, NP+2) = [0.6; 0] - x(:, NP+2);
                opp = find_opponents(L, NP+2, NP);
                for j = opp
                    err = x(:, j) - x(:, NP+2);
                    K = 000.02/(norm(err)*(norm(err)^2 + 0.0000001));
%                     K = 000.02/(norm(err)*(norm(err)^2 + 0.00001));
%                      Interestingly, the robot is able to capture the flag
%                      for this K, although there are more chances for
%                      collisions
                    dx(:, NP+2) = dx(:, NP+2) + K*(-err);
                end
            else
                statesDecoyers = 2;
            end
%             du = si_barrier_cert(dx, xuni);
            dxu = si_to_uni_dyn(dx, xuni);                            % Convert single integrator inputs into unicycle inputs
            dxu(:,NP+2) = dxu(:,NP+2)./3;
            
    end
    
    dxu = scale_velocities(dxu);
    rbtm.set_velocities(NP+1:N, dxu(:,NP+1:N)); 
    rbtm.step();              % Set new velocities to robots and update

    disk_poses = repelem(x,1, length(delta_angles));
    disk_poses_x_p = disk_poses(1,1:length(delta_angles)*NP) + disk_angles_x(1:length(delta_angles)*NP);
    disk_poses_y_p = disk_poses(2,1:length(delta_angles)*NP) + disk_angles_y(1:length(delta_angles)*NP);
    disk_poses_x_d = disk_poses(1,(length(delta_angles)*NP)+1:end) + disk_angles_x((length(delta_angles)*NP)+1:end);
    disk_poses_y_d = disk_poses(2,(length(delta_angles)*NP)+1:end) + disk_angles_y((length(delta_angles)*NP)+1:end);
    refreshdata([delta_handle_p, delta_handle_d])%, wedge_handle_p,wedge_line_handle_left,wedge_line_handle_right]);

end


rbtm.debug();
   
