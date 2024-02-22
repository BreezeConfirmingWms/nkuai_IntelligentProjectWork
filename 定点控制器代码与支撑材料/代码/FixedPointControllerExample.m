%% 移动机器人控制器设计与仿真
function FixedPointControllerExample()


    function [v,w] = FixedPointNonLinearController(t,pose,Endpose,kParam)
        dPose = Endpose -pose;
        %disp(pose(3));
        e = [cos(pose(3)) sin(pose(3)) 0; -sin(pose(3)) cos(pose(3)) 0;0 0 1]*dPose;
        
        v = -kParam(1)* e(1);
        w = -kParam(2)*e(3) + e(2)^2*sin(t);
    end

    function [v,w] = PolarCoordController(pose,Endpose,kParam)
        dPose = Endpose -pose;
        rho =  sqrt(dPose(1)^2+dPose(2)^2);
        beta = - atan2(dPose(2),dPose(1));
        alpha = - beta +dPose(3);
        
        
         v= kParam(1)*rho;
         w = kParam(2)*alpha+kParam(3)*beta;
    end




path = [2.00    1.00;
         4.50    6.00;
        -2.00    8.00;
        -3.00    -3.00;
        8.00   2.00];
%path=[2 1;5 10];


robotInitialLocation = path(1,:);
robotGoal = path(end,:);



initialOrientation = 0;

robotCurrentPose = [robotInitialLocation initialOrientation]';

robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");


controller = controllerPurePursuit;%跟踪控制器
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.6;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.3;

goalRadius = 0.2;
distanceToGoal = norm(robotInitialLocation - robotGoal);
distanceCurrent = norm(robotInitialLocation - path(2,:));

% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

% Initialize the figure
figure(1)

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/0.8;


epoch=1;
indices=1;
cnt=2;
Endpose = [path(cnt,:) 0];
MinDist=1e8;

Xpos=zeros(1e4,1);
Ypos=zeros(1e4,1);
frames=cell(1,200);

fnum=0;
while( distanceToGoal > goalRadius && epoch<1e4)
    
    if mod(epoch,indices)==0
        Xpos(epoch/indices)=robotCurrentPose(1,:);
        Ypos(epoch/indices)=robotCurrentPose(2,:);
    end
    MinDist=min(MinDist,distanceCurrent);

    if distanceCurrent<= goalRadius
        cnt=cnt+1;
        MinDist=1e8;
        Endpose=[path(cnt,:) 0];
    end
    
    % Compute the controller outputs, i.e., the inputs to the robot
    % 这里可以选择三种控制器，从上往下分别是路径跟踪控制器、非线性定点控制器和极坐标控制器
    [v, omega] = controller(robotCurrentPose);
    %[v,omega] = FixedPointNonLinearController(epoch*sampleTime,robotCurrentPose,Endpose',[-0.6 -0.3]);
    %[v,omega] = PolarCoordController(robotCurrentPose,Endpose',[3,8,-1.5]);
    
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 

    % Re-compute the distance to the goal
    distanceCurrent = norm(robotCurrentPose(1:2)- path(cnt,:)');
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    if mod(epoch,50)==0
        disp([distanceCurrent distanceToGoal])
    end
    
    % Update the plot
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % 移动机器人仿真路径实例化
    plot(path(:,1), path(:,2),"k--d");
    hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlabel('X Position');
    ylabel('Y Position');
    xlim([-5 13])
    ylim([-5 13])
    if mod(epoch,5*indices)==0
        fnum=fnum+1;
        frame = getframe(gcf);
        frames{fnum}=frame;
    end
    waitfor(vizRate);
    epoch=epoch+1;
end


filename = 'controller3.gif';

for i =1:fnum
    im  = frames{i}.cdata;
    [imcnd,cm] = rgb2ind(im,256);
    if i==1 
        imwrite(imcnd,cm,filename,'gif','Loopcount',inf);
    else
        imwrite(imcnd,cm,filename,'gif','WriteMode','append');
    end
end


% 路径的可视化与数据预处理

figure(2)
plot(Xpos(1:(epoch-1)/indices,1),Ypos(1:(epoch-1)/indices,1),'r-');
hold all
plot(path(:,1), path(:,2),"g--o");
xlim([-5 15])
ylim([-5 15])
title('trajectory of Fork Lift robot')


Tspan = sampleTime:sampleTime:((epoch-1)/indices)*sampleTime;

[n,~]=size(path);
indexPosX = zeros(n-1,length(Tspan));
indexPosY = zeros(n-1,length(Tspan));


for i=2:n
    indexPosX(i-1,:)= ones(length(Tspan),1).*path(i,1);
end
for i=2:n
    indexPosY(i-1,:)= ones(length(Tspan),1).*path(i,2);
end


figure(3)
plot(Tspan,Xpos(1:(epoch-1)/indices,1),'r-','LineWidth',3,'DisplayName','x平衡轨迹');
hold all
for i=1:n-1
  
    plot(Tspan,indexPosX(i,:),'k--','HandleVisibility','off');
    
end
xlabel('steps');
ylabel('X Pos');
legend();
title('X Pos Controlled Curve');
figure(4)
plot(Tspan,Ypos(1:(epoch-1)/indices,1),'b-','LineWidth',3,'DisplayName','y平衡轨迹');
hold all
for i=1:n-1
    plot(Tspan(:),indexPosY(i,:),'k--','HandleVisibility','off');
end
legend()
xlabel('steps');
ylabel('Y Pos');
legend();
title('Y Pos Controlled Curve');
end



