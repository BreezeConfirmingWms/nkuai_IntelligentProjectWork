%% AMCL定位仿真
load HouseMap.mat
%show(map);

odometryModel = odometryMotionModel;%引入里程计运动模型
odometryModel.Noise = [0.2 0.2 0.2 0.2];%高斯噪声设置，包括线和角的位移和速度

rangeFinderModel = likelihoodFieldSensorModel;%定义传感器模型的作用范围和精度
rangeFinderModel.SensorLimits = [0.45 8];
rangeFinderModel.Map = map;


%查询ROS中的变换矩阵关系树
tftree = rostf;
waitForTransform(tftree,'/base_footprint','/right_wheel');%基座标系到轮式里程计的变换
sensorTransform = getTransform(tftree,'/base_footprint', '/right_wheel'); 

% Get the euler rotation angles.
laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X ...
    sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');%四元数到欧拉角

% Setup the |SensorPose|, which includes the translation along base_link's
% +X, +Y direction in meters and rotation angle along base_link's +Z axis
% in radians.
rangeFinderModel.SensorPose = ...
    [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];

laserSub = rossubscriber('/my_robot/laser/scan');%订阅激光雷达的粒子数据
odomSub = rossubscriber('/my_robot/odom');%订阅里程计话题
gzPoseSub = rossubscriber('/gazebo/model_states');%订阅真实坐标

[velPub,velMsg] = ...
    rospublisher('/my_robot/cmd_vel','geometry_msgs/Twist');%创建键盘话题发布器


amcl = monteCarloLocalization;
amcl.UseLidarScan = true;

amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;

amcl.UpdateThresholds = [0.2,0.2,0.2];%自适应蒙特卡洛方法的参数阈值
amcl.ResamplingInterval = 1;

mcl.ParticleLimits = [500 5000];
amcl.GlobalLocalization = false;%不启用全局定位辅助，为自适应方法
amcl.InitialPose = [15.3 15.3 0.00];% 特别注意要调整初始化位姿，...
%这里角度以zyx的x轴为参考
amcl.InitialCovariance = eye(3)*0.1;%初始化误差模型，方差




visualizationHelper = ExampleHelperAMCLVisualization(map);
wanderHelper = ...
    ExampleHelperAMCLWanderer(laserSub, sensorTransform, velPub, velMsg);
    %创建蒙特卡洛逐步试探的坐标换算仿真对象
numUpdates = 60;
frames = cell(1, numUpdates);
i = 0;
TruePoseX = zeros(1,numUpdates);
EstimPoseX  =zeros(1,numUpdates);
TruePoseY = zeros(1,numUpdates);
EstimPoseY  =zeros(1,numUpdates);
while i < numUpdates
    % 接收激光扫描和里程计信息。
    scanMsg = receive(laserSub);
    odompose = odomSub.LatestMessage;
    
    % 创建雷达扫描的数据对象以传递给AMCL对象。
    scan = lidarScan(scanMsg);
    gzpose = receive(gzPoseSub);
   
    % For sensors that are mounted upside down, you need to reverse the
    % order of scan angle readings using 'flip' function.
    
    % Compute robot's pose [x,y,yaw] from odometry message.
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];
    
    % Update estimated robot's pose and covariance using new odometry and
    % sensor readings.
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scan);
    
    % Drive robot to next pose.
    %wander(wanderHelper);
    
    % Plot the robot's estimated pose, particles and laser scans on the map.
    if isUpdated
        i = i + 1;
        EstimPoseX(1,i) = estimatedPose(1);
        EstimPoseY(1,i)=estimatedPose(2);

        TruePoseX(1,i) = gzpose.Pose(3,1).Position.X+amcl.InitialPose(1);
        TruePoseY(1,i) = gzpose.Pose(3,1).Position.Y+amcl.InitialPose(2);
        plotStep(visualizationHelper, amcl, estimatedPose, scan.Ranges,scan.Angles, i);
        frame=getframe(gcf);
        frames{i} = frame;
    end
end


filename = 'animation3.gif';
for i = 1:numUpdates
     im = frames{i}.cdata;
    [imind, cm] = rgb2ind(im, 256);
    if i == 1
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append');
    end
end


sz=30;
figure;

scatter(TruePoseX,TruePoseY,sz,'bo','DisplayName','真实位姿');
hold all;
scatter(EstimPoseX,EstimPoseY,sz,'go','filled','DisplayName','估计位姿');
plot(TruePoseX,TruePoseY,'red','HandleVisibility','off');
plot(EstimPoseX,EstimPoseY,'r--','HandleVisibility','off');
xlabel('X坐标');
ylabel('Y坐标');
%legend();
title('自适应蒙特卡洛定位结果');



