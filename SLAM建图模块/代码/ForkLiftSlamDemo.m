%%  ROS Laser Scan ==> Matlab Lidar-SLAM建图

maxLidarRange = 8;
mapResolution = 40;
slamAlg = lidarSLAM(mapResolution, maxLidarRange);%创建雷达工具箱的slam对象

slamAlg.LoopClosureThreshold = 210;  %设置基础循环的阈值
slamAlg.LoopClosureSearchRadius = 8;

laserSub = rossubscriber('/my_robot/laser/scan');%订阅雷达扫描话题
isRunning=true;
numofSample = 0;
numIter =60;
cnt=0;
scans=cell(1,numIter);
while numofSample<numIter

    numofSample  = numofSample+1;
    ScanMsg = receive(laserSub);%雷达数据需要通过下面处理得到Scan对象
    ranges = double(ScanMsg.Ranges);
    angles = linspace(ScanMsg.AngleMin, ScanMsg.AngleMax, numel(ranges));
    
    % 创建lidarScan对象
    scans{numofSample} = lidarScan(ranges, angles);

    pause(1);
  
end


save('MyScan.mat','scans');
disp('Scans have been stored!');
firstTimeLCDetected = false;

for i=1:numIter
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{i});
    %disp(i);
    if isScanAccepted
         fprintf('Added scan %d \n', i);
    end
    % visualize the first detected loop closure, if you want to see the
    % complete map building process, remove the if condition below
end


figure
show(slamAlg);
title({'Final Built Map of the Environment', 'Trajectory of the Robot'});


[scans, optimizedPoses]  = scansAndPoses(slamAlg);%获得算法转化的扫描位姿
map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);%创建地图对象


figure; 
show(map);%展示黑白的栅格地图
hold on
show(slamAlg.PoseGraph, 'IDs', 'off');
hold off
title('Occupancy Grid Map Built Using Lidar SLAM');