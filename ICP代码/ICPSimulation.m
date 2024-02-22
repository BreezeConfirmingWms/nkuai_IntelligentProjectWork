%% ICP点云配准:旋转矩阵还是平移矩阵影响大
fbin=fopen('./pcData/oxford1.bin','r');
PointCloud1= fread(fbin,[3,inf],'float32');
ptCloud1 = pointCloud(PointCloud1');
fbin = fopen('./pcData/oxford2.bin','r');
PointCloud2=fread(fbin,[3,inf],'float32');
ptCloud2 = pointCloud(PointCloud2');


figure;
subplot(1,2,1); pcshow(ptCloud1); title('点云1');
subplot(1,2,2); pcshow(ptCloud2); title('点云2');

args.R=[eye(3); 1.5*eye(3);2*eye(3)];
args.t=[zeros(1,3);0.5*ones(3);1.5*ones(3)];
% 初始化旋转和平移矩阵
R = eye(3);
t=0.5*ones(1,3);
figure;
hAxes = pcshowpair(ptCloud1, ptCloud2);
title('Initial Alignment');

for k=1:3:size(args.R,1)
% 迭代次数
numIterations = 5;

% 显示初始状态



ptRtCloud1=ptCloud1;
ptRtCloud2=ptCloud2;


ptTrCloud1 = ptCloud1;
ptTrCloud2 = ptCloud2;

R=args.R(k:k+2,:);

maxRMSERt=inf;
% 迭代ICP算法
    for i = 1:numIterations
    % 对第二个点云进行旋转和平移
    ptCloud2Aligned = pctransform(ptRtCloud2, affine3d(cat(2,[R;t],[0;0;0;1])));
    
    % 进行ICP配准
    [tform, ptCloud2Aligned, rmse] = pcregistericp(ptRtCloud1, ptCloud2Aligned);
    
   
    if maxRMSERt>rmse
        maxRMSERt=rmse;
    end
    if rmse<1.3
        break;
    end
    
    % 更新旋转和平移矩阵
    R= tform.T(1:3,1:3);
    t = tform.T(4,1:3);
    
   
    
    % 显示配准后的点云及变换矩阵
%     disp(['Iteration ',num2str(i),', RMSE = ',num2str(rmse)]);
%     disp(['Rotation matrix:']);
%     disp(R);
%     disp(['Translation vector:']);
%     disp(t);
    hAxes = pcshowpair(ptRtCloud1, ptCloud2Aligned);
    title(['Rotate Iteration ',num2str(i)]);
    drawnow;
    
    
    end
end

R=eye(3);


for k=1:size(args.t,1)
% 迭代次数
numIterations =5;

% 显示初始状态



t=args.t(k,:);

maxRMSETr=inf;
% 迭代ICP算法
    for i = 1:numIterations
    % 对第二个点云进行旋转和平移
    ptCloud2Aligned = pctransform(ptTrCloud2, affine3d(cat(2,[R;t],[0;0;0;1])));
    
    % 进行ICP配准
    [tform, ptCloud2Aligned, rmse] = pcregistericp(ptTrCloud1, ptCloud2Aligned);
    
   
    if maxRMSETr>rmse
        maxRMSETr=rmse;
    end
    if rmse<1.3
        break;
    end
    
    % 更新旋转和平移矩阵
    R= tform.T(1:3,1:3);
    t = tform.T(4,1:3);
    
   
    
    % 显示配准后的点云及变换矩阵
%     disp(['Iteration ',num2str(i),', RMSE = ',num2str(rmse)]);
%     disp(['Rotation matrix:']);
%     disp(R);
%     disp(['Translation vector:']);
%     disp(t);
    hAxes = pcshowpair(ptTrCloud1, ptCloud2Aligned);
    title(['Translation Iteration ',num2str(i)]);
    drawnow;
    
    
    end
end

fprintf('the translation get minimum rmse %d \n',maxRMSETr);
fprintf('the rotate get minimum rmse %d \n',maxRMSERt);
if maxRMSETr>maxRMSERt
    fprintf("The Rotate Matrix have an postitive effect on ICP then Translation \n");
else
     fprintf("The Translation Matrix have an postitive effect on ICP then Rotate \n");
end



