%%  高斯模型里程计误差标定
function CarlibrationEpsCalc()


global radius;%半径
global d;%轮心距

radius=0.1;
d=0.15;
    function convPropagation=Propagation(simulation_type,cov_pose,cov_delta,theta_k,delta_theta,delta_s)
       % 定义误差传导的数理模型函数
        if simulation_type =="Gaussian"
            Func_p = [1 0 -delta_s*sin(theta_k + delta_theta/2) ;0 1 delta_s*cos(theta_k + delta_theta/2) ; 0 0 1];
            Func_delta = [radius/2*cos(theta_k+delta_theta/2) - radius/(4*d)*delta_s*sin(theta_k + delta_theta/2),radius /2 *cos(theta_k+delta_theta/2) + radius/(4*d) * delta_s *sin(theta_k+delta_theta/2);
                radius/2*sin(theta_k + delta_theta/2) + radius/(4*d)*delta_s *cos(theta_k+delta_theta/2), radius/2*sin(theta_k + delta_theta/2) - radius/(4*d)*delta_s*cos(theta_k + delta_theta/2);
                radius/(2*d),-radius/(2*d)];
            
            convPropagation = Func_p * cov_pose*Func_p'+Func_delta*cov_delta*Func_delta';
            
        else
            convPropagation=-1;
        end
    end


robot = differentialDriveKinematics('WheelRadius',0.1,"TrackWidth", 0.3,"VehicleInputs", "WheelSpeeds"); %构建差速机器人模型

v=0.15;
omega=0;

VL=[1.5 1];
VR=[1.5 1];

wheel_L=VL(1,2);
wheel_R=VR(1,2);

 k_l=2*wheel_L/(wheel_L+wheel_R);
 k_r=2*wheel_R/(wheel_L+wheel_R);

sampleTime = 0.1;

Simulation_Time =0:sampleTime:10;


vizRate = rateControl(1/sampleTime);
frameSize = robot.TrackWidth/0.8;

CurrentPose = [0;0;0];
epoch=0;
Propagations=zeros(3,3,length(Simulation_Time));


cov_pose = [0 0 0; 0 0 0;0 0 0];

 cnt=1;
 sampleIdx=20;
 


 xzero = zeros(100,1);
 yzero =zeros(100,1);
 thetas=0;
 
 
TrackX=zeros(length(Simulation_Time),1);
TrackY = zeros(length(Simulation_Time),1);
Thetas = zeros(length(Simulation_Time),1);
SigmaIdx =1;
while epoch<length(Simulation_Time)
    %循环更新 Sigma_p
    
    
    vel =derivative(robot,CurrentPose,[wheel_L,wheel_R]);
    CurrentPose = CurrentPose + vel*sampleTime;
    if mod(epoch,sampleIdx)==0 && epoch>0
   
    vTmp = radius/2*(wheel_L+wheel_R);
    omegaTmp = radius/(2*d)*(wheel_R-wheel_L);
    
    delta_s = vTmp*sampleTime*sampleIdx;
    delta_theta = omegaTmp*sampleTime*sampleIdx;
     

    
    delta_R= abs(wheel_R*sampleTime*sampleIdx);
    delta_L = abs(wheel_L*sampleTime*sampleIdx);
    
    cov_delta = [delta_L*SigmaIdx/100 0 ; 0 delta_R*SigmaIdx/100];
    Propagations(:,:,cnt)=Propagation('Gaussian',cov_pose,cov_delta,thetas,delta_theta,delta_s);
    xzero(cnt,1)=CurrentPose(1,:);
    yzero(cnt,1)= CurrentPose(2,:);
    Thetas(cnt,1) = thetas;

    thetas = thetas+delta_theta;
    
    cov_pose = Propagations(:,:,cnt);
    cnt = cnt+1;
    end
    TrackX(epoch+1,:)=CurrentPose(1,:);
    TrackY(epoch+1,:)=CurrentPose(2,:);
    epoch=epoch+1;
    
end


Ra=zeros(cnt-1,1);
Rb=zeros(cnt-1,1);

for i=1:cnt-1



CovPose = Propagations(:,:,i);
a = sqrt(CovPose(1,1));
b=  sqrt(CovPose(2,2));


Ra(i,1)=a;
Rb(i,1)=b;

end
h=ellipse(Ra, Rb, Thetas(1:cnt-1,:), xzero(1:cnt-1,:), yzero(1:cnt-1,:));
fig = gcf ;
ax =gca;
plot(ax,TrackX,TrackY);
xlabel('X/m');
ylabel('Y/m');
title('Odometry Propagation Law to  Ellipse Error Curve '); %完善图例


% 设置图柄的显示范围
% ax.XLim=[0 2];
% ax.YLim=[-0.6 0.6];
% ax.XTick=0:0.5:2;
% ax.YTick=-0.6:0.2:0.6;

% ax.XLim=[0 2];
% ax.YLim=[-0.2 1.5];
% ax.XTick=0:0.5:2;
% ax.YTick=-0.2:0.1:1.5;

ax.XGrid = 'on';
ax.YGrid = 'on';

disp('debug');

end