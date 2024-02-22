%%  非高斯模型的里程计误差传导函数
function NotGaussianPropagation()

n = 10000; % 样本数量
num=20;
df = 10; % t分布自由度
err =zeros(n/num,num);
for i=1:n/num
err(i,:) = trnd(df, num, 1); % 生成t分布的误差样本
end

function [x, y, theta] = diff_drive_robot(x0, y0, theta0, v, omega, dt)
    theta = omega * dt;
    % 计算机器人的新位置和方向
    x = x0 + v * cos(theta0+theta/2) * dt;
    y = y0 + v * sin(theta0+theta/2) * dt;
    
end


x = zeros(n, 1);
y = zeros(n, 1);
theta = zeros(n, 1);
% 初始位置和方向
x(1) = 0;
y(1) = 0;
theta(1) = 0;
% 误差标准差
sigma_v = 0.12;
sigma_omega = 0.08;
cnt=2;
simTimer=linspace(4,8,n/num);
while cnt<n   
    % 添加误差
    r=mod(cnt,num);
    q=(cnt-r)/num;
    v = 1.5+ err(q+1,r+1) * sigma_v;
    omega = 0.12 + err(q+1,r+1) * sigma_omega;

  [x(cnt), y(cnt), theta(cnt)] = diff_drive_robot(x(1), y(1), theta(1), v,omega,simTimer(q+1));
    cnt=cnt+1;
end

% 绘制灰度热力图
min_x = min(x);
max_x = max(x);
min_y = min(y);
max_y = max(y);
bins = 50;
counts = hist3([x, y], [bins, bins]);
counts = counts / sum(counts(:));
x_edges = linspace(min_x, max_x, bins+1);
y_edges = linspace(min_y, max_y, bins+1);
figure;
imagesc(x_edges, y_edges, counts');
colormap(flipud(gray));
colorbar;
hold all;
fx = linspace(-14,8.5,1000);
fy = 1.5*ones(1,length(fx));
plot(fx,fy,'r-','LineWidth',3);
ax=gca;
ax.XLim=[0 15];
ax.YLim=[-14 14];
xlabel('X/m');
ylabel('Y/m');
title('Gray Heatmap of Positions Based On Not Gaussian Model');



end