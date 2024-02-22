
pgm_image = imread('house_map.pgm');

% 转换为二值图像
binary_image = imbinarize(pgm_image);
binary_image = ~binary_image;
% 转换为地图
%map = double(binary_image);
map = binaryOccupancyMap(binary_image,13);
% 保存地图到MAT文件
save('HouseMap.mat', 'map');
refFigure = figure('Name','SimpleMap');
show(map);
