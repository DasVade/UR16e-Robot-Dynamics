function draw_cuboid(originPoint,cuboidSize,num)
%% 函数功能： 绘制长方体
% 输入：
%       originPoint：长方体的原点,行向量，如[0，0，0];
%       cuboidSize：长方体的长宽高,行向量，如[10，20，30];
%       num：长方体个数   
% 输出：长方体图形
for i = 1:num
%% 根据原点和尺寸，计算长方体的8个的顶点
vertexIndex=[0 0 0;0 0 1;0 1 0;0 1 1;1 0 0;1 0 1;1 1 0;1 1 1];
vertex=originPoint(i,:)+vertexIndex.*cuboidSize(i,:);

%% 定义6个平面分别对应的顶点
facet=[1 2 4 3;1 2 6 5;1 3 7 5;2 4 8 6;3 4 8 7;5 6 8 7];

%% 定义8个顶点的颜色，绘制的平面颜色根据顶点的颜色进行插补
color=[1;2;3;4;5;6;7;8];

%% 绘制并展示图像
% patch 对图像进行绘制。
% view(3) 将图像放到三维空间中展示。
% 其余的是设置背景等等
patch('Vertices',vertex,'Faces',facet,'FaceVertexCData',color,'FaceColor','interp','FaceAlpha',0.5);
view(3);
end
