% Generate a path for drone in 3D environment based on RRT algorithm
clear;
close all;
clc
 
%% Set Parameters
% Step size of RRT algorithm
step=5;
% Vertex points of 3D environment map
axisStart = [0 0 0];
axisLWH = [500 100 100];
 
% Coordinates of obstacles.The simplest case (regular hexahedron): [Xmin,Xmax,Ymin,Ymax,Zmin,Zmax]
ObsCoor = [50 150 20 30 0 50;
           70 110 50 60 0 60;
           160 210 60 90 0 70;
           200 260 30 50 0 80;
           300 370 35 67 0 60;
           390 470 20 50 0 60];
% The starting and ending points of the path (to improve planning efficiency, you can choose to add a middle path point)
pathPoint = [10 10 10;
            460 70 60];  % A series of path points
 
%% Fig
figure(1)
colorMatCube = [1 0 0];
pellucidity = 0.6;
% 3D environment map
drawEnvironmentMap(ObsCoor)
hold on;
scatter3(pathPoint(1,1),pathPoint(1,2),pathPoint(1,3),'MarkerEdgeColor','k','MarkerFaceColor',[1 0 0]);
scatter3(pathPoint(end,1),pathPoint(end,2),pathPoint(end,3),'MarkerEdgeColor','k','MarkerFaceColor','b');
text(pathPoint(1,1),pathPoint(1,2),pathPoint(1,3),'Start Point');
text(pathPoint(end,1),pathPoint(end,2),pathPoint(end,3),'End Point');
view(3)
grid on;
axis equal;
axis([0 axisLWH(1,1) 0 axisLWH(1,2) 0 axisLWH(1,3)])
xlabel('x')
ylabel('y')
zlabel('z')
 
 
%% Generate paths using RRT algorithm
totalPath = [];
for k1 = 1:size(pathPoint,1)-1
    startPoint = pathPoint(k1,:);
    goalPoint = pathPoint(k1+1,:);
    Path = RRT(startPoint,axisStart,axisLWH,goalPoint,ObsCoor,step);
    
    if ~isempty(Path)
        for k2 = 1:size(Path,1)-1
            line([Path(k2,1) Path(k2+1,1)],[Path(k2,2) Path(k2+1,2)],[Path(k2,3) Path(k2+1,3)],'LineWidth',1,'Color','red');
        end
        totalPath = [totalPath;Path];
    end
end