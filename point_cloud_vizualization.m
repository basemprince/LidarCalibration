ptCloud = pcread('point_cloud/01.pcd');
ptCloud = pcmedian(ptCloud);
ptCloud = removeInvalidPoints(ptCloud);
ptCloud = pcdenoise(ptCloud, 'Threshold' , 0.01,'NumNeighbors',1);
ptCloud = pcdownsample(ptCloud,'gridAverage',0.005);
pcshow(ptCloud)
title('Input Point Cloud')
% xlim([-0 4]);
% ylim([-2 5]);
% zlim([-1 5]);

% xlim([0 2]);
% ylim([-1 1]);
% zlim([-0.1 0.5]);

minDistance = 0.1;



squareSize = 67; % Square size of the checkerboard
boardSize = [402 470];

%roi2 = [0, 2, -1, 1, -0.1, 0.5];

[labels, numClusters] = pcsegdist(ptCloud , minDistance);



[lidarCheckerboardPlane, ptCloudUsed] = detectRectangularPlanePoints_mod(ptCloud,boardSize,'MinDistance',0.1,'DimensionTolerance' ,0.2);
%lidar.internal.calibration.extractRectangle

planeDimension = boardSize/1000;
tolerance = 0.05;
minWidthToCheck = planeDimension(1) - planeDimension(1) * tolerance;
maxWidthToCheck = planeDimension(1) + planeDimension(1) * tolerance;

% Limits for longer dimension
minLengthToCheck = planeDimension(2) - planeDimension(2) * tolerance;
maxLengthToCheck = planeDimension(2) + planeDimension(2) * tolerance;

%dimensionRange = [minWidthToCheck, maxWidthToCheck, minLengthToCheck, maxLengthToCheck];
%[rectangle3d, isptCloudUsed, indices] = lidar.internal.calibration.extractRectangle(ptCloud,roi2, 0.001, dimensionRange, true);
hRect = figure;
panel = uipanel('Parent',hRect,'BackgroundColor',[0 0 0]);
ax = axes('Parent',panel,'Color',[0 0 0]); 
pcshow(lidarCheckerboardPlane)
title('Rectangular Plane Points')
% %ptCloud2 = pointCloud(brushedData)
% figure
pcshowpair(ptCloud,lidarCheckerboardPlane)
title('Detected Rectangular Plane')
% xlim([-5 10])
% ylim([-5 10])