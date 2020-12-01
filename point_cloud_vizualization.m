ptCloud = pcread('point_cloud_test/01.pcd');
ptCloud = pcmedian(ptCloud);
ptCloud = pcdownsample(ptCloud,'gridAverage',0.05);
pcshow(ptCloud)
title('Input Point Cloud')
xlim([-0 4]);
ylim([-2 5]);
zlim([-1 5]);

squareSize = 67; % Square size of the checkerboard
boardSize = [402 469];

roi2 = [0, 2, -1, 1, -0.1, 0.5];

[lidarCheckerboardPlane, ptCloudUsed] = detectRectangularPlanePoints(ptCloud,boardSize,'DimensionTolerance',0.5);%,,'MinDistance',0.001);
hRect = figure;
panel = uipanel('Parent',hRect,'BackgroundColor',[0 0 0]);
ax = axes('Parent',panel,'Color',[0 0 0]); 
pcshow(lidarCheckerboardPlane)
title('Rectangular Plane Points')
ptCloud2 = pointCloud(brushedData)
figure
pcshowpair(ptCloud,lidarCheckerboardPlane)
title('Detected Rectangular Plane')
xlim([-5 10])
ylim([-5 10])