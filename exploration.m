ptCloud = pcread('point_cloud_test/02.pcd');
ptCloud = pcmedian(ptCloud);
ptCloud = removeInvalidPoints(ptCloud);
ptCloud = pcdenoise(ptCloud, 'Threshold' , 0.01,'NumNeighbors',1);
location = ptCloud.Location;
roi2 = [0, 2, -1, 1, -0.1, 0.5];

%points = findPointsInROI(ptCloud,roi2);
%ptCloud = pointCloud(points);
pcshow(ptCloud)
title('Input Point Cloud')
%ptCloud = pcdownsample(ptCloud,'gridAverage',0.005);

% xlim([-0 4]);
% ylim([-2 5]);
% zlim([-1 5]);

% xlim([0 2]);
% ylim([-1 1]);
% zlim([-0.1 0.5]);
minDistance = 0.1;



indices = [];


squareSize = 67; % Square size of the checkerboard
boardSize = [402 470];



planeDimension = boardSize/1000;
tolerance = 0.1;
minWidthToCheck = planeDimension(1) - planeDimension(1) * tolerance;
maxWidthToCheck = planeDimension(1) + planeDimension(1) * tolerance;

% Limits for longer dimension
minLengthToCheck = planeDimension(2) - planeDimension(2) * tolerance;
maxLengthToCheck = planeDimension(2) + planeDimension(2) * tolerance;




[labels, numClusters] = pcsegdist(ptCloud , minDistance);
for i = 1:numClusters
    tmpPcLoc = ptCloud.Location(labels == i, :);
    if size(tmpPcLoc,1) > 25
        [~, inliers] = pcfitplane(pointCloud(tmpPcLoc),0.1);
        if ~isempty(inliers)
            
            if size(inliers,1) > 25  
               
                segmentedPlane = select(pointCloud(tmpPcLoc), inliers);
                rectModel = ...
                    lidar.internal.calibration.fitRectangle3D(segmentedPlane, 'O', 'YPR', 'Iterations', 30);
                dimensions = rectModel.Dimensions;
                dimensions = sort(dimensions,"descend");
%                 disp(['cluster ', num2str(i) , ' is ', num2str(dimensions*1000)]);
                length = dimensions(1);
                width = dimensions(2);
%                 hRect = figure;
%                 panel = uipanel('Parent',hRect,'BackgroundColor',[0 0 0]);
%                 ax = axes('Parent',panel,'Color',[0 0 0]); 
%                 pcshowpair(ptCloud,segmentedPlane)
%                 title(i);
                
                if length < maxLengthToCheck && length > minLengthToCheck && ...
                        width > minWidthToCheck && width < maxWidthToCheck
                    plane = segmentedPlane;
                    ptCloudUsed = true;
                    indices = ismember(location,plane.Location, 'rows');
                    return;
                end
            end
        end
    end
end




