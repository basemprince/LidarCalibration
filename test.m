imagePath = fullfile('/Users','basemprince','Downloads','LidarCalibration', 'images');
ptCloudPath = fullfile('/Users','basemprince','Downloads','LidarCalibration', 'point_cloud');
%cameraParamsPath = fullfile(imagePath, 'camera_params.mat');
load('cameraParams1.mat');
%intrinsic = load(cameraParamsPath); % Load camera intrinsics
imds = imageDatastore(imagePath); % Load images using imageDatastore
pcds = fileDatastore(ptCloudPath, 'ReadFcn', @pcread); % Load point cloud files

imageFileNames = imds.Files;
ptCloudFileNames = pcds.Files;

squareSize = 67; % Square size of the checkerboard
 
% Set random seed to generate reproducible results.
rng('default');

[imageCorners3d, checkerboardDimension, dataUsed] = ...
    estimateCheckerboardCorners3d(imageFileNames, cameraParams, squareSize);
imageFileNames = imageFileNames(dataUsed); % Remove image files that are not used
%helperShowImageCorners(imageCorners3d, imageFileNames, cameraParams);

% Extract ROI from the detected image corners
roi = helperComputeROI(imageCorners3d, 2);

% Filter point cloud files corresponding to the detected images
ptCloudFileNames = ptCloudFileNames(dataUsed);

ptCloud = pcread(ptCloudFileNames{1});
for i=2:numel(ptCloudFileNames)
    ptCloud(i) = pcread(ptCloudFileNames{i});
end
for i = 1:length(ptCloud)
ptCloud(i) = pcmedian(ptCloud(i));
% ptCloud(i) = pcdenoise(ptCloud(i), 'Threshold' , 0.0001,'NumNeighbors',1);
% ptCloud(i) = pcdownsample(ptCloud(i),'gridAverage',0.001);
% ptCloud(i) = pcdenoise(ptCloud(i));%, 'Threshold' , 0.01,'NumNeighbors',1);
end

[lidarCheckerboardPlanes, framesUsed, indices] = detectRectangularPlanePoints_mod(ptCloud, checkerboardDimension, 'ROI', roi,'MinDistance',0.1,'DimensionTolerance' ,0.1);

% Remove ptCloud files that are not used
ptCloudFileNames = ptCloudFileNames(framesUsed);
% Remove image files 
imageFileNames = imageFileNames(framesUsed);
% Remove 3D corners from images
imageCorners3d = imageCorners3d(:, :, framesUsed);

helperShowLidarCorners(ptCloudFileNames, indices);

[tform, errors] = estimateLidarCameraTransform(lidarCheckerboardPlanes, ...
    imageCorners3d, 'CameraIntrinsic', cameraParams.Intrinsics);

helperFuseLidarCamera(imageFileNames, ptCloudFileNames, indices, ...
    cameraParams.Intrinsics, tform);