imagePath = fullfile('/home', 'driverless', 'Documents', 'MATLAB', 'Examples', 'R2020b', 'lidar', 'LidarCameraCalibrationExample', 'images');
ptCloudPath = fullfile('/home', 'driverless', 'Documents', 'MATLAB', 'Examples', 'R2020b', 'lidar', 'LidarCameraCalibrationExample', 'point_cloud');
%cameraParamsPath = fullfile(imagePath, 'camera_params.mat');

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

% helperShowImageCorners(imageCorners3d, imageFileNames, cameraParams);

% Extract ROI from the detected image corners
roi = helperComputeROI(imageCorners3d, 2);
 
% Filter point cloud files corresponding to the detected images
ptCloudFileNames = ptCloudFileNames(dataUsed);
[lidarCheckerboardPlanes, framesUsed, indices] = ...
    detectRectangularPlanePoints(ptCloudFileNames, checkerboardDimension, 'ROI', roi);

% Remove ptCloud files that are not used
ptCloudFileNames = ptCloudFileNames(framesUsed);
% Remove image files 
imageFileNames = imageFileNames(framesUsed);
% Remove 3D corners from images
imageCorners3d = imageCorners3d(:, :, framesUsed);

helperShowLidarCorners(ptCloudFileNames, indices);
