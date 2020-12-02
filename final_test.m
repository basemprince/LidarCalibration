imagePath = fullfile('/Users','basemprince','Downloads','LidarCalibration', 'images_test');
ptCloudPath = fullfile('/Users','basemprince','Downloads','LidarCalibration', 'point_cloud_test');
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

helperShowImageCorners(imageCorners3d, imageFileNames, cameraParams);

% Extract ROI from the detected image corners
roi = helperComputeROI(imageCorners3d, 5);
 
% Filter point cloud files corresponding to the detected images
ptCloudFileNames = ptCloudFileNames(dataUsed);

ptCloudPlane = pointCloud(brushedData);
tform = estimateLidarCameraTransform(ptCloudPlane, imageCorners3d);
%tform = invert(tform);

imPts = projectLidarPointsOnImage(ptCloudPlane, cameraParams.Intrinsics, tform);

image = imread('images_test/01.png');
figure
imshow(image)
hold on
plot(imPts(:,1), imPts(:,2), '.', 'Color', 'r');
hold off
