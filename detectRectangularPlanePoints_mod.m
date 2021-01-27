function [ptCloudPlanes, ptCloudUsed, indicesCell] = detectRectangularPlanePoints(P, planeDimension, varargin)
%detectRectangularPlanePoints Detects rectangle plane of given dimension from a
%   point cloud.
%
%   The rectangular plane is detected by segmenting the input point cloud
%   into clusters using Euclidean distance and searching each cluster for
%   rectangular plane of given dimension.
%
%   ptCloudPlane = detectRectangularPlanePoints(ptCloudIn, planeDimension)
%   extracts rectangular plane, ptCloudPlane, from a point cloud, ptCloudIn.
%   The dimension of the plane to be detected is given as planeDimension, a
%   1-by-2 vector defined as [width, length] in mm. If the plane is not detected
%   then the output is empty.
%
%   [ptCloudPlanes, ptCloudUsed] = detectRectangularPlanePoints(ptClouds, planeDimension)
%   detects planes from a P-by-1 object array of pointCloud as ptClouds.
%   The output ptCloudPlanes is a P-by-1 array pointCloud objects having all the
%   detected planes. ptCloudUsed is a logical vector of the
%   same size as ptClouds. A value of true indicates that a plane was
%   detected in the corresponding point cloud. P is the number of point
%   clouds in which a plane is detected.
%
%   [..., indicesCell] = detectRectangularPlanePoints(ptCloudFileNames, planeDimension)
%   additionally returns a cell array of indices of the detected
%   checkerboards.
%
%   [...] = detectRectangularPlanePoints(..., Name, Value) specifies additional
%   name-value pair arguments described below.
%
%   'MinDistance'               Minimum Euclidean distance in meters between
%                               points from two different clusters, specified
%                               as a positive scalar.
%
%                               Default: 0.5
%
%   'ROI'                       The roi is a cuboid specified as a 1-by-6
%                               vector in the format of [xmin, xmax,
%                               ymin, ymax, zmin, zmax]. It can be used to
%                               reduce the search space for checkerboard
%                               detection.
%
%                               Default: []
%
%   'DimensionTolerance'        Value to define the minimum and maximum limits
%                               in width as
%                               [width - width*DimensionTolerance,
%                               width + width*DimensionTolerance] and
%                               length as
%                               [length - length*DimensionTolerance,
%                               length + length*DimensionTolerance]
%                               This value varies in the range [0 1].
%
%                               Default: 0.05
%
%   'RemoveGround'              Logical flag to remove ground plane using
%                               RANSAC before processing. It is assumed
%                               that the normal of the plane is pointing in
%                               +Z direction with the reference vector as
%                               [0,0,1].
%
%                               Default: false
%
%   'Verbose'                   Set true to display progress information.
%
%                               Default: false
%
%   Notes
%   -----
%   To get good results remove the ground plane from the point cloud using
%   RemoveGround flag. If the input point cloud is noisy then try
%   increasing the DimensionTolerance value. To reduce the processing time,
%   use ROI to define the estimated region for the checkerboard.
%   There should be more than 25 lidar points on the plane for detection to
%   work.
%
%   Class Support
%   -------------
%   ptCloudIn must be a pointCloud object. ptClouds must be a pointCloud
%   object array. ptCloudFileNames must be cell arrays of strings or
%   pointCloud objects. planeDimension can be a single or double.
%
%   Example: Detect checkerboard plane in point cloud files
%   ---------------------------------------------------------
%   % Load point cloud data
%   ptCloudsPath = fullfile(toolboxdir('lidar'), 'lidardata', 'lcc', ...
%   'vlp16', 'pointCloud','*.pcd');
%
%   ptCloudDir = dir(ptCloudsPath);
%   ptCloudFiles = strcat({ptCloudDir.folder}, '/', {ptCloudDir.name});
%
%   boardSize = [729 810]; % In mm
%   [lidarCheckerboardPlane, ptCloudUsed] = detectRectangularPlanePoints...
%   (ptCloudFiles{2}, boardSize, 'RemoveGround', true);
%
%   figure;
%   ptCloudIn = pcread(ptCloudFiles{2});
%   pcshowpair(ptCloudIn, lidarCheckerboardPlane);
%
%   See also detectCheckerboardPoints, pcsegdist, pcfitplane,
%   pcfitcuboid, estimateLidarCameraTransform

%   Copyright 2019-2020 The MathWorks, Inc.

narginchk(2, 12);

pointClouds = validateAndParsePointCloud(P);

checkPlaneDimension(planeDimension);

% Find default limits
ptCloudLimits = getLimits(pointClouds);

% Parse input arguements
[minDistance, roi, verbose, tolerance, removeGround] = ...
    parseInputs(ptCloudLimits, varargin{:});

planeDimension = planeDimension/1000;
minWidthToCheck = planeDimension(1) - planeDimension(1) * tolerance;
maxWidthToCheck = planeDimension(1) + planeDimension(1) * tolerance;

% Limits for longer dimension
minLengthToCheck = planeDimension(2) - planeDimension(2) * tolerance;
maxLengthToCheck = planeDimension(2) + planeDimension(2) * tolerance;

dimensionRange = [minWidthToCheck, maxWidthToCheck, minLengthToCheck, maxLengthToCheck];
[ptCloudPlanes, ptCloudUsed, indicesCell] = detectRectangularPlanePointsImpl(pointClouds,...
    roi, minDistance, dimensionRange, removeGround, verbose);

%--------------------------------------------------------------------------
function [minDistance, roi, verbose, tolerance, removeGround] = ....
    parseInputs(ptCloudLimits, varargin)
parser = inputParser;
parser.CaseSensitive = false;
parser.FunctionName = 'detectRectangularPlanePoints';

parser.addParameter('MinDistance', 0.5, @checkMindistance);
parser.addParameter('ROI', [], @checkROI);
parser.addParameter('Verbose', false, @checkVerbose);
parser.addParameter('DimensionTolerance', 0.05, @checkDimensionTolerance);
parser.addParameter('RemoveGround', false, @checkVerbose);

parser.parse(varargin{:});

minDistance = parser.Results.MinDistance;
roi = parser.Results.ROI;
verbose = logical(parser.Results.Verbose);
tolerance = parser.Results.DimensionTolerance;
removeGround = logical(parser.Results.RemoveGround);

% Check if the roi contains Inf value;
ind = ~isfinite(roi);
% Update Inf value with valid limits
roi(ind) = ptCloudLimits(ind);

%--------------------------------------------------------------------------
function tf = checkPlaneDimension(planeDimension)
validateattributes(planeDimension, {'single','double'}, ...
    {'nonnan', 'nonsparse', 'real', 'positive', 'finite', 'vector', 'numel', 2}, ...
    'planeDimension');
tf = true;

%--------------------------------------------------------------------------
function tf = checkVerbose(x)
validateattributes(x, {'logical', 'numeric'}, ...
    {'real', 'nonnegative', 'nonsparse', 'nonempty', 'scalar', 'nonnan', 'finite'}, ...
    mfilename, 'verbose');
tf = true;

%--------------------------------------------------------------------------
function tf = checkMindistance(minDistance)
validateattributes(minDistance, {'single','double'}, ...
    {'nonnan', 'nonsparse', 'scalar', 'real', 'positive', 'finite'}, ...
    'pcsegdist', 'minDistance');
tf = true;

%--------------------------------------------------------------------------
function tf = checkROI(roi)
if ~isempty(roi)
    if isnumeric(roi)
        validateattributes(roi, {'single', 'double'}, ...
            {'real', 'nonsparse', 'nonnan', 'ncols', 6, 'nrows', 1}, mfilename, 'roi');
        if any(roi(:, 1:2:5) > roi(:, 2:2:6))
            error(message('lidar:lidarCameraCalibration:invalidROI'));
        end
    else
        error(message('lidar:lidarCameraCalibration:invalidROIDataType'));
    end
end
tf = true;

%--------------------------------------------------------------------------
function tf = checkFileNames(fileNames)
validateattributes(fileNames, {'cell'}, {'nonempty', 'vector'}, mfilename, ...
    'ptCloudFileNames');
for i = 1:numel(fileNames)
    checkFileName(fileNames{i});
end
tf = true;

%--------------------------------------------------------------------------
function tf = checkFileName(fileName)
validateattributes(fileName, {'char', 'string'}, {'nonempty'}, mfilename, ...
    'elements of ptCloudFileNames');
tf = true;

%--------------------------------------------------------------------------
function tf = checkDimensionTolerance(tolerance)
validateattributes(tolerance,{'single','double'}, ...
    {'real', 'nonsparse', 'scalar', 'nonnan', 'finite', 'nonnegative', '<=', 1}, ...
    mfilename, 'tolerance');
tf = true;

%--------------------------------------------------------------------------
function ptCloud = validateAndParsePointCloud(P)

if isa(P, 'pointCloud')
    ptCloud = reshape(P,[],1);
    
elseif ischar(P)
    ptCloud = pcread(P);
elseif iscell(P) % Read from cell
    if ~isempty(P)
        ptCloudCelltype = sum((cellfun(@(P)isa(P,'pointCloud'),P)));
        charCelltype = sum((cellfun(@(P)isa(P,'char'),P)));
        numFrames = numel(P);
        
        % Check if all the elements are point cloud
        if isequal(numFrames, ptCloudCelltype)
            ptCloud = readPtCloudFromCell(P);
        elseif isequal(numFrames, charCelltype)
            % Check if all the elements are path of point cloud
            fileNames = P;
            checkFileNames(P);
            ptCloud = readPtCloudFromFiles(fileNames);
        else % If the cell contains path to read point clouds
            error(message('lidar:lidarCameraCalibration:nonUniformCell'));
        end
    else
        error(message('lidar:lidarCameraCalibration:emptyCell'));
    end
else
    error(message('lidar:lidarCameraCalibration:wrongDatatype'));
end

%--------------------------------------------------------------------------
function ptCloud = readPtCloudFromFiles(fileNames)
ptCloud = pcread(fileNames{1});
for i=2:numel(fileNames)
    ptCloud(i) = pcread(fileNames{i});
end

%--------------------------------------------------------------------------
function ptCloud = readPtCloudFromCell(ptClouds)
ptCloud = ptClouds{1};
for i=2:numel(ptClouds)
    ptCloud(i) = ptClouds{i};
end

%--------------------------------------------------------------------------
function ptCloudLimits = getLimits(pointClouds)
xLimits = [pointClouds.XLimits];
yLimits = [pointClouds.YLimits];
zLimits = [pointClouds.ZLimits];

numPts = numel(xLimits)/2;
% Rearrange limits from 1-by-N array to N/2-by-2 array
xlim = zeros(numPts, 2);
ylim = zeros(numPts, 2);
zlim = zeros(numPts, 2);

xlim(:, 1) = xLimits(1:2:end);
xlim(:, 2) = xLimits(2:2:end);

ylim(:, 1) = yLimits(1:2:end);
ylim(:, 2) = yLimits(2:2:end);

zlim(:, 1) = zLimits(1:2:end);
zlim(:, 2) = zLimits(2:2:end);

% Find the max and min values
ptCloudLimits = [min(xlim(:, 1)), max(xlim(:, 2)), min(ylim(:, 1)), ...
    max(ylim(:, 2)), min(zlim(:, 1)), max(zlim(:, 2))];


%--------------------------------------------------------------------------
function [planes, ptCloudUsed, indicesCell] = detectRectangularPlanePointsImpl(pointClouds,...
    roi, minDistance, dimensionRange, removeGround, verbose)

numFrames = numel(pointClouds);
ptCloudUsed = ones(numFrames, 1);
planes = pointCloud([0,0,0]);

indicesCell = cell(numFrames, 1);
printer = vision.internal.MessagePrinter.configure(verbose);
counter = 1;
for i = 1:numFrames
    ptCloudIn = pointClouds(i);
    [rectangle3d, isptCloudUsed, indices] = lidar.internal.calibration.extractRectangle(ptCloudIn,...
        roi, minDistance, dimensionRange, removeGround);
    if isptCloudUsed
        if verbose
            printer.linebreak;
            printer.printMessage('lidar:lidarCameraCalibration:extractRectangle',counter);
        end
        planes(counter) = rectangle3d;
        indicesCell{counter} = indices;
        counter = counter + 1;
    else
        ptCloudUsed(i) = 0;
    end
end
ptCloudUsed = (logical(ptCloudUsed))';
indicesCell = indicesCell';
% remove empty cells
indicesCell = indicesCell(~cellfun('isempty', indicesCell));
if isequal(planes.Location, [0,0,0])
    planes = [];
end
