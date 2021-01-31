ptCloud = pcread('point_cloud/01.pcd');
ptCloud = pcmedian(ptCloud);
ptCloud = removeInvalidPoints(ptCloud);
ptCloud = pcdenoise(ptCloud, 'Threshold' , 0.0001,'NumNeighbors',1);
location = ptCloud.Location;

roi2 = [-1 2 -1.5 2 -1 3];
indicies = findPointsInROI(ptCloud,roi2);
ptCloud = select(ptCloud,indicies);

pcshow(ptCloud)
