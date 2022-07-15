function xyzMovingPPF = PPFRegiste(pcdMoving,pcdFixed)

% % single to double
% pcdMoving = pointCloud(double(pcdMoving.Location));
% pcdFixed= pointCloud(double(pcdFixed.Location));

% PPF input pcd, output pcd
% Sample
pcdMovingSampled = pcdownsample(pointCloud(pcdMoving),'random',0.05);
pcdFixedSampled = pcdownsample(pointCloud(pcdFixed),'random',0.05);

% pcd2mesh
meshMovingSampled = pcd2mesh(pcdMovingSampled);
meshFixedSampled = pcd2mesh(pcdFixedSampled);

% initialize detector
dt = PPF3DDetector(0.05,-1,30);
dt = dt.trainModel(meshMovingSampled);
[result, ~, ~] = dt.match(meshFixedSampled,1/5,true,false,true,true, -1, -1);

% Adjust the pose of the model
xyzMovingPPF = TransformPose(pcdMoving.Location,result{1}.pose);
end