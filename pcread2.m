function xyz = pcread2(path)

% Judge loading mode
[~,~,type] = fileparts(path);
isXYZ = any(contains(['.txt','.xyz','.asc','.pts','.neu','.csv'],type));
isPCDorPLY = any(contains(['.pcd','.ply'],type));

if isXYZ
    xyzPoints = importdata(path);
    if size(xyzPoints,2)>=3
        xyz = xyzPoints(:,1:3);
    else
        disp('error! This file is not recognized')        
    end

elseif isPCDorPLY
    pcdPoints = pcread(path);
    xyz = double(pcdPoints.Location);

% error
else
    disp('error-->this type is not supported')
    xyz = [];
end

% if max((max(xyz)-min(xyz)))<1  【需要适应超小尺寸的物体】
%     xyz = xyz.*100;
% end
xyz = single(xyz);
end

