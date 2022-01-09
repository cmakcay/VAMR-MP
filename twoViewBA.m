function [Scurr, refinedPose] = twoViewBA(Scurr, pointTracks, T_w_c_prev, T_w_c_curr, pms) 
% This function performs two view bundle adjustment in order to refine the
% landmarks and poses.

% IDs of last two frames
ViewId = [uint32(1); uint32(2)];

% orientation and location of last two frames
Orientation = [{T_w_c_prev(1:3,1:3)}; {T_w_c_curr(1:3,1:3)}];
Location = [{T_w_c_prev(1:3,4)'}; {T_w_c_curr(1:3,4)'}];

% generate camera poses
cameraPoses = table(ViewId, Orientation, Location);

% perform two view bundle adjustment
[xyzRefined, refinedPose] = bundleAdjustment(Scurr.X, pointTracks, cameraPoses, pms.cameraParams, 'MaxIterations', pms.ba_max_iterations, 'AbsoluteTolerance', pms.absolute_tolerance);

% refine the landmarks
Scurr.X = xyzRefined;

end