function [R_w_c, t_w_c, landmarks_init, keypts_init] = triangulateLandmarks(rel_orient, rel_loc, inliers0, inliers1, pms)
% This function triangulates the 2d keypoints into 3d landmarks and finds
% the transformation from camera to world frame.

% find the camera projection matrices (K[R|t])
camMatrix0 = cameraMatrix(pms.cameraParams, eye(3), zeros(1, 3));
camMatrix1 = cameraMatrix(pms.cameraParams, rel_orient', -rel_loc * rel_orient');

% compute rotation and translation from camera to world frame
R_w_c = rel_orient;
t_w_c = rel_loc';

% triangulate landmarks
[landmarks_init, ~, valid_ind] = triangulate(inliers0, inliers1, camMatrix0, camMatrix1);
landmarks_init = landmarks_init(valid_ind, :);

% compute initial keypoints and store their locations
keypts_init = inliers1(valid_ind, :);

end