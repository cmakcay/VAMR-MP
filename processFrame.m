function [Scurr, T_w_c_curr] = processFrame(img_prev, img_curr, Sprev, T_w_c_prev, pms)
% This function operates the continuous VO pipeline by associating
% keypoints to existing landmarks, estimating the current camera pose by
% using landmarks, and regularly triangulating new landmarks.

% match keypoints in the images and associate them to existing landmarks
[landmarks1, matched_keypts0, matched_keypts1] = associateKeyptsToLandmarks(img_prev, img_curr, Sprev, pms);

% estimate the current camera pose
[Scurr, T_w_c_curr, pointTracks] = estimateCurrentPose(img_prev, img_curr, matched_keypts0, matched_keypts1, landmarks1, Sprev, pms);

% perform 2 view bundle adjustment
[Scurr, refinedPose] = twoViewBA(Scurr, pointTracks, T_w_c_prev, T_w_c_curr, pms);
T_w_c_curr = [cell2mat(refinedPose{2,2}),cell2mat(refinedPose{2,3})';[0, 0, 0, 1]];

% triangulate new landmarks by tracking candidate keypoints
Scurr = triangulateNewLandmarks(img_prev, img_curr, Scurr, T_w_c_curr, pms);

end