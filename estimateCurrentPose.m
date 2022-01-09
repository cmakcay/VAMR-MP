function [Scurr, T_w_c_curr, pointTracks] = estimateCurrentPose(img_prev, img_curr, matched_keypts0, matched_keypts1, landmarks1, Sprev, pms)
% This function estimates the current pose of the camera by using the
% existing landmarks.

% estimate the current camera pose
[rot, trans, inliers_ind] = estimateWorldCameraPose(matched_keypts1, landmarks1, pms.cameraParams, 'MaxNumTrials', ...
    pms.max_numtrials, 'Confidence', pms.conflev, 'MaxReprojectionError', pms.max_reprojerr);

% find the inliers 
inliers0 = matched_keypts0(inliers_ind, :);
inliers1 = matched_keypts1(inliers_ind, :);

% compute transformation matrix from camera to world
T_w_c_curr = [rot, trans'; 0 0 0 1];

% update the current state
Scurr.P = inliers1;
Scurr.X = landmarks1(inliers_ind, :);
Scurr.C = Sprev.C;
Scurr.F = Sprev.F;
Scurr.T = Sprev.T;

if pms.vis_intres_cont == 1
    figure;
    subplot(2, 1, 1);
    showMatchedFeatures(img_prev, img_curr, matched_keypts0, matched_keypts1);
    title('Matched Keypoints');
    subplot(2, 1, 2);
    showMatchedFeatures(img_prev, img_curr, inliers0, inliers1);
    title('Matched Keypoints after Outlier Removal');
end

% create point tracks for bundle adjustment
pointTracks = [pointTrack([1;2], [inliers0(1,:); inliers1(1,:)])];
for i = 2:length(inliers1)
    pointTracks = [pointTracks; pointTrack([1;2], [inliers0(i,:);inliers1(i,:)])];
end

end
