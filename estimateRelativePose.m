function [orient, loc, inliers0, inliers1] = estimateRelativePose(img0, img1, matched_keypts0, matched_keypts1, pms)
% This function computes the fundamental matrix and finds the inliers by
% RANSAC. It then estimates the relative pose between the images.

[F, inliers_ind] = estimateFundamentalMatrix(matched_keypts0, matched_keypts1, 'Method', pms.fund_method, ...
    'NumTrials', pms.num_trials, 'DistanceThreshold', pms.distance_thres, 'Confidence', pms.conf);

% compute the inliers
inliers0 = matched_keypts0(inliers_ind, :);
inliers1 = matched_keypts1(inliers_ind, :);

% estimate the relative pose between the images
[orient, loc] = relativeCameraPose(F, pms.cameraParams, inliers0, inliers1);

if pms.vis_intres_init == 1
    figure;
    subplot(2, 1, 1);
    showMatchedFeatures(img0, img1, matched_keypts0, matched_keypts1);
    title('Matched Keypoints');
    subplot(2, 1, 2);
    showMatchedFeatures(img0, img1, inliers0, inliers1);
    title('Matched Keypoints after Outlier Removal');
end

end