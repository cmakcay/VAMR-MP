function [landmarks1, matched_keypts0, matched_keypts1] = associateKeyptsToLandmarks(img_prev, img_curr, Sprev, pms)
% This function matches the keypoints in img_curr to img_prev by using KLT
% and associates them to existing landmarks.

tracker = vision.PointTracker('NumPyramidLevels', pms.numpyramid_levels, 'BlockSize', pms.block_size, ...
    'MaxBidirectionalError', pms.max_bidirecterr, 'MaxIterations', pms.max_iterations);

% initialize the point tracker
initialize(tracker, Sprev.P, img_prev);
[keyptscurr, validity] = tracker(img_curr);

% keep the valid keypoints
matched_keypts0 = Sprev.P(validity, :);
matched_keypts1 = keyptscurr(validity, :);

% keep the associated landmarks
landmarks1 = Sprev.X(validity, :);

end