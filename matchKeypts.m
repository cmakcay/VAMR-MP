function [matched_keypts0, matched_keypts1] = matchKeypts(img0, img1, valid_keypts0, ~, pms)
% This function tracks the keypoints in img0 to img1 by using KLT. The
% point tracker is initialized with the locations of the valid keypoints in
% img0. It returns the locations of the tracked points and the reliability
% of track for each point.

tracker = vision.PointTracker('NumPyramidLevels', pms.numpyramid_levels, 'BlockSize', pms.block_size, ...
    'MaxBidirectionalError', pms.max_bidirecterr, 'MaxIterations', pms.max_iterations);

% initialize the point tracker
initialize(tracker, valid_keypts0.Location, img0);
[keypts1_locs, validity] = tracker(img1);

% store the locations of keypoints
valid_keypts1 = keypts1_locs;

% keep the valid keypoints
matched_keypts0 = valid_keypts0.Location(validity, :);
matched_keypts1 = valid_keypts1(validity, :);

end
