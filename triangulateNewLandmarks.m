function Scurr = triangulateNewLandmarks(img_prev, img_curr, Scurr, T_w_c_curr, pms)
% This function triangulates new landmarks and updates the current state.

% if there are candidate keypoints, track them to the current frame
if Scurr.C
    % initialize the KLT
    tracker_old_candidates = vision.PointTracker('NumPyramidLevels', pms.numpyramid_levels, 'BlockSize', pms.block_size, ...
        'MaxBidirectionalError', pms.max_bidirecterr, 'MaxIterations', pms.max_iterations);
    
    % track the candidate points from previous frame to current frame
    initialize(tracker_old_candidates, Scurr.C, img_prev);
    [candidates, validity_candidates] = tracker_old_candidates(img_curr);
    
    % update candidate keypoints
    Scurr.C = candidates(validity_candidates, :);
    Scurr.F = Scurr.F(validity_candidates, :);
    Scurr.T = Scurr.T(:, :, validity_candidates);
end

newkeypts_ind = []; % indices of candidate keypoints that satisfy to be real keypoints
for i = 1:size(Scurr.C, 1)
    % compute rotation and translation from world to camera frame
    camMatrix0 = cameraMatrix(pms.cameraParams, Scurr.T(1:3, 1:3, i)', -Scurr.T(1:3, 4, i)' * Scurr.T(1:3, 1:3, i)');
    
    % compute rotation and translation from world to camera frame
    camMatrix1 = cameraMatrix(pms.cameraParams, T_w_c_curr(1:3, 1:3)', -T_w_c_curr(1:3, 4)' * T_w_c_curr(1:3, 1:3)');

    % triangulate candidate landmarks
    [landmark_candids, ~, valid_ind] = triangulate(Scurr.F(i, :), Scurr.C(i, :), camMatrix0, camMatrix1);
    landmark_candids = landmark_candids(valid_ind, :);

    % compute bearing vectors to find bearing angle
    ray0 = Scurr.T(1:3, 4, i) - landmark_candids'; 
    ray1 = T_w_c_curr(1:3, 4) - landmark_candids'; 
    
    % compute bearing angle
    alpha = acos((ray0'*ray1) / (norm(ray0) * norm(ray1)));

    if alpha > pms.bearingangle_thres
        newkeypts_ind = [newkeypts_ind, i];
        % add candidate keypoints to real keypoints
        Scurr.P = [Scurr.P; Scurr.C(i, :)];
        
        % add candidate landmarks to real landmarks
        Scurr.X = [Scurr.X; landmark_candids];
    end
end

% update candidate keypoints
Scurr.C(newkeypts_ind, :) = [];
Scurr.F(newkeypts_ind, :) = [];
Scurr.T(:, :, newkeypts_ind) = [];

% find all features in the current frame
features_curr = detectHarrisFeatures(img_curr, 'MinQuality', pms.min_quality);
features_curr = selectUniform(features_curr, pms.num_of_features, size(img_curr));

all_keypts = [Scurr.P; Scurr.C]; % all keypoints
unique_features = []; % new candidate keypoints that are not in all keypoints
for i = 1:size(features_curr, 1)  
    % compute distance between new features and all keypoints
    distance = sqrt(sum((features_curr.Location(i, :) - all_keypts).^2, 2));
    
    % if the distance is too smaller than the threshold, the feature is not new
    if nnz(~(distance > pms.dist_thres)) == 0
        unique_features(end + 1, :) = features_curr.Location(i, :); 
    end
end

% update candidate keypoints
Scurr.C = [Scurr.C; unique_features];
Scurr.F = [Scurr.F; unique_features];
Scurr.T = cat(3, Scurr.T, repmat(T_w_c_curr, 1, 1, size(unique_features, 1)));


end