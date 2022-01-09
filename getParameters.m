function [pms] = getParameters(ds, K)
% This function loads the parameters into a struct.

% Visualization parameters
pms.vis_intres_init = 0; % visualize intermediate results of initialization if set to 1
pms.vis_init = 0; % visualize final results of initialization if set to 1
pms.vis_intres_cont = 0; % visualize intermediate results of continuous operation if set to 1
pms.vis_cont = 1; % visualize final results of continuous operation if set to 1


% kitti
if ds == 0
    % initialization
    pms.bootstrap_frames = [1 3]; % frames 1 and 3 are used for initialization
    pms.cameraParams = cameraParameters('IntrinsicMatrix', K'); % k transpose is used by matlab

    % feature detection and description
    pms.min_quality = 1e-4; % harris min quality
    pms.num_of_features = 600;

    % feature matching by KLT 
    pms.numpyramid_levels = 4; % number of pyramid levels
    pms.block_size = [21 21]; % size of neighborhood around each point being tracked
    pms.max_bidirecterr = 1; % forward-backward error threshold
    pms.max_iterations = 100; % maximum number of search iterations for each points

    % relative pose estimation and triangulation 
    pms.fund_method = 'RANSAC'; % method used to compute the fundamental matrix
    pms.num_trials = 100000; % number of random trials to find the outliers
    pms.conf = 99.9; % desired confidence level for finding the maximum number of inliers
    pms.distance_thres = 0.01; % distance threshold for finding the outliers

    % current pose estimation
    pms.max_numtrials = 100000; % maximum number of random trials
    pms.conflev = 99.9; % desired confidence level for finding the maximum number of inliers
    pms.max_reprojerr = 2; % reprojection error threshold for finding inliers

    % triangulation of new landmarks
    pms.bearingangle_thres = pi / 180; % bearing angle threshold
    pms.dist_thres = 1; % distance threshold for determining new features

    % 2-view bundle adjustment
    pms.ba_max_iterations = 100;
    pms.absolute_tolerance = 0.05;

% malaga
elseif ds == 1 
    % initialization
    pms.bootstrap_frames = [1 4]; % frames 1 and 3 are used for initialization
    pms.cameraParams = cameraParameters('IntrinsicMatrix', K'); % k transpose is used by matlab

    % feature detection and description
    pms.min_quality = 1e-4; % harris min quality
    pms.num_of_features = 400;

    % feature matching by KLT 
    pms.numpyramid_levels = 4; % number of pyramid levels
    pms.block_size = [21 21]; % size of neighborhood around each point being tracked
    pms.max_bidirecterr = 1; % forward-backward error threshold
    pms.max_iterations = 100; % maximum number of search iterations for each points

    % relative pose estimation and triangulation 
    pms.fund_method = 'RANSAC'; % method used to compute the fundamental matrix
    pms.num_trials = 10000; % number of random trials to find the outliers
    pms.conf = 99.9; % desired confidence level for finding the maximum number of inliers
    pms.distance_thres = 0.01; % distance threshold for finding the outliers

    % current pose estimation
    pms.max_numtrials = 10000; % maximum number of random trials
    pms.conflev = 99.9; % desired confidence level for finding the maximum number of inliers
    pms.max_reprojerr = 2; % reprojection error threshold for finding inliers

    % triangulation of new landmarks
    pms.bearingangle_thres = pi / 180; % bearing angle threshold
    pms.dist_thres = 5; % distance threshold for determining new features

    % 2-view bundle adjustment
    pms.ba_max_iterations = 100;
    pms.absolute_tolerance = 0.5;

% parking
elseif ds == 2 
    % initialization
    pms.bootstrap_frames = [1 5]; % frames 1 and 3 are used for initialization
    pms.cameraParams = cameraParameters('IntrinsicMatrix', K'); % k transpose is used by matlab

    % feature detection and description
    pms.min_quality = 1e-4; % harris min quality
    pms.num_of_features = 200;

    % feature matching by KLT 
    pms.numpyramid_levels = 3; % number of pyramid levels
    pms.block_size = [11 11]; % size of neighborhood around each point being tracked
    pms.max_bidirecterr = 1; % forward-backward error threshold
    pms.max_iterations = 50; % maximum number of search iterations for each points

    % relative pose estimation and triangulation 
    pms.fund_method = 'RANSAC'; % method used to compute the fundamental matrix
    pms.num_trials = 10000; % number of random trials to find the outliers
    pms.conf = 99.9; % desired confidence level for finding the maximum number of inliers
    pms.distance_thres = 0.01; % distance threshold for finding the outliers

    % current pose estimation
    pms.max_numtrials = 10000; % maximum number of random trials
    pms.conflev = 99.9; % desired confidence level for finding the maximum number of inliers
    pms.max_reprojerr = 1; % reprojection error threshold for finding inliers

    % triangulation of new landmarks
    pms.bearingangle_thres = pi / 60; % bearing angle threshold
    pms.dist_thres = 1; % distance threshold for determining new features

    % 2-view bundle adjustment
    pms.ba_max_iterations = 100;
    pms.absolute_tolerance = 1;

elseif ds == 3
    % initialization
    pms.bootstrap_frames = [1 3]; % frames 1 and 3 are used for initialization
    pms.cameraParams = cameraParameters('IntrinsicMatrix', K'); % k transpose is used by matlab

    % feature detection and description
    pms.min_quality = 1e-4; % harris min quality
    pms.num_of_features = 500;

    % feature matching by KLT 
    pms.numpyramid_levels = 4; % number of pyramid levels
    pms.block_size = [31 31]; % size of neighborhood around each point being tracked
    pms.max_bidirecterr = 2; % forward-backward error threshold
    pms.max_iterations = 500; % maximum number of search iterations for each points

    % relative pose estimation and triangulation 
    pms.fund_method = 'RANSAC'; % method used to compute the fundamental matrix
    pms.num_trials = 10000; % number of random trials to find the outliers
    pms.conf = 99.9; % desired confidence level for finding the maximum number of inliers
    pms.distance_thres = 0.01; % distance threshold for finding the outliers

    % current pose estimation
    pms.max_numtrials = 10000; % maximum number of random trials
    pms.conflev = 99.9; % desired confidence level for finding the maximum number of inliers
    pms.max_reprojerr = 4; % reprojection error threshold for finding inliers

    % triangulation of new landmarks
    pms.bearingangle_thres = pi / 180; % bearing angle threshold
    pms.dist_thres = 2; % distance threshold for determining new features

    % 2-view bundle adjustment
    pms.ba_max_iterations = 100;
    pms.absolute_tolerance = 0.05;

elseif ds == 4
    % initialization
    pms.bootstrap_frames = [1 3]; % frames 1 and 3 are used for initialization
    pms.cameraParams = cameraParameters('IntrinsicMatrix', K'); % k transpose is used by matlab

    % feature detection and description
    pms.min_quality = 1e-4; % harris min quality
    pms.num_of_features = 500;

    % feature matching by KLT 
    pms.numpyramid_levels = 4; % number of pyramid levels
    pms.block_size = [31 31]; % size of neighborhood around each point being tracked
    pms.max_bidirecterr = 1; % forward-backward error threshold
    pms.max_iterations = 50; % maximum number of search iterations for each points

    % relative pose estimation and triangulation 
    pms.fund_method = 'RANSAC'; % method used to compute the fundamental matrix
    pms.num_trials = 10000; % number of random trials to find the outliers
    pms.conf = 99.9; % desired confidence level for finding the maximum number of inliers
    pms.distance_thres = 0.01; % distance threshold for finding the outliers

    % current pose estimation
    pms.max_numtrials = 10000; % maximum number of random trials
    pms.conflev = 99.9; % desired confidence level for finding the maximum number of inliers
    pms.max_reprojerr = 2; % reprojection error threshold for finding inliers

    % triangulation of new landmarks
    pms.bearingangle_thres = pi / 360; % bearing angle threshold
    pms.dist_thres = 10; % distance threshold for determining new features

    % 2-view bundle adjustment
    pms.ba_max_iterations = 100;
    pms.absolute_tolerance = 1;

end

end