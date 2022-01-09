clear 
close all
clc
rng(7); %fix seed to repeat the results
%% Setup
% load the dataset and its parameters

ds = 0; % 0: kitti, 1: malaga, 2: parking, 3: Coop, 4: minecraft
save_or_display = 1; % 0 to save output images in a folder, 1 to display

% kitti
if ds == 0
    % need to set kitti_path to folder containing "05" and "poses"
    assert(exist('datasets/kitti', 'file') ~= 0);
    ground_truth = load('datasets/kitti/poses/05.txt');
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 2760;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];

% malaga
elseif ds == 1
    % path containing the many files of malaga 7
    assert(exist('datasets/malaga', 'file') ~= 0);
    images = dir('datasets/malaga/malaga-urban-dataset-extract-07_rectified_800x600_Images');
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
    
% parking
elseif ds == 2
    % path containing images, depths, and all
    assert(exist('datasets/parking', 'file') ~= 0);
    last_frame = 598;
    K = load('datasets/parking/K.txt');
    ground_truth = load(['datasets/parking/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    
% Coop
elseif ds == 3
    last_frame = 1501;
    K = [1470 0 960
        0  1470 540
        0 0 1];
    images = dir('datasets/coop/');
    images = images(200:2:last_frame);

% Minecraft
elseif ds == 4
    last_frame = 1100;
    images = dir('datasets/mc/');
    images = images(145:last_frame);
    K = [1610 0 1256
    0  1610 720
    0 0 1];
else
    assert(false);
end

% load the parameters of the dataset
pms = getParameters(ds, K);

%% Bootstrap and Initialize
% select two images from the dataset manually
% initialize the contionus VO pipeline by using the selected images

bootstrap_frames = pms.bootstrap_frames; % frames 1 and 3 are used for initialization

% kitti
if ds == 0
    img0 = imread(['datasets/kitti/05/image_0/' ...
        sprintf('%06d.png', bootstrap_frames(1))]);
    img1 = imread(['datasets/kitti//05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
    
% malaga
elseif ds == 1
    img0 = rgb2gray(imread(['datasets/malaga' ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread(['datasets/malaga' ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
    
% parking
elseif ds == 2
    img0 = rgb2gray(imread(['datasets/parking' ...
        sprintf('/images/img_%05d.png', bootstrap_frames(1))]));
    img1 = rgb2gray(imread(['datasets/parking' ...
        sprintf('/images/img_%05d.png', bootstrap_frames(2))]));

% Coop
elseif ds ==3
    img0 = rgb2gray(imread(['datasets/coop/' ...
        images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread(['datasets/coop/' ...
        images(bootstrap_frames(2)).name]));

% Minecraft
elseif ds ==4
    img0 = rgb2gray(imread(['datasets/mc/' ...
        images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread(['datasets/mc/' ...
        images(bootstrap_frames(2)).name]));
else
    assert(false);
end

% keypts_init = 2d keypoints detected in the frames 
% landmarks_init = 3d landmarks corresponding to the 2d keypoints 
% T_w_c_init = transformation matrix from camera to world
[keypts_init, landmarks_init, T_w_c_init] = initialization(img0, img1, pms);

%% Continuous Operation
% process each frame by associating keypoints to existing landmarks,
% estimating the current camera pose by using the landmarks, and regularly
% triangulating new landmarks.

% initialize the previous state 
img_prev = img1; % previous image
Sprev.P = keypts_init; % initial keypoints
Sprev.X = landmarks_init; % initial landmarks
Sprev.C = []; % candidate keypoints
Sprev.T = []; % camera poses at the first observation of the keypoint
Sprev.F = []; % first observations of the track of each keypoint
T_w_c_prev = T_w_c_init; % previous camera pose

num_frames = last_frame - bootstrap_frames(2) + 1; % number of frames
lastframes_ind = 1:20; % indices of last 20 frames
coords = [zeros(2, bootstrap_frames(2) - 1), T_w_c_init([1, 3], 4), zeros(2, num_frames)]; % x and z coordinates of the pose
num_landmarks = [zeros(1, bootstrap_frames(2) - 1), size(Sprev.X, 1), zeros(1, num_frames)]; % number of landmarks

range = (bootstrap_frames(2) + 1):last_frame; % range of frames


for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);

    % kitti
    if ds == 0
        img_curr = imread(['datasets/kitti/05/image_0/' sprintf('%06d.png',i)]);
    
    % malaga
    elseif ds == 1
        img_curr = rgb2gray(imread(['datasets/malaga' ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));

    % parking
    elseif ds == 2
        img_curr = im2uint8(rgb2gray(imread(['datasets/parking' ...
            sprintf('/images/img_%05d.png',i)])));
    % Coop    
    elseif ds == 3
        img_curr = rgb2gray(imread(['datasets/coop/' ...
            images(i).name]));

    % Minecraft
    elseif ds == 4
        img_curr = rgb2gray(imread(['datasets/mc/' ...
            images(i).name]));
    else
        assert(false);
    end


    % process each frame
    [Scurr, T_w_c_curr] = processFrame(img_prev, img_curr, Sprev, T_w_c_prev, pms);
 
    % visualize the trajectory and landmarks
    if i < 20 % start visualizing after first 20 frames
        num_landmarks(i) = size(Scurr.X, 1);
        coords(:, i) = T_w_c_curr([1, 3], 4);
    else
        if i == 20
            fig = figure;
            if ~save_or_display
                set(fig, 'Visible', 'off'); % set visibility off if saving
            end
        end
        [num_landmarks, coords] = visualize(img_curr, Scurr, T_w_c_curr, lastframes_ind, num_landmarks, coords, ds, fig, save_or_display);
        lastframes_ind = lastframes_ind + 1;
    end
    
    % add delay to refresh plots    
    pause(0.05);
    
    % set previous image to current image
    img_prev = img_curr;

    % set previous state to current state
    Sprev = Scurr;

    % set previous pose to current pose
    T_w_c_prev = T_w_c_curr;

end