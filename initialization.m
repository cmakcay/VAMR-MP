function [keypts_init, landmarks_init, T_w_c_init] = initialization(img0, img1, pms)
% This function takes two images and the parameters to find the initial
% keypoints in the image plane (2d) along with their corresponding landmarks
% in the world reference plane (3d) and compute the transformation matrix
% from camera to world frame.

% detect keypoints
[valid_keypts0, valid_keypts1] = detectKeypts(img0, img1, pms);

% match keypoints in the images using KLT
[matched_keypts0, matched_keypts1] = matchKeypts(img0, img1, valid_keypts0, valid_keypts1, pms);

% estimate the relative pose between the images
[orient, loc, inliers0, inliers1] = estimateRelativePose(img0, img1, matched_keypts0, matched_keypts1, pms);

% triangulate landmarks
[R_w_c, t_w_c, landmarks_init, keypts_init] = triangulateLandmarks(orient, loc, inliers0, inliers1, pms);

% compute transformation matrix ([R|t])
T_w_c_init = [R_w_c, t_w_c; 0 0 0 1];

if pms.vis_init == 1
    figure;
    hold on;
    grid on;
    plotCamera('AbsolutePose', rigid3d(eye(3), [0 0 0]), 'Label', 'Frame0', 'Size', 2, 'AxesVisible', true, 'Color', [0 0 1]);
    plotCamera('AbsolutePose', rigid3d(R_w_c, t_w_c'), 'Label', 'Frame1', 'Size', 2, 'AxesVisible', true, 'Color', [0 0 1]);
    scatter3(landmarks_init(:, 1), landmarks_init(:, 2), landmarks_init(: ,3), 'filled', 'MarkerFaceColor', [1 0 0]);
    xlabel('x'); ylabel('y'); zlabel('z'); % set labels for axes
    xlim([-30, 10]); ylim([-5 2]); zlim([0 100]); % set limits for axes
    view(0, 0); % view point cloud from top
    axis equal;
    title('Initial Landmarks');
end

end