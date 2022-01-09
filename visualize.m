function [num_landmarks, coords] = visualize(img_curr, Scurr, T_w_c_curr, lastframes_ind, num_landmarks, coords, ds, fig, save_or_display)
% This function visualizes the current frame, number of landmarks in the
% last frames, full trajectory, and trajectory of last 20 frames and landmarks.

set(gcf, 'Units', 'normalized', 'Position', [0 0 1 1], 'GraphicsSmoothing', 'on');
last_frame = lastframes_ind(end);
num_landmarks(last_frame) = size(Scurr.X, 1);
lastnum_landmarks = num_landmarks(lastframes_ind);
coords(:, last_frame) = T_w_c_curr([1, 3], 4);

subplot(4, 4, [1, 2, 5, 6]);
% if ds == 4 && ~save_or_display
%     frame_num_mc = last_frame + 142;
%     img_curr = imread(['datasets/mc/' sprintf('%08d', frame_num_mc) '.jpg']);
% end
imshow(img_curr);
hold on;
plot(Scurr.P(:, 1), Scurr.P(:, 2), 'Color', 'r', 'Marker', 'o', 'MarkerSize', 10, 'LineStyle', 'none');
plot(Scurr.C(:, 1), Scurr.C(:, 2), 'Color', 'g', 'Marker', 'x', 'MarkerSize', 8, 'LineStyle', 'none');
hold off;
legend('all keypoints', 'all candidates', 'Location', 'northeast');
title(['Current frame: No. ', num2str(last_frame)]);

subplot(4, 4, [9, 13]);
plot(lastframes_ind, lastnum_landmarks, 'Color', 'g');
title('# of landmarks in last 20 frames');

subplot(4, 4, [10, 14]);
hold on;
plot(coords(1, 1:last_frame), coords(2, 1:last_frame), 'Color', 'b');
axis equal;
title('Full Trajectory');
hold off;

f4 = subplot(4, 4, [3, 4, 7, 8, 11, 12, 15, 16]);
cla(f4);
hold on;
plot(coords(1, lastframes_ind), coords(2, lastframes_ind), 'Color', 'k',  'Marker', '*', 'MarkerSize', 4);
plot(Scurr.X(:, 1), Scurr.X(:, 3), 'Color', 'red', 'Marker', '.', 'MarkerSize', 10, 'LineStyle','none');
axis equal;
legend('trajectory', 'landmarks', 'Location', 'northeast');
title('Trajectory of last 20 frames and landmarks');
hold off;

if ~save_or_display
    % folder name to save
    if ds == 0
        folder = "kitti";
    elseif ds == 1
        folder = "malaga";
    elseif ds == 2
        folder = "parking";
    elseif ds == 3
        folder = "coop";
    else
        folder = "mc";
    end

    save_dir = strjoin(["img_", folder], "");
    if ~exist(save_dir, 'dir')
        mkdir(save_dir);
    end
    saveas(fig, strjoin([save_dir, "/", num2str(last_frame),".png"],""));    
end

end
