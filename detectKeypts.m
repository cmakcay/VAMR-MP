function [valid_keypts0, valid_keypts1] = detectKeypts(img0, img1, pms)
% This function detects features in the images and selects them uniformly.

keypts0 = detectHarrisFeatures(img0, 'MinQuality', pms.min_quality);
valid_keypts1 = detectHarrisFeatures(img1, 'MinQuality',pms.min_quality);

valid_keypts0 = selectUniform(keypts0, pms.num_of_features, size(img0));

if pms.vis_intres_init == 1
    figure;
    imshow(img0);
    hold on;
    plot(valid_keypts0);
    title('Valid Keypoints in Image 0');
    hold off;

    figure;
    imshow(img1);
    hold on;
    plot(valid_keypts1);
    title('Valid Keypoints in Image 1');
    hold off;
end

end