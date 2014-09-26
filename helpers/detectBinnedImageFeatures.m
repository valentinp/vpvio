function [ keyPoints ] = detectBinnedImageFeatures( img, featureCount)
%DETECTBINNEDIMAGEFEATURES Summary of this function goes here
%   Detailed explanation goes here

%         keyPoints1 = detectMinEigenFeatures(img, 'ROI', [1 1 640 480]);
%         keyPoints2 = detectMinEigenFeatures(img, 'ROI', [640 1 640 480]);
%         keyPoints3 = detectMinEigenFeatures(img, 'ROI', [1 480 640 480]);
%         keyPoints4 = detectMinEigenFeatures(img, 'ROI', [640 480 640 480]);
%         
%         if featureCount < 4
%             featureCount = 4;
%         end
%         
%         keyPoints = [keyPoints1.selectStrongest(round(featureCount/4)); keyPoints2.selectStrongest(round(featureCount/4)); keyPoints3.selectStrongest(round(featureCount/4)); keyPoints4.selectStrongest(round(featureCount/4))];
    
        %keyPoints = detectMinEigenFeatures(mat2gray(img));
        
       % keyPoints = keyPoints.selectStrongest(featureCount);
        
       
       
%         if featureCount < 4
%             featureCount = 4;
%         end
%         detector = cv.FeatureDetector('FAST');
% 
%         mask = zeros(960,1280);
%         mask(1:480,1:640) = 1;
%         cvKeyPoints = detector.detect(img, 'Mask', mask);
%         [~,sortedKeyPointIds] = sort([cvKeyPoints.response]);
%         cvKeyPoints = cvKeyPoints(sortedKeyPointIds(1:min(round(featureCount/4),length(cvKeyPoints))));
%         location1 = reshape([cvKeyPoints(:).pt], [2, length(cvKeyPoints)])';
%         
%         mask = zeros(960,1280);
%         mask(1:480,640:1280) = 1;
%         cvKeyPoints = detector.detect(img, 'Mask', mask);
%         [~,sortedKeyPointIds] = sort([cvKeyPoints.response]);
%         cvKeyPoints = cvKeyPoints(sortedKeyPointIds(1:min(round(featureCount/4),length(cvKeyPoints))));
%         location2 = reshape([cvKeyPoints(:).pt], [2, length(cvKeyPoints)])';
%         
%         mask = zeros(960,1280);
%         mask(480:960,1:640) = 1;
%         cvKeyPoints = detector.detect(img, 'Mask', mask);
%         [~,sortedKeyPointIds] = sort([cvKeyPoints.response]);
%         cvKeyPoints = cvKeyPoints(sortedKeyPointIds(1:min(round(featureCount/4),length(cvKeyPoints))));
%         location3 = reshape([cvKeyPoints(:).pt], [2, length(cvKeyPoints)])';
%         
%         mask = zeros(960,1280);
%         mask(480:960,640:1280) = 1;
%         cvKeyPoints = detector.detect(img, 'Mask', mask);
%         [~,sortedKeyPointIds] = sort([cvKeyPoints.response]);
%         cvKeyPoints = cvKeyPoints(sortedKeyPointIds(1:min(round(featureCount/4),length(cvKeyPoints))));
%         location4 = reshape([cvKeyPoints(:).pt], [2, length(cvKeyPoints)])';
%         
%       
%         
% 
%         keyPoints.Location = [location1; location2; location3; location4];
% % 
%        plot(location1(:,1), location1(:,2), '*r');
%        hold on;
%        plot(location2(:,1), location2(:,2), '*g');
%        plot(location3(:,1), location3(:,2), '*b');
%        plot(location4(:,1), location4(:,2), '*k');
%        
%        xlim([1,1280])
%        ylim([1,960])
        
         mask = ones(960,1280);
         mask(1:25,:) = 0;
         mask(:,1:25) = 0;
         mask(end-25:end, :) = 0;
         mask(:, end-25:end) = 0;
         
          detector = cv.FeatureDetector('FAST');
         cvKeyPoints = detector.detect(img, 'Mask', mask);
         [~,sortedKeyPointIds] = sort([cvKeyPoints.response]);
         cvKeyPoints = cvKeyPoints(sortedKeyPointIds(1:min(round(featureCount),length(cvKeyPoints))));
         keyPoints.Location = reshape([cvKeyPoints(:).pt], [2, length(cvKeyPoints)])';

        
end

