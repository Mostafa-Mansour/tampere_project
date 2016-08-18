clear all;
close all;

% video=VideoReader('fc2_save_2016-08-11-124004-0000.avi');
% frames=zeros(video.Height,video.Width,video.Duration*video.FrameRate);
% diff=[];
% 
% check=0;
% 
% matchedCoordinates=zeros(89,10,2);
% limit=.9*10^7;
% 
% our_frames=zeros(1,video.Duration*video.FrameRate);
% for t=1:(video.Duration*video.FrameRate)
%    frames(:,:,t)=rgb2gray(readFrame(video));
%    
%    %% Check whether the robot is stationary or moving %%
%    if t>1
%        d=abs(frames(:,:,t)-frames(:,:,t-1)); %% d - 960*1280
%        diff(t-1)=sum(sum(d(:))); %% 1*99
%        if (check==0) && (diff(t-1)>limit)
%            check=t-1;               
%        end
%    end
% end
% 
% % 89 изображений
% my_images=imresize(frames(:,:,check:end),0.5);

load('my_images.mat');

for t=1:89
    %% detect corners on first image, indexing
    corners_im1=detectHarrisFeatures(my_images(:,:,t),'FilterSize',5);
    [feature_im1,validpoint_im1]=extractFeatures(uint8(my_images(:,:,t)),corners_im1);
    validpoint_strong_im1 = validpoint_im1.selectStrongest(80);
    [ismember_im1,index_im1]=ismember(validpoint_strong_im1.Location(:,1),validpoint_im1.Location(:,1));
    feature_index_im1 = feature_im1.Features(index_im1,:);
    
    %% detect corners on second image, indexing
    corners_im2 = detectHarrisFeatures(my_images(:,:,t+1),'FilterSize',5);
    [feature_im2,validpoint_im2]=extractFeatures(uint8(my_images(:,:,t+1)),corners_im2);
    validpoint_strong_im2 = validpoint_im2.selectStrongest(80);
    [ismember_im2,index_im2]=ismember(validpoint_strong_im2.Location(:,1),validpoint_im2.Location(:,1));
    features_index_im2 = feature_im2.Features(index_im2,:);
    
    % Finding matching pairs of coordinates
    matchedPairs = matchFeatures(feature_index_im1,features_index_im2);
    validPointXY_im1 = validpoint_strong_im1.Location;
    validPointXY_im2 = validpoint_strong_im2.Location;
    matchedPointXY_im1 = validPointXY_im1(matchedPairs(:,1),:);
    matchedPointXY_im2 = validPointXY_im2(matchedPairs(:,2),:);
    %matchedCoordinates = cat(3,matchedPointXY_im1, matchedPointXY_im2);
    %matchedCoordinates(:,:,:,t) = (3,matchedPointXY_im1, matchedPointXY_im2,:);
    figure(1),showMatchedFeatures(my_images(:,:,t),my_images(:,:,t+1),matchedPointXY_im1,matchedPointXY_im2);
%     matchedCoordinates(t,:,:) = matchedPointXY_im1(1:10,:);
%     matchedCoordinates(t+1,:,:)= matchedPointXY_im2(1:10,:);
  %  matchedCoordinates = [matchedCoordinates(:,:,1),matchedCoordinates(:,:,2)];
  %  matchedCoordinates(:,:,t) = [matchedPointXY_ref,matchedPointsXY];
end



% corner_ref=detectHarrisFeatures(my_images(:,:,1),'FilterSize',5);
% [feature_ref,validpoint_ref]=extractFeatures(uint8(my_images(:,:,1)),corner_ref);
% validpoint_ref_ = validpoint_ref.selectStrongest(50);
% 
% [ismember_,index_ref]=ismember(validpoint_ref_.Location(:,1),validpoint_ref.Location(:,1));
% feature_ref_index=feature_ref.Features(index_ref,:);
% 
% for t=2:89
%     corners=detectHarrisFeatures(my_images(:,:,t),'FilterSize',5);
%     [features,validpoints]=extractFeatures(uint8(my_images(:,:,t)),corners);
%     validpoints_ = validpoints.selectStrongest(50);
%     [ismembers_,indexpoints]=ismember(validpoints_.Location(:,1),validpoints.Location(:,1));
%     
%     features_index=features.Features(indexpoints,:);
%     matchedPairs=matchFeatures(feature_ref_index,features_index);
%     matched_ref=validpoint_ref_(double(matchedPairs(:,1)),:);
%     matched=validpoints_(double(matchedPairs(:,2)),:);
%     validPointXY_ref = validpoint_ref_.Location;
%     validPointsXY = validpoints_.Location;
%     matchedPointXY_ref = validPointXY_ref(matchedPairs(:,1),:);
%     matchedPointsXY = validPointsXY(matchedPairs(:,2),:);
%     matchedCoordinates(1,:,:) = matchedPointXY_ref(1:10,:);
%     matchedCoordinates(t,:,:)= matchedPointsXY(1:10,:);
%    % matchedCoordinates = [matchedCoordinates(:,:,1),matchedCoordinates(:,:,2)];
%   %  matchedCoordinates(:,:,t) = [matchedPointXY_ref,matchedPointsXY];
% end




           %%Corner detection%%
%            corners_I1=detectHarrisFeatures(I1,'FilterSize',5);
%            corners_I2=detectHarrisFeatures(I2,'FilterSize',5);
%            corners_I3=detectHarrisFeatures(I3,'FilterSize',5);
%          %  all_corners=detectHarrisFeatures(frames,'FilterSize',5);
%          %  corner_croke=icorner(I1,'nfeat',20);
%            %%Feature Extraction%%
%            [features_I1,validpoints_I1]=extractFeatures(uint8(I1),corners_I1);
%            %figure,idisp(I1),hold on,plot(validpoints_I1.selectStrongest(20));
%            
%            [features_I2,validpoints_I2]=extractFeatures(uint8(I2),corners_I2);
%            
%            [features_I3,validpoints_I3]=extractFeatures(uint8(I3),corners_I3);
%            % figure,idisp(I2),hold on,plot(validpoints_I2);
%            
%            %%Features Matching%%
%            matchedPairs=matchFeatures(features_I1,features_I2,features_I3);
%            
%            matched_1=validpoints_I1(double(matchedPairs(:,1)),:);
%            matched_2=validpoints_I2(double(matchedPairs(:,2)),:);
%            
%            %figure,showMatchedFeatures(I1,I2,matched_1,matched_2);
%            
%            %figure,idisp(I1),hold on,plot(corners_I1.Location(:,1),corners_I1.Location(:,2),'rs');
%            
%            
%            xyPoints1 = validpoints_I1.Location;
%            xyPoints2 = validpoints_I2.Location;
%            numPts = size(xyPoints1,1);
%            matchedPointsArray1 = xyPoints1(matchedPairs(:,1),:);
%            matchedPointsArray2 = xyPoints2(matchedPairs(:,2),:);
%            
%            our_frames(t-1)=matchedPointsArray2;
%            matchedCoordinates = cat(3,matchedPointsArray1, matchedPointsArray2);
%            %figure,idisp(I1),hold on,plot(corners_I1.Location(:,1),corners_I1.Location(:,2),'r*');
%            %figure,idisp(I1),hold on,plot(matchedPointsArray1(:,1),matchedPointsArray1(:,2),'gs');
%            %figure,idisp(I2),hold on,plot(matchedPointsArray2(:,1),matchedPointsArray2(:,2),'bs');
%            %imagecenter(:,:) = (I1.Height/2,I1.Width/2);
%            center1=size(I1)/2;
%            %ptCloud = pointCloud(xyPoints1);
%            if matchedPointsArray1(:,1) < 200
%                pointsOutRad = matchedPointsArray1(:,1);
%            end
%            
%            
%           % pointsxy = cat(3,xyPoints1,xyPoints2);
%            %xyPoints1(:,:,2) = 1:265;
%            %point_number = 1;
%            %features = zeros(t,point_number, t*point_number, t*point_number);
%            %for point_number=1:(20)
%            %    features (:,;,:,:) = (t, point_number,xyPoints1(:,1),xyPoints1(:,2));
%            %features = cat(size(xyPoints1),xyPoints1,xyPoints2);
%          %  release(pointTracker);
%          %  initialize(pointTracker, xyPoints, videoFrameGray);
%        end
%    else
%        checkMotionFrame=t;
       
    







% %%% using Machine vision toolbox from Peter Croke %%%
% video_croke=Movie('fc2_save_2016-08-11-124004-0000.avi');
% frames_croke=zeros(video_croke.height,video_croke.width,video_croke.nframes);
% for t=1:video_croke.nframes
%    frames_croke(:,:,t)=uint8(imono(video_croke.grab('frame',t)));
%     
% end