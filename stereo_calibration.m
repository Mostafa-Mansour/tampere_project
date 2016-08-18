addpath('C:\Temp\mexopencv-master\')

%pp ='D:\Data\patti\2014-10-27_roboticlab\';
pp ='D:\Data\patti\2014-11-19_hall\calib_data\';

ims38p = dir([pp '*left*.jpg']);
ims39p = dir([pp '*right*.jpg']);
ims40p = dir([pp '*back*.jpg']);

%ims38 = dir([p '14135938*.bmp']);
%ims39 = dir([p '14135939*.bmp']);
% roboticlab = 1105:1290
% hall = [100:140,165:280,310:540]
ims38p = ims38p([100:140,165:280,346:540]);
ims39p = ims39p([100:140,165:280,346:540]);
ims40p = ims40p([100:140,165:280,346:540]);
ims38 = ims38p;
ims39 = ims39p;
ims40 = ims40p;

%% calibrointi pisteet
mm = 70.5;
imsize = [1024 1024];
%patternSize = [5 4]; % neliöiden määrä -1
patternSize = [13 9];

[Yc,Xc] = meshgrid((0:patternSize(2)-1)*mm,(0:patternSize(1)-1)*mm);
objects = [Xc(:),Yc(:),zeros(prod(patternSize),1)];
for k = 1:length(objects)
    obscell{k} = [objects(k,:)];
end




%% kalibrointi erikseen
% --------KAMERA 1----------

obsb = cell(length(ims38p),1);
imagePoints1 = cell(length(ims38p),1);
imagePoints2 = cell(length(ims39p),1);
imagePoints3 = cell(length(ims40p),1);

tveci = 1:2:length(ims38p);
objects2D = zeros(length(objects),2,length(tveci));

ti = 1;
h1 = figure;
for t = tveci
    im1 = imread([pp ims38p(t).name]);
    %im2 = imread([pp ims39p(t).name]);
    imagePoints1{ti} = cv.findChessboardCorners(im1, patternSize);
    imagePoints1{ti} = cv.cornerSubPix(im1,imagePoints1{ti},'WinSize',[4 4]);
    
    for tcell = 1:length(imagePoints1{ti});  objects2D(tcell,:,ti) = imagePoints1{ti}{tcell}; end
    %cameraParameters= estimateCameraParameters(objects2D,objects);
    %imagePoints2{t} = cv.findChessboardCorners(im2, patternSize);
    %imagePoints2{t} = cv.cornerSubPix(im2,imagePoints2{t},'WinSize',[4 4]);
     
     figure(h1),imagesc(im1),colormap gray
     hold on,
     for k = 1:length(imagePoints1{ti})
         plot(imagePoints1{ti}{k}(1)+1,imagePoints1{ti}{k}(2)+1,'bx');
     end
     drawnow
%     figure,imagesc(im2),colormap gray
%     hold on,
%     for k = 1:length(imagePoints2{t})
%         plot(imagePoints2{t}{k}(1)+1,imagePoints2{t}{k}(2)+1,'bx')
%     end
    obsb{ti} = obscell;
    ti = ti + 1;
end
[cameraParameters, imagesUsed]= estimateCameraParameters(objects2D,objects(:,1:2),'EstimateTangentialDistortion',true);
obsb = obsb(1:ti-1);
imagePoints1 = imagePoints1(1:ti-1);
[cameraMatrix1,dist1,d1,rvec1,tvec1] = cv.calibrateCamera(obsb, imagePoints1, imsize,'FixK3',true,'FixK4',true,'FixK5',true,'FixK6',true);

%--------KAMERA 2------------
obsb = cell(length(ims39p),1);
ti = 1;
h2 = figure;
for t = 1:2:length(ims39p)
    %im1 = imread([pp ims38p(t).name]);
    im2 = imread([pp ims39p(t).name]);
    %imagePoints1{t} = cv.findChessboardCorners(im1, patternSize);
    %imagePoints1{t} = cv.cornerSubPix(im1,imagePoints1{t},'WinSize',[4 4]);
    imagePoints2{ti} = cv.findChessboardCorners(im2, patternSize);
    imagePoints2{ti} = cv.cornerSubPix(im2,imagePoints2{ti},'WinSize',[4 4]);
     
    for tcell = 1:length(imagePoints2{ti});  objects2D(tcell,:,ti) = imagePoints2{ti}{tcell}; end
    
    
     figure(h2),imagesc(im2),colormap gray
     hold on,
     for k = 1:length(imagePoints1{ti})
         plot(imagePoints2{ti}{k}(1)+1,imagePoints2{ti}{k}(2)+1,'bx')
     end
%     figure,imagesc(im2),colormap gray
%     hold on,
%     for k = 1:length(imagePoints2{t})
%         plot(imagePoints2{t}{k}(1)+1,imagePoints2{t}{k}(2)+1,'bx')
%     end
    obsb{ti} = obscell;
    ti = ti + 1;
end

[cameraParameters2, imagesUsed2]= estimateCameraParameters(objects2D,objects(:,1:2),'EstimateTangentialDistortion',true);
obsb = obsb(1:ti-1);
imagePoints2 = imagePoints2(1:ti-1);

[cameraMatrix2,dist2,d2,rvec2,tvec2] = cv.calibrateCamera(obsb, imagePoints2, imsize,'FixK3',true,'FixK4',true,'FixK5',true,'FixK6',true);

%--------KAMERA 3-----------
obsb = cell(length(ims40p),1);
ti = 1;
h2 = figure;
for t = 1:2:346
    %im1 = imread([pp ims38p(t).name]);
    im3 = imread([pp ims40p(t).name]);
    %imagePoints1{t} = cv.findChessboardCorners(im1, patternSize);
    %imagePoints1{t} = cv.cornerSubPix(im1,imagePoints1{t},'WinSize',[4 4]);
    imagePoints3{ti} = cv.findChessboardCorners(im3, patternSize);
    imagePoints3{ti} = cv.cornerSubPix(im3,imagePoints3{ti},'WinSize',[4 4]);
     
    for tcell = 1:length(imagePoints3{ti});  objects2D(tcell,:,ti) = imagePoints3{ti}{tcell}; end
    
    
     figure(h2),imagesc(im3),colormap gray
     hold on,
     for k = 1:length(imagePoints3{ti})
         plot(imagePoints3{ti}{k}(1)+1,imagePoints3{ti}{k}(2)+1,'bx')
     end
%     figure,imagesc(im2),colormap gray
%     hold on,
%     for k = 1:length(imagePoints2{t})
%         plot(imagePoints2{t}{k}(1)+1,imagePoints2{t}{k}(2)+1,'bx')
%     end
    obsb{ti} = obscell;
    ti = ti + 1;
end

[cameraParameters3, imagesUsed3]= estimateCameraParameters(objects2D,objects(:,1:2),'EstimateTangentialDistortion',true);
obsb = obsb(1:ti-1);
imagePoints3 = imagePoints3(1:ti-1);

[cameraMatrix3,dist3,d3,rvec3,tvec3] = cv.calibrateCamera(obsb, imagePoints3, imsize,'FixK3',true,'FixK4',true,'FixK5',true,'FixK6',true);


% kalibrointimatriisit erikseen
S = cv.stereoCalibrate(obsb, imagePoints1, imagePoints2, imsize,...
        'CameraMatrix1',cameraMatrix1','CameraMatrix2',cameraMatrix2',...
        'DistCoeffs1',dist1,'DistCoeffs2',dist2,'FixIntrinsic',true,'FixK1',true,'FixK2',true,'FixK3',true,'FixK4',true,'FixK5',true,'FixK6',true);
S = cv.stereoCalibrate(obsb, imagePoints1(1:173), imagePoints2(1:173), imsize);
    

R = zeros(3,3,length(obsb));
Rod = zeros(3,length(obsb));
T = zeros(3,length(obsb));
T2 = zeros(3,length(obsb));
for iii = 1:length(obsb)

    S = cv.stereoCalibrate(obsb(iii), imagePoints2(iii), imagePoints3(iii), imsize,...
        'CameraMatrix1',cameraMatrix2,'CameraMatrix2',cameraMatrix3,...
        'DistCoeffs1',dist2,'DistCoeffs2',dist3,'FixIntrinsic',true,'FixK1',true,'FixK2',true,'FixK3',true,'FixK4',true,'FixK5',true,'FixK6',true);
    %KK1 = cameraMatrix1;
    %KK2 = cameraMatrix3;
    %R1 = rodrigues(rvec1{iii});
    %C1= tvec1{iii};
    %C1 = R1*(-C1); % HUOM! Tämä unohtui harjoituksissa. Eli tässä muunnetaan
    % kamerakeskeinen koordinaatisto kalibrointikohdekeskeiseen.
    
    %R2 = rodrigues(rvec2{iii});
    %C2 = tvec2{iii};
    %C2 = R2*(-C2); % HUOM! sama juttu kun yllä
    %P1 = KK1*[R1' -R1'*C1]; % HUOM! Rotaatiomatriisin pitää myös transponoida
    %P2 = KK2*[R2' -R2'*C2];
    
    % Valitaan manuaalisesti vastinpisteet
%    I1 = imread([pp ims38p(iii).name]);
%    I2 = imread([pp ims39p(iii).name]);
    %I1 = imresize(imread([p names(i1).name]),0.2);
    %I2 = imresize(imread([p names(i2).name]),0.2);
%     h1 = figure;imagesc(I1),colormap gray
%     h2 = figure;imagesc(I2),colormap gray
%     
%     % for loop ja allaolevat alustukset ovat bonustehtävää varten
%     xy = zeros(3,10);
%     XY_real = zeros(4,10);
%     for t = 1:10
%         figure(h1)
%         xy1 = [ginput(1)'; 1];
%         xy(:,t) = xy1;
%         figure(h2)
%         xy2 = [ginput(1)'; 1];
%         
%         a = xy1;
%         %skew = [0 -a(3) a(2);a(3) 0 -a(1);-a(2) a(1) 0];
%         skew = [0 -a(3) a(2);a(3) 0 -a(1)];
%         A1 = skew*P1;
%         a = xy2;
%         %skew = [0 -a(3) a(2);a(3) 0 -a(1);-a(2) a(1) 0];
%         skew = [0 -a(3) a(2);a(3) 0 -a(1)];
%         A2 = skew*P2;
%         A_full = [A1;A2];
%         
%         [U S V] = svd(A_full,0);
%         X = V(:,end);
%         X = X./X(end) % HUOM! tämä tulos on klikatun pisteen etäisyys kalibrointikohteen
%         % vasemmasta yläkulmasta(tarkasti vasemman yläkulman mustan
%         % neliön oikeasta alakulmasta), koska origo sijaitsee siellä.
%         XY_real(:,t) = X;
%     end
    
    %R(:,:,iii) = S.R;
    Rod(:,iii) = rodrigues(S.R);
    %T(:,iii) = C1;
    %T2(:,iii) = C2;
    T3(:,iii) = S.T;
end
figure,plot(T3')
trans = median(T3');
rot = rodrigues(median(Rod(:,Rod(3,:) < 1)'));

%[x,y,z] = dcm2angle(rotLB,'ZXY');
%[x,y,z].*(180./pi) 
d = dcm2quat(rotLB);
[thetaX,thetaY,thetaZ] = quat2angle(d);
[thetaX,thetaY,thetaZ].*(180./pi) 

rot = rotLB;
thetaX = atan2(rot(3,2),rot(3,3))*180/pi;
thetaY = atan2(-rot(3,1),sqrt(rot(3,2)^2+rot(3,3)^2))*180/pi;
thetaZ = atan2(rot(2,1),rot(1,1))*180/pi;

save calib_hall cameraMatrix1 cameraMatrix2 cameraMatrix3 transLR rotLR ...
    transLB rotLB transRB rotRB dist1 dist2 dist3 d3 d2 d1
%,'FixIntrinsic',true,'UseIntrinsicGuess',true
% tarkistus: kameran lokaatio
figure,plot_extrinsic(objects',rvec1,cameraMatrix1,tvec1,imsize,patternSize,mm);
figure,plot_extrinsic(objects',rvec2,cameraMatrix2,tvec2,imsize,patternSize,mm);


%% kameroiden kalibrointi yhdessä
% tämähän on täysin sama vastinpisteiden etsinnän ja siitä johdetun
% rotaatiomatriisin ja translaatiomatriisin kanssa kahden eri kameran välillä. 
% Mutta käytetään nyt tätä opencv funktiota, niin päästään helpommalla.

mm = 70.5;
imsize = [1024 1024];
%patternSize = [5 4]; % neliöiden määrä -1
patternSize = [13 9];

obsb = cell(length(ims38),1);
imagePoints1 = cell(length(ims38),1);
imagePoints2 = cell(length(ims38),1);
%patternSize = [5 4];
h1 = figure;
h2 = figure;
for t = 1:4:length(ims38)
    im1 = imread([pp ims38(t).name]);
    im2 = imread([pp ims39(t).name]);
    imagePoints1{t} = cv.findChessboardCorners(im1, patternSize);
    imagePoints1{t} = cv.cornerSubPix(im1,imagePoints1{t},'WinSize',[4 4]);
    imagePoints2{t} = cv.findChessboardCorners(im2, patternSize);
    imagePoints2{t} = cv.cornerSubPix(im2,imagePoints2{t},'WinSize',[4 4]);
     
    figure(h1),imagesc(im1),colormap gray
    hold on,
    for k = 1:length(imagePoints1{t})
        plot(imagePoints1{t}{k}(1)+1,imagePoints1{t}{k}(2)+1,'bx')
    end
    figure(h2),imagesc(im2),colormap gray
    hold on,
    for k = 1:length(imagePoints2{t})
        plot(imagePoints2{t}{k}(1)+1,imagePoints2{t}{k}(2)+1,'bx')
    end
    obsb{t} = obscell;
end

% stereokalibrointi
S = cv.stereoCalibrate(obsb, imagePoints1, imagePoints2, [2048 2048],...
    'CameraMatrix1',cameraMatrix1,'CameraMatrix2',cameraMatrix2,...
    'DistCoeffs1',dist1,'DistCoeffs2',dist2,'FixIntrinsic',true,'UseIntrinsicGuess',true) 
%save calib_rig2 S
% load calib_rig
SR = cv.stereoRectify(cameraMatrix1, dist1, cameraMatrix2, dist2, [2048 2048], S.R, S.T);
%% Lasketaan 3D pisteitä
pi = '\\intra.tut.fi\home\raunio2\My Documents\MATLAB\projektit\patti\images\\';
delete([pi(1:end-1) '*.bmp']);
dos(['C:\Temp\kamera_juttuja\32bit\bin\AFlyCapture2Test_jp.exe 1 "' pi '"']);
right_name = dir([pi '14135939*.bmp']);
left_name = dir([pi '14135938*.bmp']);

right = imread([pi right_name(1).name]);
left = imread([pi left_name(1).name]);

[map1, map3] = cv.initUndistortRectifyMap(cameraMatrix1, dist1, SR.P1, [2048 2048],'R',SR.R1);
[map2, map4] = cv.initUndistortRectifyMap(cameraMatrix2, dist2, SR.P2, [2048 2048],'R',SR.R2);
leftr = cv.remap(left, map1(:,:,1), map1(:,:,2));
rightr = cv.remap(right, map2(:,:,1), map2(:,:,2));


figure,h1 = axes; imagesc(leftr),colormap gray,axis image
figure,h2 = axes; imagesc(rightr),colormap gray,axis image
%Srec = cv.stereoRectify(S.cameraMatrix1, S.distCoeffs1, S.cameraMatrix2, S.distCoeffs2, [2048 2048], S.R, S.T);
%[map1, map2] = cv.initUndistortRectifyMap(S.cameraMatrix2, S.distCoeffs2, Srec.P2, [2048 2048]);
%dst = cv.remap(left, map1, map2);
left = rgb2gray(imread('vipstereo_hallwayLeft.png'));
right = rgb2gray(imread('vipstereo_hallwayRight.png'));
rightI3chan = (imread('vipstereo_hallwayRight.png'));

% dense
%[db,dbm,ddyn,ddynm] = blockmatching_matlabdemo(left,right,10,200,1,15,1/4);

% sparse
%blobs1jp = find_features(single(left),30); 
%blobsh = detectHarrisFeatures(left);
blobs1 = detectSURFFeatures(left);
blobs2 = detectSURFFeatures(right);

figure; imshow(left); hold on;
plot(blobs1.selectStrongest(1000));
%plot(blobs1jp(:,1),blobs1jp(:,2),'.')
title('Thirty strongest SURF features in I1');

figure; imshow(right); hold on;
plot(blobs2.selectStrongest(1000));
title('Thirty strongest SURF features in I2');

[features1, validBlobs1] = extractFeatures(left, blobs1);
[features2, validBlobs2] = extractFeatures(right, blobs2);
% Use the sum of absolute differences (SAD) metric to determine indices of matching features.
indexPairs = matchFeatures(features1, features2, 'Metric', 'SAD', ...
  'MatchThreshold', 5);
%Retrieve locations of matched points for each image
matchedPoints1 = validBlobs1(indexPairs(:,1),:);
matchedPoints2 = validBlobs2(indexPairs(:,2),:);
% matching points in images
figure; showMatchedFeatures(left, right, matchedPoints1, matchedPoints2);
legend('Putatively matched points in I1', 'Putatively matched points in I2');

R1 = [1 0 0; 0 1 0; 0 0 1]; R2 = S.R; C1 = [0 0 0]'; C2 = S.T;% C2(1) = -C2(1); 
RC1 = [R1, -C1];
P1 = S.cameraMatrix1*RC1;
RC2 = [R2, -C2];
P2 = S.cameraMatrix2*RC2;

%X3D = manual_3Dpoint(h1,h2,P1,P2); 
xy1 = [matchedPoints1.Location'; ones(1,length(matchedPoints1))];
xy2 = [matchedPoints2.Location'; ones(1,length(matchedPoints2))];
%x1 = xy1; x2 = xy2;

% lets solve the fundamental matrix and the 3D points, (Hartley and
% Zissermann page 312)
X_reconstruction = triangulateP({P1, P2}, {xy1,xy2});
X_reconstruction = bsxfun(@rdivide, X_reconstruction, X_reconstruction(end,:));


figure,plot3(X_reconstruction(3,:),X_reconstruction(1,:),X_reconstruction(2,:),'.')
set(gca,'ydir','reverse'),xlabel('x'),ylabel('y'),zlabel('z')
%set(gca,'zdir','reverse')
title('3D data'),axis image

%% kameran systeemin liike ja rotaatio kuvien välillä ja uudet 3D pisteet 
% samaan koordinaatistoon ( =X_new)
yT = 0;
xT = 0;
zT = -2000;
xrot = 20;
yrot = -10;
zrot = 0;
Rx = [1 0 0;0 cosd(xrot) -sind(xrot); 0 sind(xrot) cosd(xrot)];
Ry = [cosd(yrot) 0 sind(yrot);0 1 0; -sind(yrot) 0 cosd(yrot)];
Rz = [cosd(zrot) -sind(zrot) 0; sind(zrot) cosd(zrot) 0;0 0 1];
R = Rx*Ry*Rz;
X_new = R*X_reconstruction(1:3,:);
X_new = [X_new(1,:)-xT; X_new(2,:)-xT; X_new(3,:)-zT];
figure,plot3(X_new(3,:),X_new(1,:),X_new(2,:),'.')
set(gca,'ydir','reverse'),xlabel('x'),ylabel('y'),zlabel('z')
%set(gca,'zdir','reverse')
title('3D data'),axis image


jump = 10;
Baseline = 10;
[Xi,Yi] = meshgrid(-size(left,1)/2:jump:size(left,1)/2,-size(left,2)/2:jump:size(left,2)/2);
Z = double(Baseline*KK(1,1)./ddyn);
X = double((Z.*Xi)./KK(1,1));
Y = double((Z.*Yi)./KK(1,1));
figure,imagesc(Z)
figure,plot3(X(1:end),Y(1:end),Z(1:end),'.'),zlim([0 5500])

bm = cv.StereoBM('Preset', 'Basic', 'NDisparities',240,'SADWindowSize',31);
bm = cv.StereoSGBM('MinDisparity',20,'NumDisparities',240,'SADWindowSize',31,...
    'P2',20,'P1',5,'UniquenessRatio',10);
disparity = bm.compute(leftr, rightr);
figure,imagesc(disparity/16)

% matlab esimerkin mukaan
% Camera matrix
K = [409.4433         0    0
            0  416.0865    0
     204.1225  146.4133    1];

% Create a sub-sampled grid for backprojection.
dec = 2;
[X,Y] = meshgrid(1:dec:size(left,2),1:dec:size(left,1));
P = [X(:), Y(:), ones(numel(X), 1, 'single')] / K;
Disp = max(0,ddynm(1:dec:size(left,1),1:dec:size(left,2)));
% Derive conversion from disparity to depth with tie points:
knownDs = [15   9   2]'; % Disparity values in pixels
knownZs = [4  4.5 6.8]';
% World z values in meters based on scene measurements.
ab = [1./knownDs ones(size(knownDs), 'single')] \ knownZs; % least squares
ZZ = zeros(size(Disp));
% Convert disparity to z (distance from camera)
ZZ(:) = ab(1)./Disp(:) + ab(2);
% Threshold to [0,8] meters.
ZZdisp = min(8,max(0, ZZ ));
Pd = bsxfun(@times,P,ZZ(:));
% Remove near points
bad = Pd(:, 3)>8 | Pd(:, 3)<3;
Pd = Pd(~bad, :);

% Collect quantized colors for point display
Colors = rightI3chan(1:dec:size(right,1),1:dec:size(right,2),:);
Colors = reshape(Colors,[size(Colors,1)*size(Colors,2) size(Colors,3)]);
Colors = single(Colors(~bad,:))./255;

figure, hold on, axis ;
for i=1:size(Pd,1)
    plot3(-Pd(i,1),-Pd(i,3),-Pd(i,2),'.','Color',Colors(i,:));
end
view(161,14), grid on;
xlabel('x (meters)'), ylabel('z (meters)'), zlabel('y (meters)');

%% 3D mapping useasta kuvaparista
%otetaan kuvat
pi = '\\intra.tut.fi\home\raunio2\My Documents\MATLAB\projektit\patti\images\\';
delete([pi(1:end-1) '*.bmp']);
dos(['C:\Temp\kamera_juttuja\32bit\bin\AFlyCapture2Test_jp.exe 1 "' pi '"']);

right_name = dir([pi '14135939*.bmp']);
left_name = dir([pi '14135938*.bmp']);

T = [0 0 0; 0 0 1950;0 0 950;];
R = [0 0 0; 0 -90 0; 0 0 0];
colors = 'rgb';
figure,
for t = 1:length(right_name)
    right = imread([pi right_name(t).name]);
    left = imread([pi left_name(t).name]);
    
    %blobsh = detectHarrisFeatures(left);
    blobs1 = detectSURFFeatures(left);
    blobs2 = detectSURFFeatures(right);
    
    [features1, validBlobs1] = extractFeatures(left, blobs1);
    [features2, validBlobs2] = extractFeatures(right, blobs2);
    % Use the sum of absolute differences (SAD) metric to determine indices of matching features.
    indexPairs = matchFeatures(features1, features2, 'Metric', 'SAD', ...
        'MatchThreshold', 5);
    %Retrieve locations of matched points for each image
    matchedPoints1 = validBlobs1(indexPairs(:,1),:);
    matchedPoints2 = validBlobs2(indexPairs(:,2),:);
    % matching points in images
    %figure; showMatchedFeatures(left, right, matchedPoints1, matchedPoints2);
    %legend('Putatively matched points in left', 'Putatively matched points in right');
    
    R1 = [1 0 0; 0 1 0; 0 0 1]; R2 = S.R; C1 = [0 0 0]'; C2 = S.T;% C2(1) = -C2(1);
    RC1 = [R1, -C1];
    P1 = S.cameraMatrix1*RC1;
    RC2 = [R2, -C2];
    P2 = S.cameraMatrix2*RC2;
    
    %X3D = manual_3Dpoint(h1,h2,P1,P2);
    xy1 = [matchedPoints1.Location'; ones(1,length(matchedPoints1))];
    xy2 = [matchedPoints2.Location'; ones(1,length(matchedPoints2))];
    %x1 = xy1; x2 = xy2;
    
    % lets solve the fundamental matrix and the 3D points, (Hartley and
    % Zissermann page 312)
    X_reconstruction = triangulateP({P1, P2}, {xy1,xy2});
    X_reconstruction = bsxfun(@rdivide, X_reconstruction, X_reconstruction(end,:));
    
    
    %figure,plot3(X_reconstruction(3,:),X_reconstruction(1,:),X_reconstruction(2,:),'.')
    %set(gca,'ydir','reverse'),xlabel('x'),ylabel('y'),zlabel('z')
    %set(gca,'zdir','reverse')
    %title('3D data'),axis image
    
    % kameran systeemin liike ja rotaatio kuvien välillä ja uudet 3D pisteet
    % samaan koordinaatistoon ( =X_new)
    yT = T(t,1);
    xT = T(t,2);
    zT = T(t,3);
    xrot = R(t,1);
    yrot = R(t,2);
    zrot = R(t,3);
    Rx = [1 0 0;0 cosd(xrot) -sind(xrot); 0 sind(xrot) cosd(xrot)];
    Ry = [cosd(yrot) 0 sind(yrot);0 1 0; -sind(yrot) 0 cosd(yrot)];
    Rz = [cosd(zrot) -sind(zrot) 0; sind(zrot) cosd(zrot) 0;0 0 1];
    Rall = Rx*Ry*Rz;
    X_new = Rall*X_reconstruction(1:3,:);
    X_new = [X_new(1,:)-xT; X_new(2,:)-xT; X_new(3,:)-zT];
    hold on, plot3(X_new(3,:),X_new(1,:),X_new(2,:),'.','color',colors(t))
    set(gca,'ydir','reverse'),xlabel('x'),ylabel('y'),zlabel('z')
    %set(gca,'zdir','reverse')
    title('3D data'),axis image
end

