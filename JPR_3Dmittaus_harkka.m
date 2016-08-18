%% -----kameroiden kalibrointi----
p = 'C:\Temp\harkka\';

names = dir([p '*.JPG']);

imagepaths = cell(1,8);
imagePoints = [];
for t = 3:10
    imagepaths{t-2} = [p names(t).name];
    kuva = imread(imagepaths{t-2});
    kuva = imresize(kuva,0.2);
    [imagePoints(:,:,t-2), boardSize] = detectCheckerboardPoints(kuva);
    
    figure,imagesc(kuva),hold on,plot(imagePoints(:,1,t-2),imagePoints(:,2,t-2),'ro')
end

squareSize = 31; % millimetreissä
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
[cameraParameters, imagesUsed]= estimateCameraParameters(imagePoints,worldPoints);

showExtrinsics(cameraParameters,'PatternCentric')
showExtrinsics(cameraParameters,'CameraCentric')

%% ----Stereokuvapari-----

I1 = imresize(imread([p names(1).name]),0.2);
I2 = imresize(imread([p names(2).name]),0.2);
h1 = figure;imagesc(I1),colormap gray
h2 = figure;imagesc(I2),colormap gray

I1 = rgb2gray(I1);
I2 = rgb2gray(I2);

KK = cameraParameters.IntrinsicMatrix;
% camera matrices P = KK[R|C] where R is a rotation matrix and C is
% translation. 
R = [1 0 0;0 1 0; 0 0 1]; C1 = [0 0 0]'; C2 = [100 0 0]'; T = C2-C1;
P1 = KK'*[R -R*C1];
P2 = KK'*[R -R*C2];

% Valitaan manuaalisesti vastinpisteet
figure(h1)
xy1 = [ginput(1)'; 1];
figure(h2)
xy2 = [ginput(1)'; 1];

% lets solve the fundamental matrix and the 3D points, (Hartley and
% Zissermann page 312)
a = xy1;
%skew = [0 -a(3) a(2);a(3) 0 -a(1);-a(2) a(1) 0];
skew = [0 -a(3) a(2);a(3) 0 -a(1)];
A1 = skew*P1;
a = xy2;
%skew = [0 -a(3) a(2);a(3) 0 -a(1);-a(2) a(1) 0];
skew = [0 -a(3) a(2);a(3) 0 -a(1)];
A2 = skew*P2;
A_full = [A1;A2];

[U S V] = svd(A_full,0);
X = V(:,end);
X = X./X(end) % result: 3D point, z coordinate tells the depth

%% otetaan cameraparameters muuttujasta rotaatio ja translaatio ja 
% lasketaan 3D pisteitä kuvaparista
i1 = 3;
i2 = 6;

KK = cameraParameters.IntrinsicMatrix;
R1 = cameraParameters.RotationMatrices(:,:,i1-2);
C1= cameraParameters.TranslationVectors(i1-2,:);
C1 = R1*(-C1)'; % HUOM! Tämä unohtui harjoituksissa. Eli tässä muunnetaan 
            % kamerakeskeinen koordinaatisto kalibrointikohdekeskeiseen.
R2 = cameraParameters.RotationMatrices(:,:,i2-2); 

C2 = cameraParameters.TranslationVectors(i2-2,:);
C2 = R2*(-C2)'; % HUOM! sama juttu kun yllä
P1 = KK'*[R1' -R1'*C1]; % HUOM! Rotaatiomatriisin pitää myös transponoida
P2 = KK'*[R2' -R2'*C2];

% Valitaan manuaalisesti vastinpisteet
I1 = imresize(imread([p names(i1).name]),0.2);
I2 = imresize(imread([p names(i2).name]),0.2);
h1 = figure;imagesc(I1),colormap gray
h2 = figure;imagesc(I2),colormap gray

% for loop ja allaolevat alustukset ovat bonustehtävää varten
xy = zeros(3,10);
XY_real = zeros(4,10);
for t = 1:10
    figure(h1)
    xy1 = [ginput(1)'; 1];
    xy(:,t) = xy1;
    figure(h2)
    xy2 = [ginput(1)'; 1];
    
    a = xy1;
    %skew = [0 -a(3) a(2);a(3) 0 -a(1);-a(2) a(1) 0];
    skew = [0 -a(3) a(2);a(3) 0 -a(1)];
    A1 = skew*P1;
    a = xy2;
    %skew = [0 -a(3) a(2);a(3) 0 -a(1);-a(2) a(1) 0];
    skew = [0 -a(3) a(2);a(3) 0 -a(1)];
    A2 = skew*P2;
    A_full = [A1;A2];
    
    [U S V] = svd(A_full,0);
    X = V(:,end);
    X = X./X(end) % HUOM! tämä tulos on klikatun pisteen etäisyys kalibrointikohteen
    % vasemmasta yläkulmasta(tarkasti vasemman yläkulman mustan
    % neliön oikeasta alakulmasta), koska origo sijaitsee siellä.
    XY_real(:,t) = X;
end


%% BONUS TEHTÄVÄ
% Lasketaan vielä kameran rotaatio ja translaatio 2D ja 3D vastinpisteiden
% avulla.

% xy (2D pisteet) ja XY_real(3D pisteet) saadaan edellisestä tehtäväst.

% Nyt jokaiselle pisteelle voidaan muodostaa matriisi A jotka päällekkäin
% kasaamalla saadaan matriisi A_full
% 
nump = size(xy,2); %Pisteiden lukumäärä
A_full = []; %tähän kasatut matriisit A
X_ = XY_real;
for ii = 1:nump
    X_single = X_(:,ii);
    
    X = X_single(1);
    Y = X_single(2);
    Z = X_single(3);
    W = X_single(4);
    
    x_single = xy(:,ii);
    
    x = x_single(1);
    y = x_single(2);
    w = x_single(3);
    
    A = [0 0 0 0 -X*w -Y*w -Z*w -W*w X*y Y*y Z*y W*y;
        X*w Y*w Z*w W*w 0 0 0 0 -X*x -Y*x -Z*x -W*x;
        -X*y -Y*y -Z*y -W*y X*x Y*x Z*x W*x 0 0 0 0];
    A_full = [A_full;A];
end

% Nyt voimme etsiä vektorin p joka minimoi yhtälön A_full*p. Käytetään
% SVD:tä

[U S V] = svd(A_full, 0);
p1 = V(:,end); %Oikeanpuoleinen singulaarivektori joka vastaa pienintä
%           %singulaariarvoa (singulaariarvojen järjestys mainittu 
%           %'help svd'
P1 = zeros(3,4);  %p1 järjestettynä 3x4 matriisimuotoon
P1 = P1'; %Kikkailaan niin että rivit täyttyvät ennen sarakkeita
P1(:) = p1;
P = P1';

%translaatio (H&Z s. 163-164)
X1 = det([P(:,2),P(:,3),P(:,4)]);
Y1 = det([P(:,1),P(:,3),P(:,4)]);
Z1 = det([P(:,1),P(:,2),P(:,4)]);
W = det([P(:,1),P(:,2),P(:,3)]);
X1 = X1/W;
Y1 = Y1/W;
Z1 = Z1/W;

%rotaatio rq hajotelman avulla (H&Z s. 163-164)
Ps = P(:,1:3);
[m n]=size(Ps);
% matlabissa ei ole rq hajotelmaa, joten tehdään hajotelma qr:n avulla
[Qv Rv]=qr(flipud(Ps).');
Rv=flipud(Rv.');
Rv(:,1:m)=Rv(:,m:-1:1);
Qv=Qv.';
Qv(1:m,:)=Qv(m:-1:1,:) % Tämä on rotaatiomatriisi

%HUOM!!! Koska klikkasimme vastinpisteet käsin, estimoidut rotaatio ja
%translaatio eivät ole aivan samat kuin alunperin lasketut R1 ja C1
