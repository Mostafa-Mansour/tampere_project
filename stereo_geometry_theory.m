% stereokameraparin resoluutio
fov = 47; % field of view asteina (leveys suunnassa)

Xsiz = 2048; % kuvan leveys pikselein‰
f = Xsiz/2/tand(fov/2); %Focal length: pix
B = 30; % Baseline: cm
reprojectionError = 0.5; % arvio virheen m‰‰r‰st‰ kalibroinnin j‰lkeen (pikseli‰)

subAcc = 1/3; % realistinen ali-pikseli tarkkuus(resoluutio)
disparity = [5:subAcc:500]; % disparity (pikseleit‰)

% lasketaan et‰isyydet: 
d = f*B./disparity; 
dlow = f*B./(disparity-reprojectionError);
dhigh = f*B./(disparity+reprojectionError);

res = diff(d);

% piirret‰‰n resoluutio et‰isyyden funktiona
figure,plot(d(2:end),res,'.'),xlabel('distance (cm)'),ylabel('resolution(cm)')
title('resolution in a function of distance')

% piirret‰‰n lasketut et‰isyydet ja sen varmuusrajat reprojectionErrorin
% avulla
figure,plot(disparity,d),hold on,
plot(disparity,dlow,'r--')
plot(disparity,dhigh,'r--')
title('distance in a function of disparity')
xlabel('disparity (pixel)'),ylabel('distance(cm)')

% Esimerkiksi m‰‰rittelem‰ni (Xsiz =2048, fov = 47, B = 30) 
% stereoj‰rjestelm‰n resoluutio on 10 m (1000cm ) et‰isyydess‰ 4.8 cm mutta 
% reprojection virheen takia et‰isyys voi kuitenkin vaihdella +-8 cm 
% oikeasta arvosta. 
