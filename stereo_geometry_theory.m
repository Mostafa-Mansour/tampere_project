% stereokameraparin resoluutio
fov = 47; % field of view asteina (leveys suunnassa)

Xsiz = 2048; % kuvan leveys pikseleinä
f = Xsiz/2/tand(fov/2); %Focal length: pix
B = 30; % Baseline: cm
reprojectionError = 0.5; % arvio virheen määrästä kalibroinnin jälkeen (pikseliä)

subAcc = 1/3; % realistinen ali-pikseli tarkkuus(resoluutio)
disparity = [5:subAcc:500]; % disparity (pikseleitä)

% lasketaan etäisyydet: 
d = f*B./disparity; 
dlow = f*B./(disparity-reprojectionError);
dhigh = f*B./(disparity+reprojectionError);

res = diff(d);

% piirretään resoluutio etäisyyden funktiona
figure,plot(d(2:end),res,'.'),xlabel('distance (cm)'),ylabel('resolution(cm)')
title('resolution in a function of distance')

% piirretään lasketut etäisyydet ja sen varmuusrajat reprojectionErrorin
% avulla
figure,plot(disparity,d),hold on,
plot(disparity,dlow,'r--')
plot(disparity,dhigh,'r--')
title('distance in a function of disparity')
xlabel('disparity (pixel)'),ylabel('distance(cm)')

% Esimerkiksi määrittelemäni (Xsiz =2048, fov = 47, B = 30) 
% stereojärjestelmän resoluutio on 10 m (1000cm ) etäisyydessä 4.8 cm mutta 
% reprojection virheen takia etäisyys voi kuitenkin vaihdella +-8 cm 
% oikeasta arvosta. 
