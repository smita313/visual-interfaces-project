%% Capture 1 image
cam = webcam;
preview(cam)
im1 = snapshot(cam);
clear('cam');

%% Capture 1 image after 3 seconds
cam = webcam;
preview(cam)
pause(3)
im2 = snapshot(cam);
clear('cam');

%% Capture several images in loop
cam = webcam;
preview(cam);
pause(3)
ims = cell(1,50);
for i = 1:50
    ims{i} = snapshot(cam);
    pause(0.1)
end
clear('cam');

%% Capture one image and save just the face
cam = webcam;
preview(cam)
pause(3)
im3 = snapshot(cam);
clear('cam');
faceDetectorFront = vision.CascadeObjectDetector('FrontalFaceCART');
bbox = step(faceDetectorFront, im3);
if(isempty(bbox))
    faceDetectorProfile = vision.CascadeObjectDetector('ProfileFace');
    bbox = step(faceDetectorProfile, im3);
end
if(~isempty(bbox))
    im3 = imcrop(im3, [bbox(1)-50 bbox(2)-50 bbox(3)+50 bbox(4)+50]);
    scaleFactor = 150/size(im3,1);
    im3 = imresize(im3, scaleFactor);
end