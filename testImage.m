function result = testImage(camera)
preview(camera);
pause(3)
image = snapshot(camera);
faceDetector = vision.CascadeObjectDetector('FrontalFaceCART');
faceBox = step(faceDetector, image);
attempts = 0;
while isempty(faceBox) && attempts < 10
    pause(0.5)
    image = snapshot(camera);
    faceBox = step(faceDetector, image);
    attempts =  attempts + 1;
end
if isempty(faceBox)
    tryAgain = input('Sorry, I could not find your face. Do you want to try again? y/n', 's');
    if (~isempty(strfind(tryAgain,'y')) || ~isempty(strfind(tryAgain,'Y')))
        closePreview(camera);
        result = testImage(camera);
    elseif (~isempty(strfind(tryAgain,'n')) || ~isempty(strfind(tryAgain,'N')))
        closePreview(camera);
        result = 'Error: could not find face';
    end
else
    closePreview(camera)
    face = imcrop(image, faceBox(1,:));
    scaleFactor = 150/size(face,1);
    face = imresize(face, scaleFactor);
    imshow(face);
    
    training = imageSet('training', 'recursive');
    
    %HOG features
    totalImages = 0;
    for i=1:size(training,2)
        totalImages = totalImages + training(i).Count;
    end
    trainingFeatures = zeros(totalImages,10404);
    featureCount = 1;
    for i=1:size(training,2)
        for j=1:training(i).Count
            hog = extractHOGFeatures(read(training(i),j));
            trainingFeatures(featureCount,:) = hog(1:10404);
            trainingLabel{featureCount} = training(i).Description;
            featureCount = featureCount + 1;
        end
        emotionIndex{i} = training(i).Description;
    end
    
    faceClassifier = fitcecoc(trainingFeatures, trainingLabel);
    
    faceFeatures = extractHOGFeatures(face);
    prediction = predict(faceClassifier, faceFeatures);
    result = prediction{1};
end
end