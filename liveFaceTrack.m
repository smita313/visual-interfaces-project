function liveFaceTrack(folderName, cam)
% Create the face detector object.
faceDetector = vision.CascadeObjectDetector();

% Create the point tracker object.
pointTracker = vision.PointTracker('MaxBidirectionalError', 2);

% Capture one frame to get its size.
videoFrame = snapshot(cam);
frameSize = size(videoFrame);

% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);

runLoop = true;
numPts = 0;
frameCount = 0;

i = 0;
while runLoop && frameCount < 100
    i = i+1;
    
    % Get the next frame.
    videoFrame = snapshot(cam);
    videoFrameGray = rgb2gray(videoFrame);
    frameCount = frameCount + 1;

    if numPts < 10
        % Detection mode.
        bbox = faceDetector.step(videoFrameGray);

        if ~isempty(bbox)
            % Find corner points inside the detected region.
            points = detectMinEigenFeatures(videoFrameGray, 'ROI', bbox(1, :));

            % Re-initialize the point tracker.
            xyPoints = points.Location;
            numPts = size(xyPoints,1);
            release(pointTracker);
            initialize(pointTracker, xyPoints, videoFrameGray);

            % Save a copy of the points.
            oldPoints = xyPoints;

            % Convert the rectangle represented as [x, y, w, h] into an
            % M-by-2 matrix of [x,y] coordinates of the four corners. This
            % is needed to be able to transform the bounding box to display
            % the orientation of the face.
            bboxPoints = bbox2points(bbox(1, :));

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon = reshape(bboxPoints', 1, []);
            
            cropSize = size(videoFrame);
            invbboxPoints = bboxPoints';
            mask = poly2mask(double(invbboxPoints(1,:)), double(invbboxPoints(2,:)), cropSize(1,1), cropSize(1,2));
            
            R = videoFrame(:,:,1);
            G = videoFrame(:,:,2);
            B = videoFrame(:,:,3);
            R(~mask) = 0;
            G(~mask) = 0;
            B(~mask) = 0;
            im = cat(3,R,G,B);
%             im = imcrop(videoFrame, bbox);
%             scaleFactor = 150/size(im,1);
%             im = imresize(im, scaleFactor);
            imwrite(im, strcat(folderName,'/',int2str(i),'.png'));

            % Display a bounding box around the detected face.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);

            % Display detected corners.
            videoFrame = insertMarker(videoFrame, xyPoints, '+', 'Color', 'white');
        end

    else
        % Tracking mode.
        [xyPoints, isFound] = step(pointTracker, videoFrameGray);
        visiblePoints = xyPoints(isFound, :);
        oldInliers = oldPoints(isFound, :);

        numPts = size(visiblePoints, 1);

        if numPts >= 10
            % Estimate the geometric transformation between the old points
            % and the new points.
            [xform, oldInliers, visiblePoints] = estimateGeometricTransform(...
                oldInliers, visiblePoints, 'similarity', 'MaxDistance', 4);

            % Apply the transformation to the bounding box.
            bboxPoints = transformPointsForward(xform, bboxPoints);

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon = reshape(bboxPoints', 1, []);
            
            cropSize = size(videoFrame);
            invbboxPoints = bboxPoints';
            mask = poly2mask(double(invbboxPoints(1,:)), double(invbboxPoints(2,:)), cropSize(1,1), cropSize(1,2));
            
            R = videoFrame(:,:,1);
            G = videoFrame(:,:,2);
            B = videoFrame(:,:,3);
            R(~mask) = 0;
            G(~mask) = 0;
            B(~mask) = 0;
            im = cat(3,R,G,B);
            
            
%             im = imcrop(videoFrame, bbox);
%             scaleFactor = 150/size(im,1);
%             im = imresize(im, scaleFactor);
            imwrite(im, strcat(folderName,'/',int2str(i),'.png'));

            % Display a bounding box around the face being tracked.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);
           
            % Display tracked points.
            videoFrame = insertMarker(videoFrame, visiblePoints, '+', 'Color', 'white');

            % Reset the points.
            oldPoints = visiblePoints;
            setPoints(pointTracker, oldPoints);
        end

    end

    % Display the annotated video frame using the video player object.
    step(videoPlayer, videoFrame);

    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);
end

% Clean up.
release(videoPlayer);
release(pointTracker);
release(faceDetector);
end