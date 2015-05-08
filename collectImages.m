function collectImages(camera)
happy = 'happy';
surprised = 'surprised';

preview(camera)

% Happy
aviObject = VideoWriter('happy.avi');
aviObject.FrameRate = 60;
open(aviObject);
fprintf('Make a happy face, moving your face around the field, in:\n3\n');
pause(1)
fprintf('2\n');
pause(1)
fprintf('1\n');
pause(1)
fprintf('Go.\n');
for i = 1:100
    I = snapshot(camera);
    F = im2frame(I);
    writeVideo(aviObject, F);
end
close(aviObject);

% Surprised
aviObject = VideoWriter('surprised.avi');
aviObject.FrameRate = 60;
open(aviObject);
fprintf('Make a surprised face, moving your face around the field, in:\n3\n');
pause(1)
fprintf('2\n');
pause(1)
fprintf('1\n');
pause(1)
fprintf('Go.\n');
for i = 1:100
    I = snapshot(camera);
    F = im2frame(I);
    writeVideo(aviObject, F);
end
close(aviObject);

closePreview(camera);
end