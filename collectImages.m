function collectImages(camera)

delete('training/happy/*.png');
delete('training/surprised/*.png');

liveFaceTrack('happy', camera);
liveFaceTrack('surprised', camera);
end