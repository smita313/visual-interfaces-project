function collectImages(camera)
happy = 'happy';
surprised = 'surprised';

delete('happy/*.png');
delete('surprised/*.png');

liveFaceTrack('happy', camera);
liveFaceTrack('surprised', camera);
end