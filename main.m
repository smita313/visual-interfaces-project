function main
collect = 1;
if (~length(dir('happy')) > 2 &&  ~length(dir('surprised')) > 2)
    c = input('Do you want to collect new database photos? y/n\n','s');
    if (~isempty(strfind(c,'n')) || ~isempty(strfind(c,'N')))
        collect = 0;
    end
end

camera = webcam;

if collect
    collectImages(camera);
end

keepGoing = 1;
answers = [];
while keepGoing
    result = testImage(webcam);
    correct = input('Is this correct? y/n\n','s');
    if (~isempty(strfind(correct,'y')) || ~isempty(strfind(correct,'Y')))
        corr = 1;
    elseif (~isempty(strfind(correct,'n')) || ~isempty(strfind(correct,'N')))
        corr = 0;
    end
    answers = [answers, corr];
    go = input('Do you want to keep going? y/n\n', 's');
    keepGoing = 0;
    if (~isempty(strfind(go,'y')) || ~isempty(strfind(go,'Y')))
        keepGoing = 1;
    end
end
a = unique(answers);
output = histc(answers(:),a);
fprintf('The number of correct responses is %d\n', output(2));
fprintf('The number of incorrect responses is %d\n', output(1));
clear('camera');
end