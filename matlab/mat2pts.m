adelaidermf ();


function kusvod2 ()
    dataset = '../dataset/Lebeda/kusvod2/';
    files = dir([dataset '*.mat']);

    for i = 1:numel(files)
        load ([dataset files(i).name]);
        pts = validation.pts';
        model = validation.model;

        [~,name,~] = fileparts (files(i).name);
        save ([dataset name '_pts.txt'], 'pts', '-ascii');
        save ([dataset name '_model.txt'], 'model', '-ascii');
    end
end

function adelaidermf ()
    dataset = '../dataset/adelaidermf/';
    files = dir([dataset '*.mat']);

    for i = 1:numel(files)
        load ([dataset files(i).name]);
        pts = [data; label]';
        [~,name,~] = fileparts (files(i).name);
        imwrite (img1, [dataset name 'A.png']);
        imwrite (img2, [dataset name 'B.png']);
        save ([dataset name '_pts.txt'], 'pts', '-ascii');
    end
end
