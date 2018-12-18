% adelaidermf ();
kusvod2 ();

function kusvod2 ()
    dataset = '../dataset/Lebeda/kusvod2/';
    files = dir([dataset '*.mat']);

    for i = 1:numel(files)
        load ([dataset files(i).name]);
        pts = validation.pts';
        model = validation.model;

        [~,name,~] = fileparts (files(i).name);
%         save ([dataset name '_pts.txt'], 'pts', '-ascii');
%         save ([dataset name '_model.txt'], 'model', '-ascii');        
        fid1 = fopen([dataset name '_pts.txt'], 'w');
        fid2 = fopen([dataset name '_model.txt'], 'w');
        
        fprintf(fid1, '%f %f %d %f %f %d\n', pts');
        fprintf(fid2, '%f %f %f\n%f %f %f\n%f %f %f\n', model');
        fclose(fid1);
        fclose(fid2);
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
%         save ([dataset name '_pts.txt'], 'pts', '-ascii');
        fid = fopen([dataset name '_pts.txt'], 'w');
        fprintf(fid,'%f %f %d %f %f %d %d\n', pts');
        fclose(fid);
    end
end
