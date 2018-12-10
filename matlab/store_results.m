% folder = '../results/homography/';
% folder_res = '../results/homography/';

% folder = '../results/line2d/';
% folder_res = '../results/line2d/';

folder = '../results/kusvod2/';
folder_res = '../results/kusvod2/';

folder = '../results/adelaidermf/';
folder_res = '../results/adelaidermf/';

fileID = fopen([folder 'uniform_000_m.csv']);
t = textscan(fileID,'%s');
images = split (t{:}, ',');
images = images(:,1);

% read from 2 column
uniform000 = csvread([folder 'uniform_000_m.csv'], 0, 1);
uniform100 = csvread([folder 'uniform_100_m.csv'], 0, 1);
uniform010 = csvread([folder 'uniform_010_m.csv'], 0, 1);
uniform001 = csvread([folder 'uniform_001_m.csv'], 0, 1);
uniform111 = csvread([folder 'uniform_111_m.csv'], 0, 1);

% prosac000 = csvread([folder 'prosac_000_m.csv'], 0, 1);
% prosac100 = csvread([folder 'prosac_100_m.csv'], 0, 1);
% prosac010 = csvread([folder 'prosac_010_m.csv'], 0, 1);
% prosac001 = csvread([folder 'prosac_001_m.csv'], 0, 1);
% prosac111 = csvread([folder 'prosac_111_m.csv'], 0, 1);

% napsac000 = csvread([folder 'napsac_000_m.csv'], 0, 1);
% napsac100 = csvread([folder 'napsac_100_m.csv'], 0, 1);
% napsac010 = csvread([folder 'napsac_010_m.csv'], 0, 1);
% napsac001 = csvread([folder 'napsac_001_m.csv'], 0, 1);
% napsac111 = csvread([folder 'napsac_111_m.csv'], 0, 1);


algorithms = {'uniform', ...
            'uniform_lo', ...
            'uniform_gc', ...
            'uniform_sprt', ...
            'uniform_lo_gc_sprt'} %, ...
%             'prosac', ...
%             'prosac_lo', ...
%             'prosac_gc', ...
%             'prosac_sprt', ...
%             'prosac_lo_gc_sprt'};
%             'napsac', ...
%             'napsac_lo', ...
%             'napsac_gc', ...
%             'napsac_sprt', ...
%             'napsac_lo_gc_sprt'};

inl = 1;
it = 2;
lo_it = 3;
tm = 4;
er = 5;
fl = 6;
        
inliers = [uniform000(:,inl) uniform100(:,inl) uniform010(:,inl) uniform001(:,inl) uniform111(:,inl)]; % ...
%            prosac000(:,inl) prosac100(:,inl) prosac010(:,inl) prosac001(:,inl) prosac111(:,inl)]; % ...
%            napsac000(:,1) napsac100(:,1) napsac010(:,1) napsac001(:,1) napsac111(:,1)];

iters =  [uniform000(:,it) uniform100(:,it) uniform010(:,it) uniform001(:,it) uniform111(:,it)];% ...
%            prosac000(:,it) prosac100(:,it) prosac010(:,it) prosac001(:,it) prosac111(:,it)]; % ...
%            napsac000(:,3) napsac100(:,3) napsac010(:,3) napsac001(:,3) napsac111(:,3)];

lo_iters =  [uniform000(:,lo_it) uniform100(:,lo_it) uniform010(:,lo_it) uniform001(:,lo_it) uniform111(:,lo_it)];% ...
%            prosac000(:,lo_it) prosac100(:,lo_it) prosac010(:,lo_it) prosac001(:,lo_it) prosac111(:,lo_it)]; % ...
%            napsac000(:,3) napsac100(:,3) napsac010(:,3) napsac001(:,3) napsac111(:,3)];

time = [uniform000(:,tm) uniform100(:,tm) uniform010(:,tm) uniform001(:,tm) uniform111(:,tm)];% ...
%            prosac000(:,tm) prosac100(:,tm) prosac010(:,tm) prosac001(:,tm) prosac111(:,tm)]; % ...
%            napsac000(:,5) napsac100(:,5) napsac010(:,5) napsac001(:,5) napsac111(:,5)];

fails = [uniform000(:,fl) uniform100(:,fl) uniform010(:,fl) uniform001(:,fl) uniform111(:,fl)];% ...
%            prosac000(:,fl) prosac100(:,fl) prosac010(:,fl) prosac001(:,fl) prosac111(:,fl)]; % ...
%            napsac000(:,7) napsac100(:,7) napsac010(:,7) napsac001(:,7) napsac111(:,7)];

errors = [uniform000(:,er) uniform100(:,er) uniform010(:,er) uniform001(:,er) uniform111(:,er)];% ...
%            prosac000(:,er) prosac100(:,er) prosac010(:,er) prosac001(:,er) prosac111(:,er)]; % ...
%            napsac000(:,er) napsac100(:,er) napsac010(:,er) napsac001(:,er) napsac111(:,er)];


num_images = numel (images);
num_algs = numel (algorithms);
alg = {'1' '2' '3' '4' '5'};% '6' '7' '8' '9' 'A'};

 
save_results (inliers, images, algorithms, alg, 'Average number of inliers', [folder_res 'inliers'])
save_results (time, images, algorithms, alg, 'Average time (mcs)', [folder_res 'time'])
save_results (iters, images, algorithms, alg, 'Average number of iterations', [folder_res 'iters'])
save_results (lo_iters, images, algorithms, alg, 'Average number of LO iterations', [folder_res 'lo_iters'])
save_results (fails, images, algorithms, alg, 'Number of fails', [folder_res 'fails'])
save_results (errors, images, algorithms, alg, 'Errors', [folder_res 'errors'])


% criteria = {'inliers', 'std inliers', 'iters', 'std iters', 'time', 'std time', 'fails'};
% for img = 1:numel (images)
%     for cr = 1:numel(criteria) 
%         subplot (numel(criteria), 1, cr);
%         data = [uniform000(img,cr) uniform100(img,cr) uniform010(img,cr) uniform001(img,cr) uniform111(img,cr)];
%         if (cr == numel(criteria))
%             h = heatmap(algorithms, criteria{cr}, data);
%         else
%             h = heatmap(alg, criteria{cr}, data);
%         end
%         if (cr == 1)
%             h.title(images{img});
%         end
%     end
%     print('-fillpage',[folder_res images{img}],'-dpdf', '-r300')
%     close
% end

function save_results (data, images, algorithms, alg, titl, save)
    figure('units','normalized','outerposition',[0 0 1 1])
    title ('asdad')

    subplot (numel (images)+3, 1, 1);
    h = heatmap(alg, 'title show', ones (size(data(1,:))));
    h.title(titl);

    for img = 1:numel (images)
        subplot (numel (images)+3, 1, img+1);
        h = heatmap(alg, images{img}, data(img,:));
    end
    
    subplot (numel (images)+3, 1, numel(images)+2)
    h = heatmap (alg, 'Average (all)', mean (data));
    
    subplot (numel (images)+3, 1, numel(images)+3)
    h = heatmap(algorithms, 'i', ones (size(data(1,:))));
    
    print (gcf, [save '.png'], '-dpng', '-r300');
%     print('-fillpage',save,'-dpdf', '-r300')
    close;
end
