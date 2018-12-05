folder = '../results/homography/';
folder_res = '../results/homography/';

% folder = '../results/line2d/';
% folder_res = '../results/line2d/';


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

prosac000 = csvread([folder 'prosac_000_m.csv'], 0, 1);
prosac100 = csvread([folder 'prosac_100_m.csv'], 0, 1);
prosac010 = csvread([folder 'prosac_010_m.csv'], 0, 1);
prosac001 = csvread([folder 'prosac_001_m.csv'], 0, 1);
prosac111 = csvread([folder 'prosac_111_m.csv'], 0, 1);

algorithms = {'uniform', ...
            'uniform_lo', ...
            'uniform_gc', ...
            'uniform_sprt', ...
            'uniform_lo_gc_sprt', ...
            'prosac', ...
            'prosac_lo', ...
            'prosac_gc', ...
            'prosac_sprt', ...
            'prosac_lo_gc_sprt'};
        
inliers = [uniform000(:,1) uniform100(:,1) uniform010(:,1) uniform001(:,1) uniform111(:,1) ...
           prosac000(:,1) prosac100(:,1) prosac010(:,1) prosac001(:,1) prosac111(:,1)]; % ...
%            napsac000(:,1) napsac100(:,1) napsac010(:,1) napsac001(:,1) napsac111(:,1)];

iters =  [uniform000(:,3) uniform100(:,3) uniform010(:,3) uniform001(:,3) uniform111(:,3) ...
           prosac000(:,3) prosac100(:,3) prosac010(:,3) prosac001(:,3) prosac111(:,3)]; % ...
%            napsac000(:,3) napsac100(:,3) napsac010(:,3) napsac001(:,3) napsac111(:,3)];

time = [uniform000(:,5) uniform100(:,5) uniform010(:,5) uniform001(:,5) uniform111(:,5) ...
           prosac000(:,5) prosac100(:,5) prosac010(:,5) prosac001(:,5) prosac111(:,5)]; % ...
%            napsac000(:,5) napsac100(:,5) napsac010(:,5) napsac001(:,5) napsac111(:,5)];

fails = [uniform000(:,7) uniform100(:,7) uniform010(:,7) uniform001(:,7) uniform111(:,7) ...
           prosac000(:,7) prosac100(:,7) prosac010(:,7) prosac001(:,7) prosac111(:,7)]; % ...
%            napsac000(:,7) napsac100(:,7) napsac010(:,7) napsac001(:,7) napsac111(:,7)];
       
num_images = numel (images);
num_algs = numel (algorithms);
alg = {'1' '2' '3' '4' '5' '6' '7' '8' '9' 'A'};

 
save_results (inliers, images, algorithms, alg, 'Average number of inliers', [folder_res 'inliers'])
save_results (time, images, algorithms, alg, 'Average time (mcs)', [folder_res 'time'])
save_results (iters, images, algorithms, alg, 'Average number of iterations', [folder_res 'iters'])
save_results (fails, images, algorithms, alg, 'Number of fails', [folder_res 'fails'])



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
    h = heatmap(algorithms, 'i', zeros (size(data(1,:))));
    
    print (gcf, [save '.png'], '-dpng', '-r300');
%     print('-fillpage',save,'-dpdf', '-r300')
    close;
end
