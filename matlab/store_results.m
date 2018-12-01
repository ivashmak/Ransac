folder = '../results/homography/';
folder_res = '../results/homography/';

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

algorithms = {'uniform', ...
            'uniform_lo', ...
            'uniform_gc', ...
            'uniform_sprt', ...
            'uniform_lo_gc_sprt'};
        
inliers = [uniform000(:,1) uniform100(:,1) uniform010(:,1) uniform001(:,1) uniform111(:,1)];
std_dev_inliers = [uniform000(:,2) uniform100(:,2) uniform010(:,2) uniform001(:,2) uniform111(:,2)];
iters = [uniform000(:,3) uniform100(:,3) uniform010(:,3) uniform001(:,3) uniform111(:,3)];
std_dev_iters = [uniform000(:,4) uniform100(:,4) uniform010(:,4) uniform001(:,4) uniform111(:,4)];
time = [uniform000(:,5) uniform100(:,5) uniform010(:,5) uniform001(:,5) uniform111(:,5)];
std_dev_time = [uniform000(:,6) uniform100(:,6) uniform010(:,6) uniform001(:,6) uniform111(:,6)];
fails = [uniform000(:,7) uniform100(:,7) uniform010(:,7) uniform001(:,7) uniform111(:,7)];
         
num_images = numel (images);
num_algs = numel (algorithms);
n = '.';
alg = {};
for a = 1:num_algs
    n = [n '.'];
    alg{a} = n;
end
 
save_results (inliers, images, algorithms, alg, 'Average number of inliers', [folder_res 'inliers'])
save_results (time, images, algorithms, alg, 'Average time (mcs)', [folder_res 'time'])
save_results (iters, images, algorithms, alg, 'Average number of iterations', [folder_res 'iters'])
save_results (fails, images, algorithms, alg, 'Number of fails', [folder_res 'fails'])

criteria = {'inliers', 'std inliers', 'iters', 'std iters', 'time', 'std time', 'fails'};

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
    for img = 1:numel (images)
        subplot (numel (images), 1, img);
        if (img == numel(images))
            h = heatmap(algorithms, images{img}, data(img,:));
        else
            h = heatmap(alg, images{img}, data(img,:));
        end
        if (img == 1)
            h.title(titl);
        end
    end
    xlabel('Algorithms');
    print (gcf, [save '.png'], '-dpng', '-r300');
    
%     print('-fillpage',save,'-dpdf', '-r300')
    close;
end
