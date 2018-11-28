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
iters = [uniform000(:,3) uniform100(:,3) uniform010(:,3) uniform001(:,3) uniform111(:,3)];
time = [uniform000(:,5) uniform100(:,5) uniform010(:,5) uniform001(:,5) uniform111(:,5)];
fails = [uniform000(:,7) uniform100(:,7) uniform010(:,7) uniform001(:,7) uniform111(:,7)];

h = heatmap(algorithms,images,inliers);
h.Title = 'Average Number of inliers';
h.XLabel = 'Algorithms';
h.YLabel = 'Images';
print (gcf, [folder_res 'inliers.png'], '-dpng', '-r300')
close

figure;
h = heatmap(algorithms,images,iters);
h.Title = 'Average number of iterations';
h.XLabel = 'Algorithms';
h.YLabel = 'Images';
print (gcf, [folder_res 'iters.png'], '-dpng', '-r300')
close

figure;
h = heatmap(algorithms,images,time);
h.Title = 'Average Computational time (mcs)';
h.XLabel = 'Algorithms';
h.YLabel = 'Images';
print (gcf, [folder_res 'time.png'], '-dpng', '-r300')
close

figure;
h = heatmap(algorithms,images,fails);
h.Title = 'Number of fails';
h.XLabel = 'Algorithms';
h.YLabel = 'Images';
print (gcf, [folder_res 'fails.png'], '-dpng', '-r300')
close
