fileID = fopen('../results/homography/uniform_000_m.csv');
t = textscan(fileID,'%s');
images = split (t{:}, ',');
images = images(:,1);

% read from 2 column
uniform000 = csvread('../results/homography/uniform_000_m.csv', 0, 1);
uniform100 = csvread('../results/homography/uniform_100_m.csv', 0, 1);
uniform010 = csvread('../results/homography/uniform_010_m.csv', 0, 1);
uniform001 = csvread('../results/homography/uniform_001_m.csv', 0, 1);
uniform111 = csvread('../results/homography/uniform_111_m.csv', 0, 1);

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
h.Title = 'Number of inliers';
h.XLabel = 'Algorithms';
h.YLabel = 'Images';
print (gcf, 'inliers.png', '-dpng', '-r300')

figure;
h = heatmap(algorithms,images,iters);
h.Title = 'Number of iterations';
h.XLabel = 'Algorithms';
h.YLabel = 'Images';
print (gcf, 'iters.png', '-dpng', '-r300')

figure;
h = heatmap(algorithms,images,time);
h.Title = 'Computational time';
h.XLabel = 'Algorithms';
h.YLabel = 'Images';
print (gcf, 'time.png', '-dpng', '-r300')

figure;
h = heatmap(algorithms,images,fails);
h.Title = 'Number of fails';
h.XLabel = 'Algorithms';
h.YLabel = 'Images';
print (gcf, 'fails.png', '-dpng', '-r300')

keyboard