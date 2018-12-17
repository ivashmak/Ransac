folder = '../results/homography/';
folder_res = '../results/homography/grid_size/';

fileID = fopen([folder 'uniform_010_Grid_c_sz_25_m.csv']);
t = textscan(fileID,'%s');
images = split (t{:}, ',');
images = images(:,1);

% read from 2 column
uniform010g25 = csvread([folder 'uniform_010_Grid_c_sz_25_m.csv'], 0, 1);
uniform010g50 = csvread([folder 'uniform_010_Grid_c_sz_50_m.csv'], 0, 1);
uniform010g100 = csvread([folder 'uniform_010_Grid_c_sz_100_m.csv'], 0, 1);

algorithms = {
            'uniform_gc_grid_cell_size_25', ...
            'uniform_gc_grid_cell_size_50', ...
            'uniform_gc_grid_cell_size_100'};

inl = 1;
it = 2;
lo_it = 3;
tm = 4;
er = 5;
fl = 6;
        
inliers = [uniform010g25(:,inl) uniform010g50(:,inl) uniform010g100(:,inl) ];

iters = [uniform010g25(:,it) uniform010g50(:,it) uniform010g100(:,it) ];
       
lo_iters = [uniform010g25(:,lo_it) uniform010g50(:,lo_it) uniform010g100(:,lo_it) ];

time = [uniform010g25(:,tm) uniform010g50(:,tm) uniform010g100(:,tm) ];

fails = [uniform010g25(:,fl) uniform010g50(:,fl) uniform010g100(:,fl) ];

fails2 = [uniform010g25(:,fl+1) uniform010g50(:,fl+1) uniform010g100(:,fl+1) ];

fails3 = [uniform010g25(:,fl+2) uniform010g50(:,fl+2) uniform010g100(:,fl+2) ];

errors = [uniform010g25(:,er) uniform010g50(:,er) uniform010g100(:,er) ];


num_images = numel (images);
num_algs = numel (algorithms);
alg = {'1' '2' '3'};

 
save_results (inliers, images, algorithms, alg, 'Average number of inliers', [folder_res 'inliers'])
save_results (time, images, algorithms, alg, 'Average time (mcs)', [folder_res 'time'])
save_results (iters, images, algorithms, alg, 'Average number of iterations', [folder_res 'iters'])
save_results (lo_iters, images, algorithms, alg, 'Average number of LO iterations', [folder_res 'lo_iters'])
save_results (fails, images, algorithms, alg, 'Number of fails < 10%', [folder_res 'fails1'])
save_results (fails2, images, algorithms, alg, 'Number of fails < 25%', [folder_res 'fails2'])
save_results (fails3, images, algorithms, alg, 'Number of fails < 50%', [folder_res 'fails3'])
save_results (errors, images, algorithms, alg, 'Average Error', [folder_res 'errors'])

