folder = '../results/adelaidermf/gt_nonprosac/';
folder_res = folder;


fileID = fopen([folder 'uniform_gc_Grid_c_sz_50_m.csv']);
t = textscan(fileID,'%s');
images = split (t{:}, ',');
images = images(:,1);

% read from 2 column
uniform010g50 = csvread([folder 'uniform_gc_Grid_c_sz_50_m.csv'], 0, 1);
uniform011g50 = csvread([folder 'uniform_gc_sprt_Grid_c_sz_50_m.csv'], 0, 1);
uniform010f = csvread([folder 'uniform_gc_Nanoflann_c_sz_50_m.csv'], 0, 1);
uniform011f = csvread([folder 'uniform_gc_sprt_Nanoflann_c_sz_50_m.csv'], 0, 1);

algorithms = {
            'uniform_gc_grid_50', ...
            'uniform_gc_sprt_grid50', ...
            'uniform_gc_nanoflann', ...
            'uniform_gc_sprt_nanoflann'};
        
inl = 1;
it = 2;
lo_it = 3;
tm = 4;
er = 5;
fl = 6;

inliers = [uniform010g50(:,inl) uniform011g50(:,inl) uniform010f(:,inl) uniform011f(:,inl)];

iters = [uniform010g50(:,it) uniform011g50(:,it) uniform010f(:,it) uniform011f(:,it)];
       
lo_iters = [uniform010g50(:,lo_it) uniform011g50(:,lo_it) uniform010f(:,lo_it) uniform011f(:,lo_it)];

time = [uniform010g50(:,tm) uniform011g50(:,tm) uniform010f(:,tm) uniform011f(:,tm)];

fails = [uniform010g50(:,fl) uniform011g50(:,fl) uniform010f(:,fl) uniform011f(:,fl)];

fails2 = [uniform010g50(:,fl+1) uniform011g50(:,fl+1) uniform010f(:,fl+1) uniform011f(:,fl+1)];

fails3 = [uniform010g50(:,fl+2) uniform011g50(:,fl+2) uniform010f(:,fl+2) uniform011f(:,fl+2)];

errors = [uniform010g50(:,er) uniform011g50(:,er) uniform010f(:,er) uniform011f(:,er)];


num_images = numel (images);
num_algs = numel (algorithms);
alg = {'1' '2' '3' '4'};

 
save_results (inliers, images, algorithms, alg, 'Average number of inliers', [folder_res 'inliers'])
save_results (time, images, algorithms, alg, 'Average time (mcs)', [folder_res 'time'])
save_results (iters, images, algorithms, alg, 'Average number of iterations', [folder_res 'iters'])
save_results (lo_iters, images, algorithms, alg, 'Average number of LO iterations', [folder_res 'lo_iters'])
save_results (fails, images, algorithms, alg, 'Number of fails < 10%', [folder_res 'fails1'])
save_results (fails2, images, algorithms, alg, 'Number of fails < 25%', [folder_res 'fails2'])
save_results (fails3, images, algorithms, alg, 'Number of fails < 50%', [folder_res 'fails3'])
save_results (errors, images, algorithms, alg, 'Average Error', [folder_res 'errors'])
