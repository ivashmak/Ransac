% folder = '../results/homography/';
% folder_res = '../results/homography/';

folder = '../results/EVD/';
folder_res = '../results/EVD/';

% folder = '../results/adelaidermf/prosac_nongt/';
% folder_res = folder;

% folder = '../results/line2d/';
% folder_res = '../results/line2d/';
% 
% folder = '../results/kusvod2/';
% folder_res = '../results/kusvod2/';

% folder = '../results/adelaidermf/';
% folder_res = '../results/adelaidermf/';

fileID = fopen([folder 'uniform_gc_Grid_c_sz_50_m.csv']);
t = textscan(fileID,'%s');
images = split (t{:}, ',');
images = images(:,1);

% read from 2 column
uniform010g50 = csvread([folder 'uniform_gc_Grid_c_sz_50_m.csv'], 0, 1);
uniform011g50 = csvread([folder 'uniform_gc_sprt_Grid_c_sz_50_m.csv'], 0, 1);
uniform010f = csvread([folder 'uniform_gc_Nanoflann_c_sz_50_m.csv'], 0, 1);
uniform011f = csvread([folder 'uniform_gc_sprt_Nanoflann_c_sz_50_m.csv'], 0, 1);

prosac010g50 = csvread([folder 'prosac_gc_Grid_c_sz_50_m.csv'], 0, 1);
prosac011g50 = csvread([folder 'prosac_gc_sprt_Grid_c_sz_50_m.csv'], 0, 1);
prosac010f = csvread([folder 'prosac_gc_Nanoflann_c_sz_50_m.csv'], 0, 1);
prosac011f = csvread([folder 'prosac_gc_sprt_Nanoflann_c_sz_50_m.csv'], 0, 1);


algorithms = {
            'uniform_gc_grid_50', ...
            'uniform_gc_sprt_grid50', ...
            'uniform_gc_nanoflann', ...
            'uniform_gc_sprt_nanoflann' ...
            'prosac_gc_grid_50', ...
            'prosac_gc_sprt_grid50', ...
            'prosac_gc_nanoflann', ...
            'prosac_gc_sprt_nanoflann'};
        
inl = 1;
it = 2;
lo_it = 3;
tm = 4;
er = 5;
fl = 6;

inliers = [uniform010g50(:,inl) uniform011g50(:,inl) uniform010f(:,inl) uniform011f(:,inl) ...
           prosac010g50(:,inl) prosac011g50(:,inl) prosac010f(:,inl) prosac011f(:,inl)];

iters = [uniform010g50(:,it) uniform011g50(:,it) uniform010f(:,it) uniform011f(:,it) ...
           prosac010g50(:,it) prosac011g50(:,it) prosac010f(:,it) prosac011f(:,it)];
       
lo_iters = [uniform010g50(:,lo_it) uniform011g50(:,lo_it) uniform010f(:,lo_it) uniform011f(:,lo_it) ...
           prosac010g50(:,lo_it) prosac011g50(:,lo_it) prosac010f(:,lo_it) prosac011f(:,lo_it)];

time = [uniform010g50(:,tm) uniform011g50(:,tm) uniform010f(:,tm) uniform011f(:,tm) ...
           prosac010g50(:,tm) prosac011g50(:,tm) prosac010f(:,tm) prosac011f(:,tm)];     
       
fails = [uniform010g50(:,fl) uniform011g50(:,fl) uniform010f(:,fl) uniform011f(:,fl) ...
           prosac010g50(:,fl) prosac011g50(:,fl) prosac010f(:,fl) prosac011f(:,fl)];
   
fails2 = [uniform010g50(:,fl+1) uniform011g50(:,fl+1) uniform010f(:,fl+1) uniform011f(:,fl+1) ...
           prosac010g50(:,fl+1) prosac011g50(:,fl+1) prosac010f(:,fl+1) prosac011f(:,fl+1)];
   
fails3 = [uniform010g50(:,fl+2) uniform011g50(:,fl+2) uniform010f(:,fl+2) uniform011f(:,fl+2) ...
           prosac010g50(:,fl+2) prosac011g50(:,fl+2) prosac010f(:,fl+2) prosac011f(:,fl+2)];
   
errors = [uniform010g50(:,er) uniform011g50(:,er) uniform010f(:,er) uniform011f(:,er) ...
           prosac010g50(:,er) prosac011g50(:,er) prosac010f(:,er) prosac011f(:,er)];
   

num_images = numel (images);
num_algs = numel (algorithms);
alg = {'1' '2' '3' '4' '5' '6' '7' '8'};

 
save_results (inliers, images, algorithms, alg, 'Average number of inliers', [folder_res 'inliers'])
save_results (time, images, algorithms, alg, 'Average time (mcs)', [folder_res 'time'])
save_results (iters, images, algorithms, alg, 'Average number of iterations', [folder_res 'iters'])
save_results (lo_iters, images, algorithms, alg, 'Average number of LO iterations', [folder_res 'lo_iters'])
save_results (fails, images, algorithms, alg, 'Number of fails < 10%', [folder_res 'fails1'])
save_results (fails2, images, algorithms, alg, 'Number of fails < 25%', [folder_res 'fails2'])
save_results (fails3, images, algorithms, alg, 'Number of fails < 50%', [folder_res 'fails3'])
save_results (errors, images, algorithms, alg, 'Average Error', [folder_res 'errors'])
