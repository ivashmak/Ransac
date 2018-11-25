close all
load ('points.mat');

iter = 0;
max_iters = 10000;

current_score = 0;
best_score = 0;

best_model = zeros (1,3);
points_size = size (points, 1);
sample_size = 2;
threshold = 10;

weights = ones (1, points_size) / points_size;
indices = 1:points_size;

x = min(min(points)):max(max(points));
    
while iter < max_iters
    for s = 1:1000
        sample = unique (randsample(indices, sample_size, true, weights));
        if size (sample, 2) == sample_size
            pt1 = points(sample(1), :);
            pt2 = points(sample(2), :);
            break
        end
    end
    if s == 1000
        disp ('bad samping');
        return
    end

    a = pt1 (2) - pt2 (2);
    b = pt2 (1) - pt1 (1);
    mag = sqrt (a^2 + b^2);
    a = a/mag; b = b/mag;
    c = (pt1 (1) * pt2 (2) - pt2 (1) * pt1 (2)) / mag;
    
    inliers = abs (points * [a;b] + c) < threshold;
    current_score = sum (inliers);
    
%     weights(~inliers) = 2 * weights (~inliers);
%     weights(inliers)  =     weights (inliers) / 2;
    
    weights(inliers)  = weights (inliers) / 4;
    add = (1 - sum (weights(inliers))) / size (weights (~inliers), 2);
    weights(~inliers) = weights (~inliers) + add;
    
%     figure;
%     xlim ([0 max(points (:,1))+1]); ylim ([0 max(points (:,2))+1]);
%     disp ('press key')
%     hold on;
%     y = (-a*x - c)/b;
%     plot (points(:,1), points(:,2), '.'); plot (x, y, 'LineWidth', 2); 
%     inls = points (inliers, :);
%     plot (inls(:,1), inls(:,2), 'o'); 
%     plot ([pt1(1); pt2(1)], [pt1(2); pt2(2)], '+');
%     hold off
%     w = waitforbuttonpress;
%     close;
    
%     figure;
%     xlim ([0 max(points (:,1))+1]); ylim ([0 max(points (:,2))+1]);
%     hold on;
%     y = (-a*x - c)/b;
%     plot (x, y, 'LineWidth', 2); 
%     arrayfun (@(i) plot (points (i,1), points(i,2), ...
%                     '.', 'Color', 'blue', 'MarkerSize', weights(i)*10000), indices);
%     hold off;
%     w = waitforbuttonpress;
%     saveas (gcf, [char(string(iter)) '.png']);
%     close;
    
    if current_score > best_score
        best_score = current_score;
        best_model = [a b c];
        
        max_iters = log (0.01) / log (1 - (current_score/points_size)^sample_size);
    end
    iter = iter + 1;
end
disp('iterations reached');
disp (iter)

a = best_model(1);
b = best_model(2);
c = best_model(3);

inliers = points (abs (points * [a;b] + c) < threshold, :);
[V,~] = eig (cov (inliers));
a = V(1,1);
b = V(2,1);
c = -mean (inliers) * V(:, 1);

inliers = abs (points * [a;b] + c) < threshold;
disp ('number of inliers')
sum (inliers)
inliers = points (inliers, :);

figure;
hold on;
y = (-a*x - c)/b;
plot (points(:,1), points(:,2), '.');
plot (x, y, 'LineWidth', 2);
hold off

