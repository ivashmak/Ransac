clear, clc, close all

% rng(0)
% data(:,1) = randn(30,1);
% data(:,2) = 3.4 + 1.2 * data(:,1);
% data(:,2) = data(:,2) + 0.2*randn(size(data(:,1)));
data = [208.13788, 409.93533;
 403.98703, 166.20012;
 492.69501, 11.031432];

data = sortrows(data,1);



% end of generating points

data(:,1) = data(:,1)-mean(data(:,1));
data(:,2) = data(:,2)-mean(data(:,2));

% end of translation to origin
data 

C = cov(data)


[V,~] = eig(C)

figure
axes('LineWidth',0.6,...
    'FontName','Helvetica',...
    'FontSize',8,...
    'XAxisLocation','Origin',...
    'YAxisLocation','Origin');
line(data(:,1),data(:,2),...
    'LineStyle','None',...
    'Marker','o');

% perpendicular line
% line([0 V(1,1)],[0 V(2,1)],...
%     'Color',[0.8 0.5 0.3],...
%     'LineWidth',0.75);

k = 500;
line([-k*V(1,2) k*V(1,2)],[-k*V(2,2) k*V(2,2)], 'Color', 'red');
axis equal



[coeff,newdatapca,latend,tsquared,variance] = pca(data);
coeff



