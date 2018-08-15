close all

% pts = [208.13788, 409.93533;
%  403.98703, 166.20012;
%  492.69501, 11.031432];

pts = [289.59152, 125.82558;
 564.44745, 123.5111;
 231.82735, 13.047687];

means = mean(pts)
B = pts - ones (3,1)*mean(pts);

C = cov (B);
[V,~] = eig (C)

figure
axes('LineWidth',0.6,...
    'FontName','Helvetica',...
    'FontSize',8,...
    'XAxisLocation','Origin',...
    'YAxisLocation','Origin');

line(pts(:,1), pts(:,2),...
    'LineStyle','None',...
    'Marker','.');

k = 500;
% cc = -V(1,2)*means(1) - V(2,2)*means(2)
cc = (means(1) +means(2))/2
a = V(1,2);
b = V(2,2);
line([-k*a+cc k*a+cc],[-k*b+cc k*b+cc], 'Color', 'red');

axis equal

