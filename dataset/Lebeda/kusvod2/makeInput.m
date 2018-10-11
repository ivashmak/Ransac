% make input structure from files
names = {'booksh', 'box', 'castle', 'corr', 'graff', 'head', 'kampa', 'Kyoto', 'leafs', ...
	'plant', 'rotunda', 'shout', 'valbonne', 'wall', 'wash', 'zoom'};
exts = {'png', 'png', 'png', 'png', 'png', 'jpg', 'png', 'jpg', 'jpg', 'png',...
	'png', 'png', 'png', 'jpg', 'png', 'png'};
path = [pwd '/'];

for i = 13:16
	names{i}
	set(GUI.main.h.limage, 'String', [names{i} 'A.' exts{i}]);
	delete(findobj(allchild(GUI.figh1),'flat','serializable','on'));
	load_img(1, [path names{i}, 'A.', exts{i}]); zoom(GUI.figh1,'out');

	set(GUI.main.h.rimage, 'String', [names{i}, 'B.', exts{i}]);
	delete(findobj(allchild(GUI.figh2),'flat','serializable','on'));
	load_img(2, [path names{i}, 'B.', exts{i}]); zoom(GUI.figh2,'out');

	RES = [];
	do_all('eg');
	
	load([names{i} '_vpts']);
	
	input(i).name = names{i};
	input(i).ext = exts{i};
	input(i).u = TC.u(1:6,:);
	input(i).th = RES.model.params.th;
	input(i).validation = validation;
	
end

save inputHA input
