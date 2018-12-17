function save_results (data, images, algorithms, alg, titl, save)
    figure('units','normalized','outerposition',[0 0 1 1])
    title ('asdad')

    subplot (numel (images)+3, 1, 1);
    h = heatmap(alg, 'title show', ones (size(data(1,:))));
    h.title(titl);

    for img = 1:numel (images)
        subplot (numel (images)+3, 1, img+1);
        h = heatmap(alg, images{img}, data(img,:));
    end
    
    subplot (numel (images)+3, 1, numel(images)+2)
    h = heatmap (alg, 'Average (all)', mean (data));
    
    subplot (numel (images)+3, 1, numel(images)+3)
    h = heatmap(algorithms, 'i', ones (size(data(1,:))));
    
    print (gcf, [save '.png'], '-dpng', '-r300');
%     print('-fillpage',save,'-dpdf', '-r300')
    close;
end