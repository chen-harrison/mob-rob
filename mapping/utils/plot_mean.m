function plot_mean(obj, figure_title, filename)
    % plot ogm using conventional black-gray-white cells
    figure; hold on;
    axis([obj.range_x obj.range_y]);
    axis equal tight

    h = obj.grid_size/2;
    for i = 1:obj.map.size
        m = obj.map.occMap.X(i,:);
        x = m(1) - h;
        y = m(2) - h;
        rectangle('Position',[x,y,obj.grid_size,obj.grid_size],...
            'FaceColor',(1-obj.map.mean(i))*[1 1 1],'LineStyle','none');
    end
    
    set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
    set(groot, 'defaultLegendInterpreter','latex');
    set(gca, 'fontsize', 16)
    title(figure_title, 'FontSize', 16)
    print('-opengl','-dpng', '-r600', filename)
end