function plot_variance(obj, figure_title, filename)
    % plot ogm variance using colormap
    figure; hold on;
    axis([obj.range_x obj.range_y]);
    axis equal tight

    h = obj.grid_size/2;
    v_max = max(obj.map.variance);
    cmap = jet(256); % colormap with 256 colors
    for i = 1:obj.map.size
        m = obj.map.occMap.X(i,:);
        x = m(1) - h;
        y = m(2) - h;
        color = cmap(round((obj.map.variance(i)-0)*255/v_max)+1,:);
        rectangle('Position',[x,y,obj.grid_size,obj.grid_size],...
            'FaceColor',color,'LineStyle','none');
    end
    colormap jet
    colorbar 
    caxis([0, v_max])
    
    set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
    set(groot, 'defaultLegendInterpreter','latex');
    set(gca, 'fontsize', 16)
    title(figure_title, 'FontSize', 16)
    print('-opengl','-dpng', '-r600', filename)
end