function plot_semantic(obj, figure_title, filename)
    % plot ogm
    color_semantic = [[0 0.4470 0.7410]; [0.8500 0.3250 0.0980]; [0.9290 0.6940 0.1250];
                  [0.4940 0.1840 0.5560]; [0.4660 0.6740 0.1880]; [0.3010 0.7450 0.9330]; [0.6350 0.0780 0.1840]];
    figure; hold on;
    axis([obj.range_x obj.range_y]);
    axis equal tight

    h = obj.grid_size/2;
    unknown = obj.map.mean(1,:);
    for i = 1:obj.map.size
        m = obj.map.occMap.X(i,:);
        x = m(1) - h;
        y = m(2) - h;
        if isequal(obj.map.mean(i,:), unknown)
            rectangle('Position',[x,y,obj.grid_size,obj.grid_size],...
                'FaceColor',0.5*[1 1 1],'LineStyle','none');
        else
            [~,k] = max(obj.map.mean(i,:));
            if k == obj.num_classes+1 % free space
                rectangle('Position',[x,y,obj.grid_size,obj.grid_size],...
                    'FaceColor',[1 1 1],'LineStyle','none');
            else
                rectangle('Position',[x,y,obj.grid_size,obj.grid_size],...
                    'FaceColor',color_semantic(k, :),'LineStyle','none');
            end
        end
    end
    set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
    set(groot, 'defaultLegendInterpreter','latex');
    set(gca, 'fontsize', 16)
    title(figure_title, 'FontSize', 16)
    print('-opengl','-dpng', '-r600', filename)
end