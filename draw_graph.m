function draw_graph( xs, ys, A, Color, Width )

    %A-> Adjacency matrix
    %xs-> Points in x axis
    %ys-> Points in y axis
    
    hold on;

    % Draw the points
    n=length(xs);
    cm = colormap('Lines');
    for i=1:n
        x=xs(i);
        y=ys(i);
        rectangle('Position', [x-0.025, y-0.025, 0.05, 0.05], ...
            'Curvature', [1 1],...
            'FaceColor', cm(i,:), ...
            'EdgeColor', 'k', ...
            'LineWidth', 2);
    end
    % Plot the connections
    [v1,v2]=find(triu((A==A')&A));
    xstart=xs(v2); % In adjacency
    ystart=ys(v2);
    xend=xs(v1);
    yend=ys(v1);
    line([xstart';xend'], [ystart';yend'], 'Color', [Color(1),Color(2),Color(3)], 'LineWidth', Width);
    % Add arrows
    [v1,v2]=find((A~=A')&A);
    xstart=xs(v2); % In adjacency
    ystart=ys(v2);
    xend=xs(v1);
    yend=ys(v1);
    arrow(xstart, xend, ystart, yend);
    
    hold off;

end

