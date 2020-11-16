% draws relevant information for gaussian-perturbed arm
function [ax, f] = draw_arm_gaussian(fignum,...
                                          endpoints,...
                                          end_vectors,...
                                          draw_vectors,...
                                          endpoint_colors,...
                                          link_ends,...
                                          joint_axis_vectors_R)
    % figure setup
    f = figure(fignum);
    clf(f, 'reset');
    ax = axes(f);
    axis(ax, 'equal');
    box(ax, 'on');
    xlabel(ax, 'x');
    ylabel(ax, 'y');
    zlabel(ax, 'z');
    view(ax, 3);

    hold on;

    % draw tip positions
    if draw_vectors
        quiver3(endpoints(1,:), endpoints(2,:), endpoints(3,:), end_vectors(1,:), end_vectors(2,:), end_vectors(3,:), 'showarrowhead', 'on', 'color', endpoint_colors, 'parent', ax);
    end
    ends = scatter3(endpoints(1,:), endpoints(2,:), endpoints(3,:), 20, endpoint_colors, 'marker', '.', 'parent', ax);
        
    % draw arm at mean joint angles
    arm_line = line('parent', ax,...
             'xdata', link_ends(1,:),...
             'ydata', link_ends(2,:),...
             'zdata', link_ends(3,:),...
             'marker', 'o',...
             'linestyle', '-');

    % draw joint axes
    num_links = size(link_ends, 2)-1;
    axes_lines = cell(1, num_links);
    colors = {'b' 'r' 'g'};
    for j = 1:num_links
        vec = [link_ends(:,j), 0.5*joint_axis_vectors_R{j} + link_ends(:,j)];
        axes_lines{j} = line('parent', ax,...
                     'xdata', vec(1,:),...
                     'ydata', vec(2,:),...
                     'zdata', vec(3,:),...
                     'linestyle', '--',...
                     'color', colors{j});                 
    end
    hold off;
end