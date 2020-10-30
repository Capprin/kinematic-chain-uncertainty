function f = plot_prismatic_arm_distributions(fignum, deviation_matrix)
    f = figure(fignum);
    clf(f, 'reset');
    
    links = size(deviation_matrix, 1)/2;
    for i = 1:links
        subplot(2, links, i);
        histogram(deviation_matrix(i, :));
        title("Angle frequency, Joint " + num2str(i));
        xlabel("Angle");
        
        subplot(2, links, i+links);
        histogram(deviation_matrix(i, :));
        title("Length frequency, Link " + num2str(i));
        xlabel("Length");
    end
end