% generates a surface of possible end positions when joint angles are uncertain

% arm information
links = 1;
link_vectors = {[1 0 0]'};
joint_axes = {'y'};
joint_angles = {0};
% gaussian information
joint_angle_sds = {pi/12};
joint_length_sds = {1/10};
num_samples = 150;

% create matrix to store angle, length deviations sampled from gaussian
rng(7,'twister'); % repeatable seed
deviation_mat = zeros(2*links, num_samples);
% store deviations
for i = 1:links
    deviation_mat(i,:) = joint_angle_sds{i}.*randn(num_samples, 1) + joint_angles{i};
    deviation_mat(i+links,:) = joint_length_sds{i}.*randn(num_samples, 1) + norm(link_vectors{i});
end


% compute possible combinations of deviations
% note: haven't found better way to automate this with different joint
% lengths
[a1, l1] = ndgrid(1:num_samples);
cart_prod = [deviation_mat(1,a1); deviation_mat(2,l1)];

% calculate all endpoints under cartesian product
end_points = zeros(3, length(cart_prod));
end_point_colors = end_points';
for i = 1:length(cart_prod)
    current_angles = cell(1,links);
    current_lengths = cell(1,links);
    for j = 1:links
        current_angles{j} = cart_prod(j,i);
        current_lengths{j} = cart_prod(j+links, i)*link_vectors{j};
    end
    current_end_points = robot_arm_endpoints(current_lengths, current_angles, joint_axes);
    end_points(:, i) = current_end_points(:,end);
    
    % compute colors
    % note: not done in a great way, but produces results
    within_one = true;
    within_two = true;
    for j = 1:links
        if norm(cart_prod(j,i)-joint_angles{j}) > joint_angle_sds{j} || norm(cart_prod(j+links,i)-norm(link_vectors{j})) > joint_length_sds{j}
            within_one = false;
            if norm(cart_prod(j,i)-joint_angles{j}) > 2*joint_angle_sds{j} || norm(cart_prod(j+links,i)-norm(link_vectors{j})) > 2*joint_length_sds{j}
                within_two = false;
            end
        end
    end
    if within_one
        end_point_colors(i,:) = [66 170 245]/255;
    elseif within_two
        end_point_colors(i,:) = [66 114 245]/255;
    else
        end_point_colors(i,:) = [86 3 252]/255;
    end
end

% compute arm geometry at mean joint angles
[~, link_ends, ~, ~, joint_axis_vectors_R] = link_jacobian(link_vectors, joint_angles, joint_axes, j);

% draw endpoints, arm, arm axes
ax = draw_arm_gaussian(1, end_points, end_point_colors, link_ends, joint_axis_vectors_R);
view(ax, 0, 0);
