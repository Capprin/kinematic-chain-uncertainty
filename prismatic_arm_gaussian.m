% generates a cloud of possible end positions when joint angles are uncertain

% arm information
links = 3;
link_vectors = {[1 0 0]' [1 0 0]' [1 0 0]'};
joint_axes = {'y' 'y' 'y'};
joint_angles = {-pi/8 -pi/8 -pi/8};
% gaussian information
joint_angle_sds = {pi/12 pi/12 pi/12};
joint_length_sds = {1/12 1/12 1/12};
num_samples = 10000;

% create matrix to store angle, length deviations sampled from gaussian
rng(7,'twister'); % repeatable seed
deviation_mat = zeros(2*links, num_samples);
% store absolute deviations
for i = 1:links
    deviation_mat(i,:) = joint_angle_sds{i}.*randn(num_samples, 1) + joint_angles{i};
    deviation_mat(i+links,:) = joint_length_sds{i}.*randn(num_samples, 1) + norm(link_vectors{i});
end

% produce histograms of distributions
plot_prismatic_arm_distributions(2, deviation_mat);

% compute endpoints of sampled configurations
end_points = zeros(3, num_samples);
for i = 1:num_samples
    current_angles = cell(1,links);
    current_vectors = cell(1,links);
    for j = 1:links
        current_angles{j} = deviation_mat(j,i);
        current_vectors{j} = deviation_mat(j+links, i)*link_vectors{j};
    end
    current_end_points = robot_arm_endpoints(current_vectors, current_angles, joint_axes);
    end_points(:, i) = current_end_points(:,end);
end

% compute arm geometry at mean joint angles
[~, link_ends, ~, ~, joint_axis_vectors_R] = link_jacobian(link_vectors, joint_angles, joint_axes, j);

% draw endpoints, arm, arm axes
ax = draw_arm_gaussian(1, end_points, [66 114 245]/255, link_ends, joint_axis_vectors_R);
view(ax, 0, 0);
