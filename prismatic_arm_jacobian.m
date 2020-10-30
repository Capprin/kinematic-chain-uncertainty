% generates a cloud of approximate end positions when joint angles are uncertain

% arm information
links = 1;
link_vectors = {[1 0 0]'};
joint_axes = {'y'};
joint_angles = {0};
% gaussian information
joint_angle_sds = {pi/12};
joint_length_sds = {1/10};
num_samples = 10000;

% create matrix to store angle, length deviations sampled from gaussian
rng(7,'twister'); % repeatable seed
deviation_mat = zeros(2*links, num_samples);
% store relative deviations
for i = 1:links
    deviation_mat(i,:) = joint_angle_sds{i}.*randn(num_samples, 1);
    deviation_mat(i+links,:) = joint_length_sds{i}.*randn(num_samples, 1);
end

% produce histograms of distributions
plot_prismatic_arm_distributions(4, deviation_mat);

% get mean arm position, arm endpoint jacobian
[J, link_ends, ~, ~, joint_axis_vectors_R] = link_jacobian(link_vectors, joint_angles, joint_axes, links);

% augment jacobian with unit link vectors
J_aug = [J, zeros(size(J))];
for j = 1:links
    J_aug(:,links+j) = link_vectors{j}/norm(link_vectors{j});
end

% use jacobian to deviate end position
end_points = zeros(3, num_samples);
for i = 1:num_samples
    end_points(:,i) = link_ends(:,end) + J_aug * deviation_mat(:,i);
end

% draw endpoints, arm, arm axes
ax = draw_arm_gaussian(3, end_points, [66 114 245]/255, link_ends, joint_axis_vectors_R);
view(ax, 0, 0);