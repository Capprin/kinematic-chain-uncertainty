% generates a surface of possible end positions when joint angles are uncertain
% should produce similar results to arm_gaussian, but faster

% arm information (2 links for simplified analysis)
link_vectors = {[1 0 0]' [1 0 0]'};
joint_axes = {'y', 'y'};
joint_angles = {-pi/2 pi/4};
% gaussian information
joint_angle_sds = {pi/24 pi/24};
num_samples = 100;

% generate cell array of nx1 vectors of normally-distributed angle
% deviations
% note: a wrapped normal may be better for this
rng('default'); % repeatable seed
angle_deviations = cell(size(joint_angles));
for j = 1:length(joint_angles)
    angle_deviations{j} = joint_angle_sds{j}.*randn(num_samples, 1);
end

% create matrix of all possible deviation vectors
% uses some matlab logic I don't _entirely_ understand, producing all
% possible combinations of two vectors
[A,B] = meshgrid(angle_deviations{1}, angle_deviations{2});
c=cat(2,A',B');
angle_dev_mat=reshape(c,[],2)';

% get mean arm position, arm endpoint jacobian
[J, link_ends, ~, ~, joint_axis_vectors_R] = link_jacobian(link_vectors, joint_angles, joint_axes, 2);

% use jacobian for endpoint to deviate end position
    % produces a linearly bound region
    % approximates well for small standard deviations
    % approximation could increase with system knowledge
ends = zeros(3, length(angle_dev_mat));
colors = ends';
for dev = 1:length(angle_dev_mat)
    ends(:,dev) = link_ends(:,end) + J * angle_dev_mat(:,dev);
    
    % get end colors
    within_one = true;
    within_two = true;
    for j = 1:length(joint_angles)
        if norm(angle_dev_mat(j, dev)) > joint_angle_sds{j}
            within_one = false;
            if norm(angle_dev_mat(j, dev)) > 2*joint_angle_sds{j}
                within_two = false;
            end
        end
    end
    if within_one
        colors(dev, :) = [66 170 245]/255;
    elseif within_two
        colors(dev, :) = [66 114 245]/255;
    else
        colors(dev, :) = [86 3 252]/255;
    end
end
    
% draw arm, joint axes, endpoints
ax = draw_arm_gaussian(2, ends, colors, link_ends, joint_axis_vectors_R);
view(ax, 0, 0);