% generates a surface of possible end positions when joint angles are uncertain

% arm information (expect 3 links)
link_vectors = {[1 0 0]' [0.5 0 0]' [0.5 0 0]'};
joint_axes = {'y', 'y', 'y'};
joint_angles = {-pi/2 0 0};
% gaussian information
joint_angle_sds = {pi/12 pi/12 0};
num_samples = 25;


% generate cell array of nx1 vectors of normally-distributed joint angles
% note: a wrapped normal may be better for this
rng(7,'twister'); % repeatable seed
angles = cell(size(joint_angles));
for j = 1:length(joint_angles)
    angles{j} = joint_angle_sds{j}.*randn(num_samples, 1) + joint_angles{j};
end

% compute tip position for all combinations alpha, and color based on sd
% note: this is bad code, but produces output for now
ends = zeros(3, num_samples^3);
colors = ends';
idx = 1;
for i = 1:num_samples
    for j = 1:num_samples
        for k = 1:num_samples
            current_angles = {angles{1}(i) angles{2}(j) angles{3}(k)};
            current_endpoints = robot_arm_endpoints(link_vectors, current_angles, joint_axes);
            ends(:,idx) = current_endpoints(:,end);
            
            % get colors
            within_one = true;
            within_two = true;
            for jnt = 1:length(joint_angles)
                if current_angles{jnt} > joint_angles{jnt} + joint_angle_sds{jnt} || current_angles{jnt} < joint_angles{jnt} - joint_angle_sds{jnt}
                    within_one = false;
                    if current_angles{jnt} > joint_angles{jnt} + 2*joint_angle_sds{jnt} || current_angles{jnt} < joint_angles{jnt} - 2*joint_angle_sds{jnt}
                        within_two = false;
                    end
                end
            end
            if within_one
                colors(idx, :) = [66 170 245]/255;
            elseif within_two
                colors(idx, :) = [66 114 245]/255;
            else
                colors(idx, :) = [86 3 252]/255;
            end
            idx = idx + 1;
        end
    end
end

% compute arm geometry at mean joint angles
J = cell(1, 3);
for j = 1:3
    [J{j}, link_ends, ~, ~, joint_axis_vectors_R] = link_jacobian(link_vectors, joint_angles, joint_axes, j);
end

% draw endpoints, arm, arm axes
ax = draw_arm_gaussian(1, ends, colors, link_ends, joint_axis_vectors_R);
view(ax, 0, 0);
