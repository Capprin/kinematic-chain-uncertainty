function [link_ends,...
          R_joints,...
          R_links,...
          link_vectors_in_world,...
          link_end_set,...
          link_end_set_with_base] = robot_arm_endpoints(link_vectors,...
                                                        joint_angles,...
                                                        joint_axes)
    % generate rotation matrices
    R_joints = cell(size(joint_angles));
    for j = 1:length(joint_angles)
        alpha = joint_angles{j};
        switch joint_axes{j}
            case 'x'
                R_joints{j} = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];
            case 'y'
                R_joints{j} = [cos(alpha) 0 sin(alpha); 0 1 0; -sin(alpha) 0 cos(alpha)];
            case 'z'
                R_joints{j} = [cos(alpha) -sin(alpha) 0; sin(alpha) cos(alpha) 0; 0 0 1];
            otherwise
                error("Supplied joint axis is not x, y, or z");
        end
    end
    
    % generate cumulative link frame rotations
    R_links = R_joints;
    for j = 2:length(R_links)
        R_links{j} = R_links{j-1} * R_joints{j};
    end
    
    % generate rotated links in respective frames
    link_vectors_in_world = cell(size(link_vectors));
    for j = 1:length(link_vectors_in_world)
        link_vectors_in_world{j} = R_links{j} * link_vectors{j};
    end
    
    % add links end to end
    link_end_set = link_vectors_in_world;
    for j = 2:length(link_end_set)
        link_end_set{j} = link_end_set{j-1} + link_vectors_in_world{j};
    end
    
    % output
    link_end_set_with_base = [{[0 0 0]'} link_end_set];
    link_ends = cell2mat(link_end_set_with_base);
end