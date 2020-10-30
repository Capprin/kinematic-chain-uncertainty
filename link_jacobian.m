function [J,...
          link_ends,...
          link_end_set,...   
          link_end_set_with_base,...
          joint_axis_vectors_R] = link_jacobian(link_vectors,...
                                               joint_angles,...
                                               joint_axes,...
                                               link_number)
    [link_ends, ~, R_links, ~, link_end_set, link_end_set_with_base] = robot_arm_endpoints(link_vectors,...
                                                                                           joint_angles,...
                                                                                           joint_axes);
    % generate vectors between joints and current link end
    vector_diff = cell(1, link_number);
    for j = 1:link_number
        vector_diff{j} = link_end_set{link_number} - link_end_set_with_base{j};
    end
    
    % generate joint axis vectors in-world
    joint_axis_vectors_R = cell(size(vector_diff));
    for j = 1:link_number
        switch joint_axes{j}
            case 'x'
                axis = [1 0 0]';
            case 'y'
                axis = [0 1 0]';
            case 'z'
                axis = [0 0 1]';
            otherwise
                error("Supplied joint axis is not x, y, or z");
        end
        joint_axis_vectors_R{j} = R_links{j} * axis;
    end
    
    % generate jacobian
    J = zeros(3, length(link_vectors));
    for j = 1:link_number
        J(:, j) = cross(joint_axis_vectors_R{j}, vector_diff{j});
    end
end