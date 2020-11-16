classdef GaussianArm
    %GaussianArm operates on system and configuration for a kinematic chain
    
    properties
        % system and configuration
        num_links;
        link_vectors;
        link_vector_sds;
        joint_angles;
        joint_angle_sds;
        joint_axes;
        J;
        J_aug;
        
        % statistics data
        mu_vec;
        cov_mat;
        num_samples;
        samples;
        sample_densities;
        
        % output data
        end_points;
        end_point_colors;
    end
    
    methods
        function obj = GaussianArm(link_vectors,...
                                   link_vector_sds,...
                                   joint_angles,...
                                   joint_angle_sds,...
                                   joint_axes)
            % initializes system, configuration for chain
            obj.link_vectors = link_vectors;
            obj.link_vector_sds = link_vector_sds;
            obj.joint_angles = joint_angles;
            obj.joint_angle_sds = joint_angle_sds;
            obj.joint_axes = joint_axes;
            obj.num_links = length(link_vectors);
            
            % init known mu
            obj.mu_vec = [cell2mat(obj.joint_angles) cellfun(@(v) norm(v), obj.link_vectors)];
            
            % init colors
            obj.end_point_colors = [1 0 0]';
        end
        
        function obj = sample_configuration(obj, num_samples)
            % produces multivariate normal distribution in configuration
            
            rng(7,'twister'); % repeatable seed
            % sample variances to get good idea of covariance (lightly
            % unnecesssary)
            var = zeros(2*obj.num_links, num_samples);
            for j = 1:obj.num_links                
                var(j,:) = obj.joint_angle_sds{j} .* randn(num_samples, 1);
                var(j+obj.num_links,:) = obj.link_vector_sds{j} .* randn(num_samples, 1);
            end
            
            % compute covariance matrix
            obj.cov_mat = zeros(2*obj.num_links);
            for s = 1:num_samples
                obj.cov_mat = obj.cov_mat + var(:,s) * var(:,s)';
            end
            obj.cov_mat = obj.cov_mat/num_samples;
            
            % sample multivariate normal
            obj.num_samples = num_samples;
            obj.samples = mvnrnd(obj.mu_vec, obj.cov_mat, num_samples)';
            obj.sample_densities = mvnpdf(obj.samples', obj.mu_vec, obj.cov_mat);
        end
        
        function obj = true_end_points(obj)
            % produces true endpoints of end effector
            
            % setup
            mustBeNonempty(obj.samples);
            obj.end_points = zeros(3, obj.num_samples);
            
            for s = 1:obj.num_samples
                % generate cell array for robot_arm_endpoints
                current_angles = cell(1, obj.num_links);
                current_link_vectors = current_angles;
                for j = 1:obj.num_links
                    current_angles{j} = obj.samples(j, s);
                    current_link_vectors{j} = obj.samples(j+obj.num_links, s) * obj.link_vectors{j};
                end
                
                % generate endpoint
                current_link_ends = robot_arm_endpoints(current_link_vectors, current_angles, obj.joint_axes);
                obj.end_points(:, s) = current_link_ends(:, end);
            end
        end
        
        function obj = jacobian_end_points(obj)
            % produces endpoint approximation using jacobian
            
            % setup
            mustBeNonempty(obj.samples);
            obj.end_points = zeros(3, obj.num_samples);
            
            % compute augmented Jacobian (incl. prismatic links)
            [obj.J, link_ends, link_end_set] = link_jacobian(obj.link_vectors,...
                                                 obj.joint_angles,...
                                                 obj.joint_axes,...
                                                 obj.num_links);
            obj.J_aug = [obj.J zeros(size(obj.J))];
            for j = 1:obj.num_links
                obj.J_aug(:, obj.num_links+j) = link_end_set{j}/norm(obj.link_vectors{j});
            end
            
            % deviate end effector using jacobian
            for s = 1:obj.num_samples
                conf_deviation = obj.samples(:, s) - obj.mu_vec';
                eucl_deviation = obj.J_aug * conf_deviation;
                obj.end_points(:, s) = link_ends(:, end) + eucl_deviation;
            end
        end
        
        function obj = color_ends_gradient(obj, high_color, low_color)
            % colors endpoints in a gradient of their probability densities
            
            % setup
            mustBeNonempty(obj.samples);
            obj.end_point_colors = zeros(3, obj.num_samples);
            max_density = max(obj.sample_densities);
            
            % compute colors
            for s = 1:obj.num_samples
                obj.end_point_colors(:, s) = color_gradient(high_color,...
                                                            low_color,...
                                                            max_density,...
                                                            0,...
                                                            obj.sample_densities(s));
            end
        end
        
        function obj = color_ends_concentric(obj,...
                                             base_color,...
                                             contour_color,...
                                             num_contours,...
                                             tol)
            % colors endpoints with concentric contours
            
            % setup
            mustBeNonempty(obj.samples);
            obj.end_point_colors = repmat(base_color', 1, obj.num_samples);
            max_density = max(obj.sample_densities);
            
            % color rings
            bounds = linspace(0, max_density, num_contours+1);
            bounds = bounds(1:end-1);
            for s = 1:obj.num_samples
                for bound = bounds
                    if abs(bound - obj.sample_densities(s)) < tol*bounds(2)
                        obj.end_point_colors(:,s) = contour_color';
                    end
                end
            end
        end
        
        function [ax, fig] = draw(obj, fig_num)
            % plots arm with relevant information
            
            mustBeNonempty(obj.end_points)
            
            % compute mean arm geometry
            [~, link_ends, ~, ~, joint_axis_vectors_R] = link_jacobian(obj.link_vectors,...
                                                                       obj.joint_angles,...
                                                                       obj.joint_axes,...
                                                                       obj.num_links);
            % draw arm
            [ax, fig] = draw_arm_gaussian(fig_num,...
                                   obj.end_points,...
                                   [],...
                                   false,...
                                   obj.end_point_colors',...
                                   link_ends,...
                                   joint_axis_vectors_R);
        end
    end
end

