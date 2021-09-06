function correspondences = findLineAsCorrespondence(opts, ...
    original_pc, template_lines, axes_h)
    if ~exist('axes_h', 'var') || isempty(axes_h)
       opts.debug == 0;
    end

    if opts.debug == 2
        opts.simulate_lidar = 0;
        opts.arrow_length = 0.5;
        plotObjectsList(axes_h, opts, original_pc.LiDAR_opts, ideal_object_list);
        colors = getColors(4);
        for i = 1:4
            scatter3(ideal_object_list.object_vertices.x(i), ...
                 ideal_object_list.object_vertices.y(i), ...
                 ideal_object_list.object_vertices.z(i), ...
                 'MarkerFaceColor', colors{i}, ...
                 'MarkerEdgeColor', colors{i})
        end
    end
    num_points = size(original_pc.object_list.points_mat, 2);
    correspondences = repmat(Point2Line(), 1, ...
        opts.ncorrespondences.num_per_point*num_points);

    num_lines = size(template_lines.line, 2);
    for i = 1:num_points
        if opts.ncorrespondences.num_per_point == 4
            indx = opts.ncorrespondences.num_per_point * (i-1) + 1;
            point = Point(original_pc.object_list.centered_points(1:3, i));
            for j = 1:num_lines
                correspondences(indx + j - 1) = Point2Line(...
                    point, template_lines.line(j));
            end
        elseif opts.ncorrespondences.num_per_point == 2
            indx = opts.ncorrespondences.num_per_point * (i-1) + 1;
            point = Point(original_pc.object_list.centered_points(1:3, i));
            farthest_dis = -1;
            farthest_indx = 0;
            closest_dis = 1e5;
            closest_indx = 0;
            for j = 1:num_lines
                dis = template_lines.line(j).distSq(point);
                if (dis < closest_dis)
                    closest_dis = dis;
                    closest_indx = j;
                end
                if (dis > farthest_dis)
                    farthest_dis = dis;
                    farthest_indx = j;
                end
            end
            correspondences(indx) = Point2Line(...
                point, template_lines.line(closest_indx));
            correspondences(indx+1) = Point2Line(...
                point, template_lines.line(farthest_indx));  
            if opts.debug == 2
                h1 = scatter3(axes_h, point.x(1), point.x(2), point.x(3), 'm*');
                h2 = correspondences(indx).model.plot(axes_h, rand(1,3));
                h3 = correspondences(indx+1).model.plot(axes_h, rand(1,3));
                viewCurrentPlot(axes_h, "process", [-90, 0], 1)
                delete(h1);
                delete(h2);
                delete(h3);
            end
        elseif opts.ncorrespondences.num_per_point == 1
            point = Point(original_pc.object_list.centered_points(1:3, i));
            if opts.ncorrespondences.mode == 1
                closest_dis = 1e5;
                closest_indx = 0;
                for j = 1:num_lines
                    dis = template_lines.line(j).distSq(point);
                    if (dis < closest_dis)
                        closest_dis = dis;
                        closest_indx = j;
                    end
                end
                correspondences(i) = Point2Line(...
                    point, template_lines.line(closest_indx));
            elseif opts.ncorrespondences.mode == 2
                farthest_dis = -1;
                farthest_indx = 0;
                for j = 1:num_lines
                    dis = template_lines.line(j).distSq(point);
                    if (dis > farthest_dis)
                        farthest_dis = dis;
                        farthest_indx = j;
                    end
                end
                correspondences(i) = Point2Line(...
                    point, template_lines.line(farthest_indx));
            end

            if opts.debug == 2
                h1 = scatter3(axes_h, point.x(1), point.x(2), point.x(3), 'm*');
                h2 = correspondences(i).model.plot(axes_h, rand(1,3));
                viewCurrentPlot(axes_h, "process", [-90, 0], 1)
                delete(h1);
                delete(h2);
            end
        end
    end
end