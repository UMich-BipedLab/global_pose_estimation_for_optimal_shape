function obj_list = computeLineEquations(obj_list)
    line_order = [1,2; 
                  2,3;
                  3,4;
                  4,1;];
    for i = 1:length(line_order)
        point_on_line = obj_list.object_vertices_mat(:, i);
        line_dir = ...
            obj_list.object_vertices_mat(:, line_order(i, 2)) - ...
            obj_list.object_vertices_mat(:, line_order(i, 1));
        line_dir = line_dir ./ norm(line_dir);
        obj_list.line(i) = Line(point_on_line, line_dir);
    end
end