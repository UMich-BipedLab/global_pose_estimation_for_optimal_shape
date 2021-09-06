function out_t = estimateLiDARPose(opts, object_list, template_list)
    % compute line equations
    template_list = computeLineEquations(template_list);
    opts.ncorrespondences.mode = 1;
    
    cur_pc = object_list;
    
    % projects the point cloud to the ideal frame so inv(H_final) reproject 
    solver.H_final = eye(4); 
    solver.H_previous = eye(4);
    solver.cost_pre = 1e5;
    solver.affine_pre = eye(4);
    solver.computation_time = 0;

    for iter = 1: opts.max_iter
        t_start = cputime;
        cur_pc.object_list.centroid = ...
            mean(cur_pc.object_list.points_mat(1:3, :), 2);
        cur_pc.object_list.centered_points = ...
            cur_pc.object_list.points_mat(1:3, :) - ...
            cur_pc.object_list.centroid;

        correspondences = findLineAsCorrespondence(opts, ...
            cur_pc, template_list);


        %% Solver
        s = 1;
        opts.quite = 1;
        opts.show_statistic = 1;
        out_t = solveRCQPAndSignedCost(opts, correspondences, [], s);
        solver.computation_time = ...
            solver.computation_time + (cputime - t_start);
        printOptimalShapeFittingResults(opts, iter, out_t);
        solver.optimized_cost = out_t.sum_cost;

        if solver.optimized_cost < solver.cost_pre
            H = out_t.H.T;
            scaling = [out_t.s    0        0        0
                0        out_t.s    0        0
                0          0      out_t.s    0
                0          0        0        1];
            affine = H * scaling * constructHByRotationTranslation(eye(3), ...
            -cur_pc.object_list.centroid);
            solver.affine_pre = H;
            solver.cost_pre = solver.optimized_cost;
        else
            affine = eye(4);
        end

        transformed_pc = (affine) * converToHomogeneousCoord(...
            cur_pc.object_list.points_mat(1:3, :));
        solver.H_final = (affine) * solver.H_final;

        transformed_pc_in_tempalte = solver.H_final * converToHomogeneousCoord(...
            object_list.object_list.points_mat(1:3, :));
        estimated_vertices = solver.H_final \ ...
            converToHomogeneousCoord(template_list.object_vertices_mat_h);
        cur_pc.object_list.points_mat = transformed_pc;
        solver.H_previous = affine;

        if  norm(Log_SE3( affine \  solver.H_previous)) < 1e-5
            break;
        end
    end
    out_t.solver = solver;
    out_t.template_pc = transformed_pc_in_tempalte;
    out_t.estimated_vertices = estimated_vertices;
end