function printOptimalShapeFittingResults(opts, iter, results)
    if opts.show_statistic == 1
        disp("----------------------------------")
        fprintf("iter: %i\n", iter)
        fprintf("current scaling: %.4f\n", results.s)
        fprintf("current gap: %.3f\n", results.gap)
        fprintf("current fstar: %.4f\n", results.f)
        fprintf("current dstar: %.4f\n", results.dstar)
        if isfield(results, 'original_cost')
            fprintf("original cost: %.3f\n", results.original_cost)
        end
        fprintf("optimized cost: %.3f\n", results.sum_cost)
    end
end