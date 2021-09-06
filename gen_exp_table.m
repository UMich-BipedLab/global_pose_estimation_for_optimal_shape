clear, clc, close all
project_name = "ExpTableForPaper";
clear_fig = 1;
save_fig = 0;
fig = createFigureOptions(3, 1, project_name, clear_fig, 1, 30);
fig.save_figs = save_fig;
[axes_handles, fig_handles] = createFigHandleWithOptions(fig);


addpath(genpath("./data/"))
addpath(genpath("./paper_results/"))
exp_results(1).T = load('exp_result-1.mat');
exp_results(2).T = load('exp_result-2.mat');
exp_results(3).T = load('exp_result-3.mat');
exp_results(4).T = load('exp_result-4.mat');
exp_results(5).T = load('exp_result-5.mat');


%%
varNames = {'Name', 'Distance','QuantizationError','NumberOfPoints', ...
    'RMSE', 'RMSE_percentage', ...
    'translation_error', 'translation_error_percentage', 'rot_error'};

% table_for_paper = table('VariableNames', varNames
face_on = [];
face_off = [];
face_on_with_names = [];
face_off_with_names = [];
for i = 1 : length(exp_results)
    cur_tables = exp_results(i).T.result_cell;
    for j = 1:length(cur_tables)
        cur_table = cur_tables{j};
        name = cur_table.Name{1};
        name = name(1:strfind(name, "-L")-1);
        cur_table.Name{1} = name;
        if contains(name, "angled", 'IgnoreCase', true)
            face_off = [face_off; cur_table(1, 2:9)];
            face_off_with_names = [face_off_with_names; cur_table(1, 1:9)];
        else
            face_on = [face_on; cur_table(1, 2:9)];
            face_on_with_names = [face_on_with_names; cur_table(1, 1:9)];
        end
    end
end
face_on_with_names
face_off_with_names

%%
% [axes_h, fig_h] = getCurrentFigure(1, axes_handles, fig_handles);
% h1 = plot(axes_h, face_on.Distance, face_on.RMSE);
% h2 = plot(axes_h, face_on.Distance, face_on.translation_error);
% h3 = plot(axes_h, face_on.Distance, face_on.rot_error);
% assignLegend(axes_h, [h1, h2, h3], ["RMSE", "$e_t$", "$e_r$"], 15, 5)
