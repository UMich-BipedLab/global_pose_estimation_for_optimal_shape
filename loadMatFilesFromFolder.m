function files_from_a_folder = loadMatFilesFromFolder(path, extension)
    files_from_a_folder = dir(fullfile(path, extension));
    
    for file = 1:size(files_from_a_folder, 1)
        name = convertCharsToStrings(files_from_a_folder(file).name);
        filepath = convertCharsToStrings(files_from_a_folder(file).folder) + "/";
        files_from_a_folder(file).file_name = filepath + name;
        if contains(extension, "target", 'IgnoreCase', true)
            files_from_a_folder(file).target_scale = identifyOptimalTargetScaleFromName(files_from_a_folder(file).name);
        end
    end
end