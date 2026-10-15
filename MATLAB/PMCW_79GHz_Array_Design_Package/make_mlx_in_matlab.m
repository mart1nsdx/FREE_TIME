% Converts the Live Script source .m file into a MATLAB .mlx file.
% Run this inside MATLAB from the folder containing PMCW_79GHz_Array_Design_LiveScript_Source.m
src = fullfile(pwd,'PMCW_79GHz_Array_Design_LiveScript_Source.m');
dst = fullfile(pwd,'PMCW_79GHz_Array_Design.mlx');
if exist(src,'file') ~= 2
    error('Source file not found: %s', src);
end
try
    matlab.internal.liveeditor.openAndSave(src, dst);
    fprintf('Created Live Script: %s\n', dst);
catch ME
    warning('Automatic .mlx conversion failed: %s', ME.message);
    fprintf(['Open %s in MATLAB Live Editor manually, then use Save As > MATLAB Live Script (*.mlx).\n'], src);
end
