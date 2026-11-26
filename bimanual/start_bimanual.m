function start_bimanual()
% Run from this folder; restore the previous folder and path afterward.
root = fileparts(mfilename('fullpath'));
oldFolder = pwd;
oldPath = path;
cleanup = onCleanup(@() restoreEnvironment(oldFolder, oldPath));
cd(fullfile(root, 'matlab'));
addpath(pwd);
BimanualMain();
end

function restoreEnvironment(oldFolder, oldPath)
cd(oldFolder);
path(oldPath);
end
