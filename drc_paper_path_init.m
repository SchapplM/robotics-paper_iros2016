% Füge alle Unterordner der Beiträge zum Matlab-Pfad hinzu, damit die
% Skripte funktionieren.

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-09
% (c) Institut für Regelungstechnik, Universität Hannover

paper_path = fileparts(which('drc_paper_path_init'));
addpath(paper_path);
addpath(fullfile(paper_path, 'data'));
addpath(fullfile(paper_path, 'identification'));
addpath(fullfile(paper_path, 'plotting'));
addpath(fullfile(paper_path, 'robot_model'));
addpath(fullfile(paper_path, 'simulink'));