

%% Example of the frc_racecars in simulation without obstacle avoidance:
%% This example is for the optimal racing of the frc race cars. The model is a simple bicycle model and the lateral acceleration is constraint in order to validate the model assumptions.
%% The simulation starts at s=-2m until one round is completed(s=8.71m). The beginning is cut in the final plots to simulate a 'warm start'. 

addpath('helper_functions', 'tracks');

track_file = 'Trackinfo.txt';
[Sref, ~, ~, ~, ~] = getTrack(track_file);



%% horizon parameters
N = 50;
T = 1.0; % time horizon length,这样计算步长就是det_T = T/N

%% model dynamics
[model, constraint] = bicycle_model(track_file);

nx = length(model.x);
nu = length(model.u);


open_system( 'simulink_model_closedloop')
% open_system(fullfile(target_folder, 'simulink_model_closedloop_all'))
%%
disp('Press play in Simulink!');
plotted