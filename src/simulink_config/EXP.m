function e = EXP()
% Commands and payload positions use FLU / world Z-up coordinates.
e.command.position = [0 0 1]; % V1 z=-1 converted to Z-up
% 0: constant, 1: delayed V1-style sine on selected axes.
e.command.position_mode = 0; % safe hover default; select 1 for V1 trajectory
e.command.position_sine_amplitude = [0 -10 0];
e.command.position_sine_rad_s = [0 .5 0];
e.command.position_sine_phase = [0 0 0];
e.command.position_start_s = 10; % sine uses absolute simulation time, as V1
e.command.attitude = [0 0 0];
e.command.attitude_sine_enable = [false false false];
e.command.attitude_sine_amplitude = [.2 -.2 -1];
e.command.attitude_sine_rad_s = [.4 .4 .2];
e.command.attitude_sine_phase = [0 0 0];
e.payload.mass = .5;
e.payload.position = [.31 .31 .56]; % relative to body geometry origin, point mass
e.initial.position = [0 0 .145]; % fixed body origin, like MuJoCo base_pos
e.initial.velocity = [0 0 0]; % fixed body origin, like MuJoCo base_linvel
e.initial.attitude = [0 0 0];
e.initial.body_rate = [0 0 0];
e.seed = 1;
e.stop_time = 10;
e.physics_step = .00125;
end
