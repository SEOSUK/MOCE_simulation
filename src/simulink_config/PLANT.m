function p = PLANT()
% True lumped model. Default values follow palletrone_interfaces/config/model.yaml.
% init_v2 combines base + four neutral rotor bodies, then the point payload.
p.vehicle.mass = 4.2; % includes the four rotors
p.vehicle.com = [0 0 0]; % MuJoCo balances the unloaded vehicle about body origin
p.vehicle.base_inertia = diag([.09062181 .05262181 .04652020]);
p.vehicle.rotor_mass = .2;
p.vehicle.rotor_inertia = diag([.0003 .0003 .0003]);
p.vehicle.rotor_com = [0 0 -.0466612]; % relative to thrust site in rotor frame
p.gravity = 9.81;
p.geometry.arm_xy = .148492;
p.geometry.rotor_z = .07;
p.geometry.servo_pivot_offset = .075; % mount-to-thrust-site; see README
p.geometry.radial_xy = [1 -1 -1 1; 1 1 -1 -1];
p.geometry.tangent = [1 1 -1 -1; -1 1 1 -1; 0 0 0 0]/sqrt(2);
p.geometry.spin = [1 -1 1 -1]; % MuJoCo BLDC1..4, replaces V1 reaction signs
p.propeller.thrust_coefficient = .02; % documented speed/thrust conversion; no speed state
p.propeller.b_over_k = .02;
p.propeller.reaction_torque_ratio = .02;
p.propeller.time_constant_s = .017;
p.propeller.delay_s = .01;
p.propeller.effectiveness.enable = true;
p.propeller.effectiveness.initial = .6966;
p.propeller.effectiveness.slope_per_s = -.000844;
p.propeller.effectiveness.min = .63;
p.propeller.effectiveness.max = .70;
p.propeller.relative_scale = ones(4,1); % control.yaml motor.thrust_scale
p.propeller.sample_time = .0025; % MuJoCo command/physics sample period
p.propeller.max_thrust = 50;
p.propeller.initial_thrust = zeros(4,1);
p.servo.time_constant_s = .052; % equivalent FOPDT tau; physical joint remains MuJoCo-only
p.servo.delay_s = .05;
p.servo.dc_gain = .994;
p.servo.limit_rad = .7;
p.servo.initial_angle = zeros(4,1);
end
