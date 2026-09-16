function p = PLANT()
% True physical model, independent of CTRL.nominal. V1 defaults in FLU.
p.vehicle.mass = 6;
p.vehicle.com = [.01 .005 .001]; % V1 [.01 -.005 -.001] FRD -> FLU
p.vehicle.inertia = diag([.3 .3 .4]); % about unloaded CoM
p.gravity = 9.80665;
p.geometry.arm_length = .4;
p.geometry.rotor_z = -.015;
p.geometry.radial_xy = [1 -1 -1 1; 1 1 -1 -1]/sqrt(2);
p.geometry.tangent = [1 1 -1 -1; -1 1 1 -1; 0 0 0 0]/sqrt(2);
p.geometry.spin = [-1 1 -1 1];
p.propeller.b_over_k = .01;
p.propeller.reaction_torque_ratio = .01;
p.propeller.time_constant_s = .01;
p.propeller.delay_s = 0;
p.propeller.effectiveness = .8*ones(4,1);
p.propeller.max_thrust = 200; % saturation before lag and effectiveness, as in V1
p.propeller.initial_thrust = zeros(4,1);
p.servo.time_constant_s = 1/30;
p.servo.delay_s = 0;
p.servo.dc_gain = 1;
p.servo.limit_rad = 1;
p.servo.initial_angle = zeros(4,1);
end
