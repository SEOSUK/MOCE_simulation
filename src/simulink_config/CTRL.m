function c = CTRL()
% Controller-owned parameters. SI units; vectors ordered x,y,z.
c.sample_time = 0.0025;
c.position = pid([2.5 2.5 3.4], [0 0 0], [0 0 0], [.3 .3 .3], [10 10 100], 10);
c.position.velocity_ff.enable = false;
c.position.velocity_ff.cutoff_hz = 10;
c.velocity = pid([3.5 3.5 8], [.12 .12 2.5], [.08 .08 .07], [10 10 10], [1000 1000 1000], 10);
c.velocity.gravity_comp.enable = true;
c.attitude = pid([10 10 4.5], [0 0 0], [.05 .05 .01], [.2 .2 .2], [3.84 3.84 3.49], 15);
c.rate = pid([2.2 2.2 1.6], [0 0 0], [.04 .04 .02], [.4 .4 .4], [4 4 3], 20);
c.rate.gyroscopic_ff.enable = true;
c.force_limit = [30 30 100];
c.torque_limit = [5 5 4];
c.nominal.mass = 4.7; % independent of the true payload-derived mass
c.nominal.gravity = 9.81;
c.nominal.inertia = diag([.0768 .0871 .113]);
% MuJoCo ModelConfig geometry, indices 1..4 and positive servo sense.
c.nominal.geometry.arm_xy = .148492;
c.nominal.geometry.rotor_z = .07;
c.nominal.geometry.servo_pivot_offset = .075;
c.nominal.geometry.radial_xy = [1 -1 -1 1; 1 1 -1 -1];
c.nominal.geometry.tangent = [1 1 -1 -1; -1 1 1 -1; 0 0 0 0]/sqrt(2);
c.nominal.geometry.spin = [1 -1 1 -1]; % MuJoCo reaction torque relative to thrust
% Equal defaults reproduce the single MuJoCo reaction ratio.
c.nominal.propeller.b_over_k = .02; % lateral reaction torque / lateral thrust
c.nominal.propeller.reaction_torque_ratio = .02; % axial reaction torque / axial thrust
c.nominal.propeller.thrust_coefficient = .02; % N/(rad/s)^2; thrust-domain allocator
c.nominal.propeller.max_thrust = 50;
c.nominal.servo_limit = .7;
c.allocator.yaw_split_cutoff_hz = .5;
c.allocator.yaw_reaction_limit = .6;
c.allocator.regularization = 1e-8;
c.dob.enable = true;
c.dob.compensate = true;
c.dob.cutoff_rad_s = 2;
c.dob.estimate_limit = [3 3 3];
c.moce.enable = false;
c.moce.gamma = [7e-6 7e-6 1.5e-4];
c.moce.initial_com = [0 0 0];
c.moce.offset_limit = [.15 .15 .15];
c.moce.rate_limit = [.02 .02 .02];
c.moce.min_force = 5;
c.moce.estimate_z = false;
c.moce.min_horizontal_force = .1;
c.moce.force_cutoff_rad_s = 2;
c.moce.output_tau_s = .15;
end
function p = pid(kp,ki,kd,il,ol,fc)
p = struct('Kp',kp,'Ki',ki,'Kd',kd,'integral_limit',il, ...
    'output_limit',ol,'derivative_cutoff_hz',fc);
end
