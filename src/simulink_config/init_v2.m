function cfg = init_v2(model, overrides)
% Reload editable files every initialization; apply explicit test overrides.
% No controller or physical parameters are read from the base workspace.
if nargin < 1, model = ''; end
if nargin < 2, overrides = struct(); end
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'kernels'));
cfg.CTRL = CTRL(); cfg.PLANT = PLANT(); cfg.SENSOR = SENSOR(); cfg.EXP = EXP();
cfg = merge(cfg,overrides);
p = cfg.PLANT; e = cfg.EXP;
% Match PlantModel's neutral-joint unloaded body aggregation. The base CoM
% balances rotor first moments so the unloaded total CoM remains vehicle.com.
base_mass=p.vehicle.mass-4*p.vehicle.rotor_mass;
assert(base_mass>0,'Vehicle mass includes four rotor bodies.');
rotor_positions=zeros(3,4); rotor_rotations=zeros(3,3,4);
for i=1:4
    yaw=atan2(p.geometry.radial_xy(2,i),p.geometry.radial_xy(1,i));
    R=v2_rotation([0;0;yaw]); rotor_rotations(:,:,i)=R;
    mount=[p.geometry.arm_xy*p.geometry.radial_xy(:,i); ...
        p.geometry.rotor_z+p.geometry.servo_pivot_offset];
    rotor_positions(:,i)=mount-[0;0;p.geometry.servo_pivot_offset]+R*p.vehicle.rotor_com(:);
end
base_com=(p.vehicle.mass*p.vehicle.com(:)-p.vehicle.rotor_mass*sum(rotor_positions,2))/base_mass;
p.vehicle.inertia=p.vehicle.base_inertia+parallel_axis(base_mass,base_com-p.vehicle.com(:));
for i=1:4
    R=rotor_rotations(:,:,i);
    p.vehicle.inertia=p.vehicle.inertia+R*p.vehicle.rotor_inertia*R' ...
        +parallel_axis(p.vehicle.rotor_mass,rotor_positions(:,i)-p.vehicle.com(:));
end
p.vehicle.base_com=base_com; % derived, not independently editable

assert(e.payload.mass >= 0 && p.vehicle.mass > 0,'Mass must be nonnegative and vehicle mass positive.');
m = p.vehicle.mass + e.payload.mass;
c = (p.vehicle.mass*p.vehicle.com(:) + e.payload.mass*e.payload.position(:))/m;
r = p.vehicle.com(:)-c; q = e.payload.position(:)-c;
p.derived.mass = m;
p.derived.com = c;
p.derived.inertia = p.vehicle.inertia + p.vehicle.mass*(dot(r,r)*eye(3)-r*r') ...
    + e.payload.mass*(dot(q,q)*eye(3)-q*q');
assert(all(eig(p.derived.inertia)>0),'True inertia must be positive definite.');
assert(all(eig(cfg.CTRL.nominal.inertia)>0),'Nominal inertia must be positive definite.');
assert(cfg.CTRL.dob.cutoff_rad_s>0 && cfg.CTRL.moce.force_cutoff_rad_s>0);
assert(all(abs(cfg.CTRL.moce.initial_com)<=cfg.CTRL.moce.offset_limit));
assert(cfg.CTRL.sample_time>0 && e.physics_step>0);
assert(abs(cfg.CTRL.sample_time/e.physics_step-round(cfg.CTRL.sample_time/e.physics_step))<1e-9);
% EXP initial position/velocity refer to O. Initialize C consistently even
% when the initial attitude and body rate are nonzero.
R0=v2_rotation(e.initial.attitude(:));
p.derived.W_p_C0=e.initial.position(:)+R0*c;
p.derived.W_v_C0=e.initial.velocity(:)+R0*cross(e.initial.body_rate(:),c);
assert(p.propeller.time_constant_s>0 && p.servo.time_constant_s>0);
assert(p.propeller.effectiveness.min<=p.propeller.effectiveness.max);
assert(all(p.propeller.relative_scale>=0 & p.propeller.relative_scale<=1));
cfg.PLANT = p;
if ~isempty(model)
    w = get_param(model,'ModelWorkspace');
    names = fieldnames(cfg);
    for k=1:numel(names), assignin(w,names{k},cfg.(names{k})); end
end
end
function a = merge(a,b)
f = fieldnames(b);
for k=1:numel(f)
    n=f{k};
    if isfield(a,n) && isstruct(a.(n)) && isstruct(b.(n)), a.(n)=merge(a.(n),b.(n));
    else, a.(n)=b.(n); end
end
end

function J=parallel_axis(m,r)
J=m*(dot(r,r)*eye(3)-r*r');
end
