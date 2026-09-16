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
