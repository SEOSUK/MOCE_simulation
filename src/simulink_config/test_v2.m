function report = test_v2()
% Structural checks, equation regressions, hover, and independent mode toggles.
here=fileparts(mfilename('fullpath'));src=fileparts(here);addpath(here,fullfile(here,'kernels'));
if ~isfile(fullfile(src,'V2.slx')),build_v2();end
load_system(fullfile(src,'V2.slx'));m='V2';
cleanup=onCleanup(@()restore(m));
w=get_param(m,'ModelWorkspace');
assignin(w,'V2_OVERRIDES',struct());v2_initialize_model(m);
assert(isempty(find_system(m,'SearchDepth',1,'BlockType','Goto')));
assert(isempty(find_system(m,'SearchDepth',1,'BlockType','From')));
assert(isempty(find_system(m,'SearchDepth',1,'BlockType','Scope')));
required={'Command Generator','Position Controller','Velocity Controller','Attitude Controller', ...
    'Rate Controller + DOB','CoM Estimator','Control Allocator','Propeller Dynamics','Servo Dynamics', ...
    'Rigid-Body Plant','Measurement Model','Monitoring'};
for k=1:numel(required),assert(getSimulinkBlockHandle([m '/' required{k}])>0);end
links=find_system(m,'LookUnderMasks','all','FollowLinks','off','Type','block');
for k=1:numel(links),assert(~strcmp(get_param(links{k},'LinkStatus'),'unresolved'));end
structure_checks(m);
test_v2_pid_blocks(m);
unit_checks();
% Current file defaults, including the configured payload and actuator model.
report=struct('case',{},'duration',{},'max_position_error',{},'final_position_error',{},'max_body_rate',{});
report(end+1)=run_case(m,'default_hover',struct(),10);
paths={'position.velocity_ff.enable','velocity.gravity_comp.enable','dob.enable','moce.enable','rate.gyroscopic_ff.enable'};
for k=1:numel(paths)
    for value=[false true]
        ov=struct(); parts=strsplit(paths{k},'.');
        ov.CTRL=setfield(struct(),parts{:},value); %#ok<SFLD>
        label=sprintf('%s=%d',paths{k},value);
        report(end+1)=run_case(m,label,ov,1);
    end
end
% All observer modes, with adaptation enabled even in estimation-only mode.
for enabled=[false true]
    for compensate=[false true]
        ov=struct();ov.CTRL.dob=struct('enable',enabled,'compensate',compensate);
        ov.CTRL.moce.enable=true;
        report(end+1)=run_case(m,sprintf('dob_mode_%d_%d',enabled,compensate),ov,2);
    end
end
% Exercise nonzero references, payload derivation, and force-driven adaptation.
ov.EXP.payload=struct('mass',.5,'position',[.15 .08 .05]);
ov.EXP.command=struct('position',[.15 -.1 1],'attitude',[.04 -.03 .1]);
ov.CTRL.moce=struct('enable',true,'estimate_z',true);
report(end+1)=run_case(m,'payload_and_excitation',ov,3);
% Reproducible sensor noise across independent simulation runs.
noise=struct();groups={'position','velocity','attitude','gyro'};
for k=1:4,noise.SENSOR.(groups{k}).enable=true;end
[~,a]=run_case(m,'noise_seed_repeat_1',noise,.2);
[~,b]=run_case(m,'noise_seed_repeat_2',noise,.2);
assert(isequal(a.logsout.get('B_omega').Values.Data,b.logsout.get('B_omega').Values.Data));
noise.EXP.seed=2;[~,c]=run_case(m,'noise_seed_changed',noise,.2);
assert(~isequal(a.logsout.get('B_omega').Values.Data,c.logsout.get('B_omega').Values.Data));
restore(m);
% Text report is small and reviewable; large trajectory/cache files stay local.
fid=fopen(fullfile(here,'validation.txt'),'w');f=onCleanup(@()fclose(fid));
fprintf(fid,'MATLAB %s\nGenerated: %s\n',version,datestr(now,31));
fprintf(fid,'Native PID structure, Scope wiring, logging, equation, all DOB modes, MOCE routing, payload, and noise checks PASS.\n');
for k=1:numel(report)
    fprintf(fid,'%s: %.2f s, max |position error| %.6g m, final error %.6g m, max |body rate| %.6g rad/s\n', ...
        report(k).case,report(k).duration,report(k).max_position_error,report(k).final_position_error,report(k).max_body_rate);
end
disp(struct2table(report));fprintf('V2 validation passed.\n');
end
function [r,out]=run_case(m,label,ov,duration)
w=get_param(m,'ModelWorkspace');assignin(w,'V2_OVERRIDES',ov);
v2_initialize_model(m);set_param(m,'SimulationCommand','update');
out=sim(m,'StopTime',num2str(duration));
required={'W_p_ref','W_p','W_v_ref','W_v','euler_ref','euler','B_omega_ref','B_omega', ...
    'B_F_cmd','B_tau_pid','B_tau_nom','B_tau_cmd','B_d_hat','B_c_hat','B_F_actual','B_tau_actual', ...
    'thrust_cmd','thrust','servo_cmd','servo_angle','B_c_true','B_c_filtered','B_d_hat_to_MOCE'};
for k=1:numel(required)
    sig=out.logsout.get(required{k});assert(~isempty(sig),'Missing log: %s',required{k});
    assert(all(isfinite(sig.Values.Data),'all'),'Nonfinite signal %s in %s',required{k},label);
    width=3;if any(strcmp(required{k},{'thrust_cmd','thrust','servo_cmd','servo_angle'})),width=4;end
    assert(numel(sig.Values.Data)==width*numel(sig.Values.Time),'Wrong width: %s',required{k});
end
c=getVariable(w,'CTRL');
ts=out.logsout.get('B_tau_cmd').Values;
tau=reshape(ts.Data,3,[]);
assert(all(abs(tau)<=c.torque_limit(:)+1e-10,'all'));
d=out.logsout.get('B_d_hat').Values.Data;
if ~c.dob.enable,assert(all(d==0,'all'));end
nom=values_at(out.logsout.get('B_tau_nom').Values,ts.Time);
expected=nom;
if c.dob.enable && c.dob.compensate,expected=expected-values_at(out.logsout.get('B_d_hat').Values,ts.Time);end
expected=min(max(expected,-c.torque_limit(:)),c.torque_limit(:));
assert(max(abs(tau-expected),[],'all')<1e-10,'DOB compensation/sign mismatch.');
% The diagnostic branch entering MOCE must carry the published estimate.
dm=out.logsout.get('B_d_hat_to_MOCE').Values;
dp=out.logsout.get('B_d_hat').Values;
assert(max(abs(values_at(dm,ts.Time)-values_at(dp,ts.Time)),[],'all')<1e-14,'MOCE did not receive published DOB.');
plant=getVariable(w,'PLANT');
truth=reshape(out.logsout.get('B_c_true').Values.Data,3,[]);
assert(max(abs(truth-plant.derived.com(:)),[],'all')<1e-14);
com=reshape(out.logsout.get('B_c_hat').Values.Data,3,[]);
assert(all(abs(com)<=c.moce.offset_limit(:)+1e-10,'all'));
if ~c.moce.enable,assert(all(abs(com-c.moce.initial_com(:))<1e-14,'all'));end
if startsWith(label,'dob_mode_1_')
    assert(max(abs(d),[],'all')>1e-5,'Enabled observer is inactive.');
    assert(max(abs(com-c.moce.initial_com(:)),[],'all')>1e-9,'Enabled MOCE did not adapt.');
end
p=squeeze(out.logsout.get('W_p').Values.Data);
% Simulink column-vector timeseries are 3x1xN (squeeze -> 3xN).
if size(p,1)~=3,p=p';end
e=getVariable(w,'EXP');err=vecnorm(p-e.command.position(:));
omega=out.logsout.get('B_omega').Values.Data;
r=struct('case',label,'duration',duration,'max_position_error',max(err),'final_position_error',err(end), ...
    'max_body_rate',max(abs(omega),[],'all'));
if strcmp(label,'default_hover')
    save(fullfile(tempdir,'v2_default_hover.mat'),'out');
    assert(norm(p(:,end)-e.command.position(:))<.25,'Hover failed to settle near reference.');
    assert(max(abs(omega),[],'all')<2,'Hover has excessive body rate.');
end
fprintf('PASS %s (%.2f s)\n',label,duration);
end
function unit_checks()
cfg=init_v2();c=cfg.CTRL;p=cfg.PLANT;
% Parallel-axis theorem about the *resulting* combined CoM.
ov.EXP.payload=struct('mass',2,'position',[.3 -.2 .4]);z=init_v2('',ov);
mu=p.vehicle.mass*2/(p.vehicle.mass+2);r=ov.EXP.payload.position(:)-p.vehicle.com(:);
expected=p.vehicle.inertia+mu*(dot(r,r)*eye(3)-r*r');
assert(norm(z.PLANT.derived.inertia-expected,'fro')<1e-12);
assert(norm(v2_rotvec(v2_rotation([0;0;2*pi-.01]))-[0;0;-.01])<1e-10);
assert(abs(norm(v2_rotvec(v2_rotation([pi;0;0])))-pi)<1e-10);
% Exact Q matches matrix exponential for arbitrary dt, not Euler integration.
x=[.1 .2 .3;-.2 .1 .4];u=[1;2;3];dt=.003;wc=2;
[next,~,~]=v2_q(x,u,wc,dt);A=[-sqrt(2)*wc -wc^2;1 0];eq=[zeros(1,3);u'/wc^2];
assert(norm(next-(eq+expm(A*dt)*(x-eq)),'fro')<1e-12);
% Independent reaction coefficients affect the intended components only.
[p0,d0,r0]=v2_rotors(.2*ones(4,1),c.nominal.geometry,c.nominal.propeller);
pp=c.nominal.propeller;pp.b_over_k=2*pp.b_over_k;
[~,~,r1]=v2_rotors(.2*ones(4,1),c.nominal.geometry,pp);
assert(norm(r1(3,:)-r0(3,:))==0 && norm(r1(1:2,:)-2*r0(1:2,:))<1e-14);
% Geometry follows the editable nominal configuration (no stale V1 constants).
g=c.nominal.geometry;
assert(norm(p0-[g.arm_xy*g.radial_xy;g.rotor_z*ones(1,4)],'fro')<1e-12);
assert(norm(d0-(g.tangent*sin(.2)+[0;0;cos(.2)]*ones(1,4)),'fro')<1e-12);
clear v2_allocator
[f,theta]=v2_allocator([0;0;6*9.80665],zeros(3,1),zeros(3,1),zeros(4,1),c);
assert(max(abs(f-6*9.80665/4))<1e-10 && norm(theta)<1e-10);
fprintf('PASS equation and convention checks\n');
end
function restore(m)
if bdIsLoaded(m)
    w=get_param(m,'ModelWorkspace');assignin(w,'V2_OVERRIDES',struct());
    v2_initialize_model(m);
end
end

function structure_checks(m)
controllers={'Position Controller/Position PID','Velocity Controller/Velocity PID', ...
    'Attitude Controller/Attitude PID','Rate Controller + DOB/Rate PID'};
for k=1:numel(controllers)
    blocks=find_system([m '/' controllers{k}],'SearchDepth',1,'MaskType','PID 1dof');
    assert(numel(blocks)==3,'Expected three native axis PID blocks.');
    for j=1:3
        assert(startsWith(get_param(blocks{j},'P'),'CTRL.'));
        assert(startsWith(get_param(blocks{j},'I'),'CTRL.'));
        assert(startsWith(get_param(blocks{j},'D'),'CTRL.'));
        assert(strcmp(get_param(blocks{j},'UseExternalDerivativeSource'),'on'));
    end
end
ss=find_system(m,'LookUnderMasks','all','FollowLinks','off','BlockType','StateSpace');
assert(numel(ss)==9,'Expected explicit three-axis Q, sQ, and MOCE force Q filters.');
assert(getSimulinkBlockHandle([m '/CoM Estimator/Projected CoM integrator'])>0);
r=sfroot;charts=r.find('-isa','Stateflow.EMChart');
for k=1:numel(charts)
    if startsWith(charts(k).Path,[m '/'])
        assert(isempty(regexp(charts(k).Script,'v2_(position|velocity|attitude|rate_dob|moce|pid|q)\(','once')), ...
            'Research-critical controller kernel remains hidden in MATLAB code.');
    end
end
scopes={'Position Tracking',2;'Velocity Tracking',2;'Attitude Tracking',2; ...
    'Rate Tracking',2;'Body Force',2;'Torque and DOB',5;'CoM Estimation',3; ...
    'Propeller Commands',2;'Servo Commands',2};
assert(numel(find_system(m,'LookUnderMasks','all','BlockType','Scope'))==9);
for k=1:size(scopes,1)
    b=[m '/Monitoring/' scopes{k,1}];ports=get_param(b,'PortHandles');
    assert(numel(ports.Inport)==scopes{k,2});
    for port=ports.Inport,assert(get_param(port,'Line')~=-1);end
    config=get_param(b,'ScopeConfiguration');assert(config.ShowLegend);
end
assert(isempty(find_system([m '/Monitoring'],'SearchDepth',1,'BlockType','Outport')));
assert(strcmp(get_param(m,'SignalLogging'),'on'));
assert(strcmp(get_param(m,'AlgebraicLoopMsg'),'error'));
fprintf('PASS visible PID/DOB/MOCE structure and nine diagnostic Scopes\n');
end

function data=values_at(signal,t)
data=reshape(signal.Data,3,[])';
if numel(signal.Time)==1,data=repmat(data,numel(t),1);
else,data=interp1(signal.Time,data,t,'previous','extrap');end
data=data';
end
