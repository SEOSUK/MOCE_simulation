function [B_c_hat,B_c_filtered] = v2_moce(B_F_cmd,B_d_hat,CTRL)
%#codegen
persistent force_state estimate filtered
if isempty(force_state)
    force_state=zeros(2,3); estimate=CTRL.moce.initial_com(:); filtered=estimate;
end
dt=CTRL.sample_time;
[force_state,f,~]=v2_q(force_state,B_F_cmd,CTRL.moce.force_cutoff_rad_s,dt);
if ~CTRL.moce.enable
    estimate=CTRL.moce.initial_com(:);
elseif norm(f)>=CTRL.moce.min_force
    S=[0 -f(3) f(2);f(3) 0 -f(1);-f(2) f(1) 0];
    A=CTRL.nominal.inertia\S;
    rate=v2_bound(CTRL.moce.gamma(:).*(A'*B_d_hat),CTRL.moce.rate_limit);
    if ~CTRL.moce.estimate_z || norm(f(1:2))<CTRL.moce.min_horizontal_force, rate(3)=0; end
    estimate=v2_bound(estimate+dt*rate,CTRL.moce.offset_limit);
end
a=1;
if CTRL.moce.output_tau_s>0, a=1-exp(-dt/CTRL.moce.output_tau_s); end
filtered=filtered+a*(estimate-filtered);
B_c_hat=estimate; B_c_filtered=filtered;
end
