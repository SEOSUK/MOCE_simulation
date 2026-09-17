function [B_tau_cmd,B_d_hat,B_tau_pid,B_tau_nom] = v2_rate_dob(B_omega_ref,B_omega,CTRL)
%#codegen
persistent integral derivative previous rate_state torque_state estimate
if isempty(integral)
    integral=zeros(3,1); derivative=zeros(3,1); previous=B_omega;
    rate_state=[zeros(1,3);B_omega(:)'/CTRL.dob.cutoff_rad_s^2];
    torque_state=zeros(2,3); estimate=zeros(3,1);
end
dt=CTRL.sample_time; J=CTRL.nominal.inertia;
[B_tau_pid,integral,derivative]=v2_pid(B_omega_ref-B_omega,(B_omega-previous)/dt, ...
    integral,derivative,CTRL.rate,dt,zeros(3,1));
gyro=cross(B_omega,J*B_omega);
B_tau_nom=B_tau_pid;
if CTRL.rate.gyroscopic_ff.enable, B_tau_nom=B_tau_nom+gyro; end
B_d_hat=zeros(3,1);
if CTRL.dob.enable
    % C++ predictor uses previous estimate, bounds torque, then removes actual
    % nominal gyroscopic torque irrespective of feed-forward switch.
    observer_input=B_tau_nom;
    if CTRL.dob.compensate, observer_input=observer_input-estimate; end
    effective=v2_bound(observer_input,CTRL.torque_limit)-gyro;
    [rate_state,~,dq]=v2_q(rate_state,B_omega,CTRL.dob.cutoff_rad_s,dt);
    [torque_state,qt,~]=v2_q(torque_state,effective,CTRL.dob.cutoff_rad_s,dt);
    B_d_hat=v2_bound(J*dq-qt,CTRL.dob.estimate_limit);
end
estimate=B_d_hat;
B_tau_cmd=B_tau_nom;
if CTRL.dob.enable && CTRL.dob.compensate, B_tau_cmd=B_tau_cmd-B_d_hat; end
B_tau_cmd=v2_bound(B_tau_cmd,CTRL.torque_limit);
previous=B_omega;
end
