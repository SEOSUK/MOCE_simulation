function W_v_ref = v2_position(W_p_ref,W_p,CTRL)
%#codegen
persistent integral derivative previous previous_ref ff
if isempty(integral)
    integral=zeros(3,1); derivative=zeros(3,1); previous=W_p;
    previous_ref=W_p_ref; ff=zeros(3,1);
end
dt=CTRL.sample_time;
ff=ff+(1-exp(-2*pi*CTRL.position.velocity_ff.cutoff_hz*dt))*((W_p_ref-previous_ref)/dt-ff);
feedforward=zeros(3,1);
if CTRL.position.velocity_ff.enable, feedforward=ff; end
[W_v_ref,integral,derivative]=v2_pid(W_p_ref-W_p,(W_p-previous)/dt, ...
    integral,derivative,CTRL.position,dt,feedforward);
previous=W_p; previous_ref=W_p_ref;
end
