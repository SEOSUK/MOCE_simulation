function B_F_cmd = v2_velocity(W_v_ref,W_v,euler,CTRL)
%#codegen
persistent integral derivative previous
if isempty(integral), integral=zeros(3,1); derivative=zeros(3,1); previous=W_v; end
dt=CTRL.sample_time;
[a,integral,derivative]=v2_pid(W_v_ref-W_v,(W_v-previous)/dt,integral,derivative,CTRL.velocity,dt,zeros(3,1));
if CTRL.velocity.gravity_comp.enable, a(3)=a(3)+CTRL.nominal.gravity; end
W_F_cmd=CTRL.nominal.mass*a;
W_F_cmd(3)=max(0,W_F_cmd(3)); % C++ nonnegative vertical thrust demand
B_F_cmd=v2_bound(v2_rotation(euler)'*W_F_cmd,CTRL.force_limit);
previous=W_v;
end
