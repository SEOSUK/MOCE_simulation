function B_omega_ref = v2_attitude(euler_ref,euler,B_omega,CTRL)
%#codegen
persistent integral derivative
if isempty(integral), integral=zeros(3,1); derivative=zeros(3,1); end
error=v2_rotvec(v2_rotation(euler)'*v2_rotation(euler_ref));
[B_omega_ref,integral,derivative]=v2_pid(error,B_omega,integral,derivative,CTRL.attitude,CTRL.sample_time,zeros(3,1));
end
