function [thrust_cmd,servo_cmd] = v2_allocator(B_F_cmd,B_tau_cmd,B_c_hat,servo_angle,CTRL)
%#codegen
persistent yaw_lpf
if isempty(yaw_lpf), yaw_lpf=0; end
yaw_lpf=yaw_lpf+(1-exp(-2*pi*CTRL.allocator.yaw_split_cutoff_hz*CTRL.sample_time))*(B_tau_cmd(3)-yaw_lpf);
r=max(-CTRL.allocator.yaw_reaction_limit,min(CTRL.allocator.yaw_reaction_limit,B_tau_cmd(3)-yaw_lpf));
[p,d,reaction]=v2_rotors(servo_angle,CTRL.nominal.geometry,CTRL.nominal.propeller);
A1=zeros(4,4); A2=zeros(4,4);
for i=1:4
    moment=cross(p(:,i)-B_c_hat,d(:,i))+reaction(:,i);
    A1(:,i)=[moment(1:2);reaction(3,i);d(3,i)];
end
thrust_cmd=min(max(solve4(A1,[B_tau_cmd(1:2);r;B_F_cmd(3)],CTRL.allocator.regularization),0),CTRL.nominal.propeller.max_thrust);
for i=1:4
    tangent=CTRL.nominal.geometry.tangent(:,i)*thrust_cmd(i);
    moment=cross(p(:,i)-B_c_hat,tangent);
    A2(:,i)=[tangent(1:2);moment(3);CTRL.nominal.geometry.spin(i)*thrust_cmd(i)];
end
sine=solve4(A2,[B_F_cmd(1:2);B_tau_cmd(3)-r;0],CTRL.allocator.regularization);
servo_cmd=asin(min(max(sine,-sin(CTRL.nominal.servo_limit)),sin(CTRL.nominal.servo_limit)));
end
function x=solve4(A,b,lambda)
if rcond(A)>1e-10, x=A\b;
else, x=(A'*A+lambda*eye(4))\(A'*b); end
end
