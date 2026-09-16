function euler_dot = v2_kinematics(euler,B_omega)
%#codegen
% V1 Euler kinematics; Euler representation is singular at pitch +/-pi/2.
s=sin(euler(1)); c=cos(euler(1)); ct=cos(euler(2));
assert(abs(ct)>1e-6,'Euler pitch singularity');
euler_dot=[1 s*tan(euler(2)) c*tan(euler(2));0 c -s;0 s/ct c/ct]*B_omega;
end
