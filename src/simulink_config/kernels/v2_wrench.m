function [B_F_actual,B_tau_actual] = v2_wrench(thrust,servo_angle,PLANT)
%#codegen
[p,d,reaction]=v2_rotors(servo_angle,PLANT.geometry,PLANT.propeller);
B_F_actual=zeros(3,1); B_tau_actual=zeros(3,1);
for i=1:4
    f=thrust(i)*d(:,i);
    B_F_actual=B_F_actual+f;
    B_tau_actual=B_tau_actual+cross(p(:,i)-PLANT.derived.com,f)+thrust(i)*reaction(:,i);
end
end
