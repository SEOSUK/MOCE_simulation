function [positions,directions,reaction] = v2_rotors(theta,g,p)
%#codegen
positions=[g.arm_length*g.radial_xy;g.rotor_z*ones(1,4)];
directions=zeros(3,4); reaction=zeros(3,4);
for i=1:4
    directions(:,i)=g.tangent(:,i)*sin(theta(i))+[0;0;cos(theta(i))];
    reaction(:,i)=g.spin(i)*[p.b_over_k*directions(1:2,i);p.reaction_torque_ratio*directions(3,i)];
end
end
