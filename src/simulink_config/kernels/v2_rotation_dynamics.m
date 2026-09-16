function B_omega_dot = v2_rotation_dynamics(B_tau_actual,B_omega,PLANT)
%#codegen
J=PLANT.derived.inertia;
B_omega_dot=J\(B_tau_actual-cross(B_omega,J*B_omega));
end
