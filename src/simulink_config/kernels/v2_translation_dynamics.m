function W_a = v2_translation_dynamics(B_F_actual,euler,PLANT)
%#codegen
W_a=v2_rotation(euler)*B_F_actual/PLANT.derived.mass-[0;0;PLANT.gravity];
end
