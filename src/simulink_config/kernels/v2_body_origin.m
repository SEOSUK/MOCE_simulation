function [W_p,W_v] = v2_body_origin(W_p_C,W_v_C,euler,B_omega,PLANT)
%#codegen
% Newton's equation is integrated at C; controller feedback is fixed origin O.
R_WB=v2_rotation(euler); B_c_true=PLANT.derived.com;
W_p=W_p_C-R_WB*B_c_true;
W_v=W_v_C-R_WB*cross(B_omega,B_c_true);
end
