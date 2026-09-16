function R = v2_rotation(e)
%#codegen
% R_WB = Rz(yaw)*Ry(pitch)*Rx(roll).
c=cos(e); s=sin(e);
R=[c(3)*c(2), c(3)*s(2)*s(1)-s(3)*c(1), c(3)*s(2)*c(1)+s(3)*s(1); ...
   s(3)*c(2), s(3)*s(2)*s(1)+c(3)*c(1), s(3)*s(2)*c(1)-c(3)*s(1); ...
   -s(2), c(2)*s(1), c(2)*c(1)];
end
