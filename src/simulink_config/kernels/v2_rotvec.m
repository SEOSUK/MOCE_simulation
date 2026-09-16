function v = v2_rotvec(R)
%#codegen
% Quaternion from largest diagonal branch: stable at zero AND pi rotation.
q=zeros(4,1); tr=trace(R);
if tr>0
    s=2*sqrt(max(0,1+tr));
    q=[s/4;(R(3,2)-R(2,3))/s;(R(1,3)-R(3,1))/s;(R(2,1)-R(1,2))/s];
elseif R(1,1)>=R(2,2) && R(1,1)>=R(3,3)
    s=2*sqrt(max(0,1+R(1,1)-R(2,2)-R(3,3)));
    q=[(R(3,2)-R(2,3))/s;s/4;(R(1,2)+R(2,1))/s;(R(1,3)+R(3,1))/s];
elseif R(2,2)>=R(3,3)
    s=2*sqrt(max(0,1+R(2,2)-R(1,1)-R(3,3)));
    q=[(R(1,3)-R(3,1))/s;(R(1,2)+R(2,1))/s;s/4;(R(2,3)+R(3,2))/s];
else
    s=2*sqrt(max(0,1+R(3,3)-R(1,1)-R(2,2)));
    q=[(R(2,1)-R(1,2))/s;(R(1,3)+R(3,1))/s;(R(2,3)+R(3,2))/s;s/4];
end
if q(1)<0, q=-q; end
n=norm(q(2:4));
if n<1e-10, v=2*q(2:4); else, v=(2*atan2(n,q(1))/n)*q(2:4); end
end
