function [x,y,dy] = v2_q(x,u,w,dt)
%#codegen
% Exact ZOH discretization of wc^2/(s^2+sqrt(2)*wc*s+wc^2).
a=w/sqrt(2); s=sin(a*dt); c=cos(a*dt);
transition=exp(-a*dt)*[c-s,-w*w*s/a;s/a,c+s];
equilibrium=[zeros(1,3);u(:)'/(w*w)];
x=equilibrium+transition*(x-equilibrium);
y=w*w*x(2,:)'; dy=w*w*x(1,:)';
end
