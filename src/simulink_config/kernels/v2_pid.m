function [y,integral,derivative] = v2_pid(error,measurement_rate,integral,derivative,p,dt,ff)
%#codegen
% C++ Pid semantics: integral is the I contribution (includes Ki).
a=1-exp(-2*pi*p.derivative_cutoff_hz*dt);
derivative=derivative+a*(measurement_rate-derivative);
candidate=v2_bound(integral+p.Ki(:).*error*dt,p.integral_limit);
pd=p.Kp(:).*error-p.Kd(:).*derivative+ff;
for i=1:3
    if abs(pd(i)+candidate(i))<=p.output_limit(i) || error(i)*(pd(i)+candidate(i))<=0
        integral(i)=candidate(i);
    end
end
y=v2_bound(pd+integral,p.output_limit);
end
