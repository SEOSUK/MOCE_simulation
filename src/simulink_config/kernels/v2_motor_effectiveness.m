function [effective_thrust,eta_common] = v2_motor_effectiveness(thrust_cmd,t,PLANT)
%#codegen
% MuJoCo setInput order: saturation -> effectiveness -> delay -> motor lag.
persistent started start_time
if isempty(started), started=false; start_time=0; end
nominal=min(max(thrust_cmd,0),PLANT.propeller.max_thrust);
if ~started && max(nominal)>1e-9, started=true; start_time=t; end
elapsed=0;
if started, elapsed=max(0,t-start_time); end
p=PLANT.propeller.effectiveness;
eta_common=1;
if p.enable
    eta_common=min(max(p.initial+p.slope_per_s*elapsed,p.min),p.max);
end
effective_thrust=eta_common*PLANT.propeller.relative_scale(:).*nominal;
end
