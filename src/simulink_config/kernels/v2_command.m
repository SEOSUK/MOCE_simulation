function [W_p_ref,euler_ref] = v2_command(t,EXP)
%#codegen
W_p_ref=EXP.command.position(:);
if EXP.command.position_mode==1 && t>EXP.command.position_start_s
    W_p_ref=W_p_ref+EXP.command.position_sine_amplitude(:).* ...
        sin(EXP.command.position_sine_rad_s(:)*t+EXP.command.position_sine_phase(:));
end
euler_ref=EXP.command.attitude(:)+double(EXP.command.attitude_sine_enable(:)).* ...
    EXP.command.attitude_sine_amplitude(:).*sin(EXP.command.attitude_sine_rad_s(:)*t+EXP.command.attitude_sine_phase(:));
end
