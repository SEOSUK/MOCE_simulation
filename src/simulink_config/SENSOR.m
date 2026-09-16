function s = SENSOR()
% Noise is sampled at the controller rate. std and bias use SI units.
s.position = group([.00019 .00022 .00017]);
s.velocity = group([.00473 .005 .00368]);
s.attitude = group([.00071 .00067 .00044]);
s.gyro = group([.0236 .0215 .0174]);
s.delay_s = 0;
end
function g = group(sd)
g = struct('enable',false,'noise_std',sd,'bias',[0 0 0]);
end
