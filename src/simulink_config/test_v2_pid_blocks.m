function test_v2_pid_blocks(source)
% Independent small-signal test catches derivative sign, gain, pole and axes.
m='v2_pid_test';if bdIsLoaded(m),close_system(m,0);end
new_system(m);cleanup=onCleanup(@()close_system(m,0));cfg=init_v2(m);
set_param(m,'SolverType','Fixed-step','Solver','FixedStepDiscrete','FixedStep','CTRL.sample_time','StopTime','.1');
paths={'Position Controller/Position PID','Velocity Controller/Velocity PID', ...
    'Attitude Controller/Attitude PID','Rate Controller + DOB/Rate PID'};
groups={'position','velocity','attitude','rate'};
e=[.003;-.004;.005];v=[.02;-.03;.04];
for k=1:4
    name=['PID' num2str(k)];add_block([source '/' paths{k}],[m '/' name]);
    add_block('simulink/Sources/Constant',[m '/error' num2str(k)],'Value',mat2str(e),'VectorParams1D','off');
    add_block('simulink/Sources/Constant',[m '/rate' num2str(k)],'Value',mat2str(v),'VectorParams1D','off');
    add_block('simulink/Sinks/To Workspace',[m '/result' num2str(k)],'VariableName',['y' num2str(k)],'SaveFormat','Timeseries');
    add_line(m,['error' num2str(k) '/1'],[name '/1']);add_line(m,['rate' num2str(k) '/1'],[name '/2']);
    add_line(m,[name '/1'],['result' num2str(k) '/1']);
end
out=sim(m);
for k=1:4
    signal=out.get(['y' num2str(k)]);y=reshape(signal.Data,3,[]);p=cfg.CTRL.(groups{k});
    % Backward-Euler I includes the current sample in its first output.
    t=signal.Time(:)';pole=exp(-2*pi*p.derivative_cutoff_hz*cfg.CTRL.sample_time);
    d=v.*(1-pole.^(round(t/cfg.CTRL.sample_time)+1));
    expected=p.Kp(:).*e+p.Ki(:).*e.*(t+cfg.CTRL.sample_time)-p.Kd(:).*d;
    assert(max(abs(y-expected),[],'all')<1e-10,'Native %s PID does not match P/I/measurement-D equation.',groups{k});
end
fprintf('PASS four native PID banks: P/I/D gains, measurement sign, filter pole, and axis order\n');
end
