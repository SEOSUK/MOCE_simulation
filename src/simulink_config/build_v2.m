function build_v2()
% Reproducible model authoring using supported Simulink/Stateflow APIs only.
here=fileparts(mfilename('fullpath')); src=fileparts(here);
addpath(here,fullfile(here,'kernels'));
load_system('simulink');
if bdIsLoaded('V2'), close_system('V2',0); end
new_system('V2'); m='V2';
init_v2(m);
callback=['addpath(fullfile(fileparts(get_param(bdroot,''FileName'')),''simulink_config'')); ' ...
    'v2_initialize_model(bdroot);'];
set_param(m,'PostLoadFcn',callback,'InitFcn',callback, ...
    'SolverType','Fixed-step','Solver','ode4','FixedStep','EXP.physics_step', ...
    'StopTime','EXP.stop_time','SignalLogging','on','SignalLoggingName','logsout', ...
    'ReturnWorkspaceOutputs','on','AlgebraicLoopMsg','error', ...
    'SaveTime','on','SaveOutput','off');
% Two aligned control lanes.
algorithm(m,'Command Generator','v2_command',{'t'},{'W_p_ref','euler_ref'},'EXP',[50 115 235 405],false);
p=[m '/Command Generator']; delete_line(p,'t/1','Algorithm/1'); delete_block([p '/t']);
add_block('simulink/Sources/Clock',[p '/Experiment time'],'Position',[25 50 55 80]);
wire(p,'Experiment time/1','Algorithm/1','');
v2_build_controls(m);
algorithm(m,'Control Allocator','v2_allocator',{'B_F_cmd','B_tau_cmd','B_c_hat','servo_angle'}, ...
    {'thrust_cmd','servo_cmd'},'CTRL',[1210 115 1380 480],true);
actuator(m,'Propeller Dynamics','propeller',[1485 145 1660 230]);
actuator(m,'Servo Dynamics','servo',[1485 370 1660 455]);
plant(m,[1770 135 1960 470]);
measurement(m,[2070 255 2255 360]);
subsystem(m,'Measured Feedback',{'MeasuredStateBus'},{'W_p','W_v','euler','B_omega'},[390 680 615 810]);
p=[m '/Measured Feedback'];
add_block('simulink/Signal Routing/Bus Selector',[p '/Feedback states'], ...
    'OutputSignals','W_p,W_v,euler,B_omega','Position',[110 40 115 240]);
wire(p,'MeasuredStateBus/1','Feedback states/1','');
for k=1:4
    names={'W_p','W_v','euler','B_omega'};
    wire(p,sprintf('Feedback states/%d',k),[names{k} '/1'],'');
end
set_param(p,'Orientation','left');

% Explicit control interfaces.
wire(m,'Command Generator/1','Position Controller/1','W_p_ref',true);
wire(m,'Command Generator/2','Attitude Controller/1','euler_ref',true);
wire(m,'Position Controller/1','Velocity Controller/1','W_v_ref',true);
wire(m,'Velocity Controller/1','Control Allocator/1','B_F_cmd',true);
wire(m,'Velocity Controller/1','CoM Estimator/1','');
wire(m,'Attitude Controller/1','Rate Controller + DOB/1','B_omega_ref',true);
wire(m,'Rate Controller + DOB/1','Control Allocator/2','B_tau_cmd',true);
wire(m,'Rate Controller + DOB/2','CoM Estimator/2','B_d_hat',true);
wire(m,'CoM Estimator/1','Control Allocator/3','B_c_hat',true);
wire(m,'Control Allocator/1','Propeller Dynamics/1','thrust_cmd',true);
wire(m,'Control Allocator/2','Servo Dynamics/1','servo_cmd',true);
wire(m,'Propeller Dynamics/1','Rigid-Body Plant/1','thrust',true);
wire(m,'Servo Dynamics/1','Rigid-Body Plant/2','servo_angle',true);
wire(m,'Servo Dynamics/1','Control Allocator/4','');
wire(m,'Rigid-Body Plant/1','Measurement Model/1','TrueStateBus');

% Single measured bus trunk below both control lanes.
sp=get_param([m '/Measurement Model'],'PortHandles'); dp=get_param([m '/Measured Feedback'],'PortHandles');
a=get_param(sp.Outport(1),'Position'); b=get_param(dp.Inport(1),'Position');
h=add_line(m,[a;a+[45 0];a(1)+45 855;b(1)+80 855;b(1)+80 b(2);b]);
set_param(h,'Name','MeasuredStateBus');
wire(m,'Measured Feedback/1','Position Controller/2','W_p',true);
wire(m,'Measured Feedback/2','Velocity Controller/2','W_v',true);
wire(m,'Measured Feedback/3','Velocity Controller/3','euler',true);
wire(m,'Measured Feedback/3','Attitude Controller/2','');
wire(m,'Measured Feedback/4','Attitude Controller/3','B_omega',true);
wire(m,'Measured Feedback/4','Rate Controller + DOB/2','');
v2_build_monitoring(m);
% Semantic colors and headings.
set_param([m '/Command Generator'],'BackgroundColor','lightBlue');
for n={'Position Controller','Velocity Controller','Attitude Controller','Rate Controller + DOB'}
    set_param([m '/' n{1}],'BackgroundColor','lightBlue');
end
set_param([m '/CoM Estimator'],'BackgroundColor','yellow');
set_param([m '/Control Allocator'],'BackgroundColor','orange');
for n={'Propeller Dynamics','Servo Dynamics','Rigid-Body Plant'},set_param([m '/' n{1}],'BackgroundColor','[0.86 0.94 0.86]');end
note=Simulink.Annotation(m,sprintf(['V2 | Cascaded control and torque-domain disturbance estimation\n' ...
    'World Z-up; body FLU. R_WB = Rz(yaw) Ry(pitch) Rx(roll). Euler angles in rad.\n' ...
    'V1 FRD converted by diag([1 -1 -1]); actuator order and positive servo sense retained.\n' ...
    'SI: m, m/s, rad/s, N, N m. Parameters reload from simulink_config on initialization.']));
note.Position=[50 -75]; note.FontSize=12;
style_v2(m);
set_param(m,'ZoomFactor','FitSystem');
save_system(m,fullfile(src,'V2.slx'));
set_param(m,'SimulationCommand','update');
save_system(m);
fprintf('Built and compiled %s\n',fullfile(src,'V2.slx'));
end

function subsystem(parent,name,inputs,outputs,position)
p=[parent '/' name];
add_block('built-in/Subsystem',p,'Position',position,'ContentPreviewEnabled','off');
for k=1:numel(inputs)
    add_block('built-in/Inport',[p '/' inputs{k}],'Port',num2str(k), ...
        'Position',[25 40+60*(k-1) 55 60+60*(k-1)]);
end
for k=1:numel(outputs)
    add_block('built-in/Outport',[p '/' outputs{k}],'Port',num2str(k), ...
        'Position',[460 40+60*(k-1) 490 60+60*(k-1)]);
end
end

function algorithm(parent,name,file,inputs,outputs,param,position,discrete,nout)
if nargin<9, nout=numel(outputs); end
subsystem(parent,name,inputs,outputs(1:nout),position); p=[parent '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function',[p '/Algorithm'], ...
    'Position',[180 40 340 max(120,60*numel(outputs))]);
root=sfroot; chart=root.find('-isa','Stateflow.EMChart','Path',[p '/Algorithm']);
chart.Script=fileread(fullfile(fileparts(mfilename('fullpath')),'kernels',[file '.m']));
if ~isempty(param)
    data=chart.find('-isa','Stateflow.Data','Name',param);
    if isempty(data),data=Stateflow.Data(chart);data.Name=param;end
    data.Scope='Parameter'; data.Tunable=false;
end
if discrete,chart.ChartUpdate='DISCRETE';chart.SampleTime='CTRL.sample_time';end
for k=1:numel(inputs)
    wire(p,[inputs{k} '/1'],sprintf('Algorithm/%d',k),'');
    if ~strcmp(inputs{k},'t')
        dim=3;
        if any(strcmp(inputs{k},{'thrust','servo_angle','thrust_cmd','servo_cmd'})),dim=4;end
        set_param([p '/' inputs{k}],'PortDimensions',sprintf('[%d 1]',dim));
    end
end
for k=1:numel(outputs)
    if k<=nout, dest=[outputs{k} '/1'];
    else
        add_block('built-in/Terminator',[p '/' outputs{k} ' monitor'],'Position',[460 40+60*(k-1) 480 60+60*(k-1)]);
        dest=[outputs{k} ' monitor/1'];
    end
    wire(p,sprintf('Algorithm/%d',k),dest,outputs{k},k>nout);
end
end

function actuator(m,name,kind,position)
isMotor=strcmp(kind,'propeller');
if isMotor, in='thrust_cmd';out='thrust';else,in='servo_cmd';out='servo_angle';end
subsystem(m,name,{in},{out},position);p=[m '/' name];
if isMotor,upper='PLANT.propeller.max_thrust';lower='0';
else,upper='PLANT.servo.limit_rad';lower='-PLANT.servo.limit_rad';end
set_param([p '/' in],'PortDimensions','[4 1]');
add_block('simulink/Discontinuities/Saturation',[p '/Command limits'],'UpperLimit',upper,'LowerLimit',lower,'Position',[90 35 130 65]);
wire(p,[in '/1'],'Command limits/1','');
add_block('simulink/Continuous/Transport Delay',[p '/Transport delay'],'TransDelayFeedthrough','on','DelayTime',['PLANT.' kind '.delay_s'],'Position',[160 35 220 65]);
if isMotor
    algorithm(p,'Motor effectiveness','v2_motor_effectiveness',{'thrust_cmd','t'}, ...
        {'effective_thrust','eta_common'},'PLANT',[145 155 320 235],true,1);
    e=[p '/Motor effectiveness'];
    chart=sfroot;chart=chart.find('-isa','Stateflow.EMChart','Path',[e '/Algorithm']);chart.SampleTime='PLANT.propeller.sample_time';
    delete_line(e,'t/1','Algorithm/2');delete_block([e '/t']);
    add_block('simulink/Sources/Clock',[e '/Motor time'],'Position',[25 115 55 145]);
    wire(e,'Motor time/1','Algorithm/2','');
    wire(p,'Command limits/1','Motor effectiveness/1','');
    wire(p,'Motor effectiveness/1','Transport delay/1','');
else
    wire(p,'Command limits/1','Transport delay/1','');
end
% Vector continuous first-order lag: integrator states keep V1 initial output.
add_block('simulink/Math Operations/Sum',[p '/Lag error'],'Inputs','+-','Position',[265 35 285 65]);
add_block('simulink/Math Operations/Gain',[p '/Lag bandwidth'],'Gain',['1/PLANT.' kind '.time_constant_s'],'Position',[320 30 385 70]);
if isMotor,ic='PLANT.propeller.initial_thrust';else,ic='PLANT.servo.initial_angle';end
add_block('simulink/Continuous/Integrator',[p '/Actuator state'],'InitialCondition',ic,'Position',[420 35 450 65]);
wire(p,'Transport delay/1','Lag error/1','');wire(p,'Actuator state/1','Lag error/2','');
wire(p,'Lag error/1','Lag bandwidth/1','');wire(p,'Lag bandwidth/1','Actuator state/1','');
if isMotor
    from='Actuator state/1';
else
    add_block('simulink/Math Operations/Gain',[p '/Static effectiveness'],'Gain','PLANT.servo.dc_gain','Position',[495 30 570 70]);
    wire(p,'Actuator state/1','Static effectiveness/1','');
    add_block('simulink/Discontinuities/Saturation',[p '/Physical travel limits'],'UpperLimit',upper,'LowerLimit',lower,'Position',[610 35 650 65]);
    wire(p,'Static effectiveness/1','Physical travel limits/1','');from='Physical travel limits/1';
end
set_param([p '/' out],'Position',[710 40 740 60]);wire(p,from,[out '/1'],'');
end

function plant(m,position)
subsystem(m,'Rigid-Body Plant',{'thrust','servo_angle'},{'TrueStateBus'},position);p=[m '/Rigid-Body Plant'];
algorithm(p,'Actuator Wrench Model','v2_wrench',{'thrust','servo_angle'},{'B_F_actual','B_tau_actual'},'PLANT',[120 75 300 210],false);
subsystem(p,'Rotational Dynamics',{'B_tau_actual'},{'B_omega'},[390 185 580 270]);r=[p '/Rotational Dynamics'];
algorithm(r,'Euler rigid body equation','v2_rotation_dynamics',{'B_tau_actual','B_omega'},{'B_omega_dot'},'PLANT',[110 40 285 120],false);
add_block('simulink/Continuous/Integrator',[r '/Body rate'],'InitialCondition','EXP.initial.body_rate(:)','Position',[340 45 370 75]);
wire(r,'B_tau_actual/1','Euler rigid body equation/1','');wire(r,'Euler rigid body equation/1','Body rate/1','');
wire(r,'Body rate/1','Euler rigid body equation/2','');wire(r,'Body rate/1','B_omega/1','');
subsystem(p,'Kinematics',{'B_omega'},{'euler'},[670 185 830 270]);r=[p '/Kinematics'];
algorithm(r,'Euler angle rates','v2_kinematics',{'euler','B_omega'},{'euler_dot'},'',[110 40 285 120],false);
add_block('simulink/Continuous/Integrator',[r '/Euler attitude'],'InitialCondition','EXP.initial.attitude(:)','Position',[340 45 370 75]);
wire(r,'B_omega/1','Euler angle rates/2','');wire(r,'Euler angle rates/1','Euler attitude/1','');
wire(r,'Euler attitude/1','Euler angle rates/1','');wire(r,'Euler attitude/1','euler/1','');
subsystem(p,'Translational Dynamics',{'B_F_actual','euler'},{'W_p','W_v'},[660 30 840 120]);r=[p '/Translational Dynamics'];
algorithm(r,'Newton equation','v2_translation_dynamics',{'B_F_actual','euler'},{'W_a'},'PLANT',[110 40 285 120],false);
add_block('simulink/Continuous/Integrator',[r '/World velocity'],'InitialCondition','PLANT.derived.W_v_C0','Position',[335 40 365 70]);
add_block('simulink/Continuous/Integrator',[r '/World position'],'InitialCondition','PLANT.derived.W_p_C0','Position',[405 40 435 70]);
set_param([r '/W_p'],'Position',[510 45 540 65]);set_param([r '/W_v'],'Position',[510 115 540 135]);
wire(r,'B_F_actual/1','Newton equation/1','');wire(r,'euler/1','Newton equation/2','');
wire(r,'Newton equation/1','World velocity/1','');wire(r,'World velocity/1','World position/1','');
wire(r,'World velocity/1','W_v/1','');wire(r,'World position/1','W_p/1','');
wire(p,'thrust/1','Actuator Wrench Model/1','');wire(p,'servo_angle/1','Actuator Wrench Model/2','');
wire(p,'Actuator Wrench Model/1','Translational Dynamics/1','B_F_actual',true);
wire(p,'Actuator Wrench Model/2','Rotational Dynamics/1','B_tau_actual',true);
wire(p,'Rotational Dynamics/1','Kinematics/1','B_omega_true',true);
wire(p,'Kinematics/1','Translational Dynamics/2','euler_true',true);
algorithm(p,'Body origin feedback','v2_body_origin',{'W_p_C','W_v_C','euler','B_omega'}, ...
    {'W_p','W_v'},'PLANT',[660 375 840 495],false);
wire(p,'Translational Dynamics/1','Body origin feedback/1','W_p_C');
wire(p,'Translational Dynamics/2','Body origin feedback/2','W_v_C');
wire(p,'Kinematics/1','Body origin feedback/3','');
wire(p,'Rotational Dynamics/1','Body origin feedback/4','');
names={'W_p','W_v','euler','B_omega','B_F_actual','B_tau_actual'};
origins={'Body origin feedback/1','Body origin feedback/2','Kinematics/1','Rotational Dynamics/1','Actuator Wrench Model/1','Actuator Wrench Model/2'};
add_block('simulink/Signal Routing/Bus Creator',[p '/True state assembly'],'Inputs','6','Position',[980 20 990 300]);
for k=1:6
    % Signal Conversion permits bus element naming without renaming upstream logs.
    b=['State ' names{k}];add_block('simulink/Signal Attributes/Signal Conversion',[p '/' b],'Position',[900 25+45*(k-1) 925 45+45*(k-1)]);
    wire(p,origins{k},[b '/1'],'');wire(p,[b '/1'],sprintf('True state assembly/%d',k),names{k});
end
set_param([p '/TrueStateBus'],'Position',[1070 145 1100 165]);wire(p,'True state assembly/1','TrueStateBus/1','TrueStateBus');
end

function measurement(m,position)
subsystem(m,'Measurement Model',{'TrueStateBus'},{'MeasuredStateBus'},position);p=[m '/Measurement Model'];
add_block('simulink/Signal Routing/Bus Selector',[p '/Physical state'],'OutputSignals','W_p,W_v,euler,B_omega','Position',[100 30 105 330]);
wire(p,'TrueStateBus/1','Physical state/1','');
names={'W_p','W_v','euler','B_omega'};groups={'position','velocity','attitude','gyro'};
add_block('simulink/Signal Routing/Bus Creator',[p '/Measured state assembly'],'Inputs','4','Position',[690 30 695 330]);
for k=1:4
    y=40+100*(k-1);g=groups{k};
    add_block('simulink/Discrete/Zero-Order Hold',[p '/' g ' sample'],'SampleTime','CTRL.sample_time','Position',[160 y 205 y+30]);
    add_block('simulink/Sources/Random Number',[p '/' g ' noise'],'Mean','[0;0;0]','Variance', ...
        ['double(SENSOR.' g '.enable)*SENSOR.' g '.noise_std(:).^2'],'Seed',sprintf('EXP.seed+%d',101*k), ...
        'SampleTime','CTRL.sample_time','Position',[255 y+40 310 y+65]);
    add_block('simulink/Sources/Constant',[p '/' g ' bias'],'Value',['SENSOR.' g '.bias(:)'],'Position',[340 y+50 390 y+75]);
    add_block('simulink/Math Operations/Sum',[p '/' g ' measurement'],'Inputs','+++','Position',[440 y 460 y+30]);
    add_block('simulink/Continuous/Transport Delay',[p '/' g ' delay'],'TransDelayFeedthrough','on','DelayTime','SENSOR.delay_s','Position',[535 y 590 y+30]);
    wire(p,sprintf('Physical state/%d',k),[g ' sample/1'],'');
    wire(p,[g ' sample/1'],[g ' measurement/1'],'');wire(p,[g ' noise/1'],[g ' measurement/2'],'');
    wire(p,[g ' bias/1'],[g ' measurement/3'],'');wire(p,[g ' measurement/1'],[g ' delay/1'],'');
    wire(p,[g ' delay/1'],sprintf('Measured state assembly/%d',k),names{k});
end
set_param([p '/MeasuredStateBus'],'Position',[755 175 785 195]);wire(p,'Measured state assembly/1','MeasuredStateBus/1','MeasuredStateBus');
end

function h=wire(parent,from,to,name,logging)
if nargin<4,name='';end
if nargin<5,logging=false;end
h=add_line(parent,from,to,'autorouting','on');
if ~isempty(name),set_param(h,'Name',name);end
if logging
    port=get_param(h,'SrcPortHandle');
    set_param(port,'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName',name);
end
end
