function v2_build_controls(m)
% Native PID, conventional DOB, and visible MOCE authoring (R2024b).
position(m); velocity(m); attitude(m); rate(m); moce(m);
end

function position(m)
p=sub(m,'Position Controller',{'W_p_ref','W_p'},{'W_v_ref'},[360 100 535 190]);
sumblock(p,'Position error','+-',[130 40 155 75]);
w(p,'W_p_ref/1','Position error/1');w(p,'W_p/1','Position error/2');
difference(p,'Measured velocity','EXP.initial.position(:)+SENSOR.position.bias(:)',[120 140 270 195]);
w(p,'W_p/1','Measured velocity/1');
pids(p,'Position PID','position',{'X','Y','Z'},[335 30 505 115]);
w(p,'Position error/1','Position PID/1','W_e_p');w(p,'Measured velocity/1','Position PID/2');
difference(p,'Reference velocity','EXP.command.position(:)',[120 260 270 315]);
w(p,'W_p_ref/1','Reference velocity/1');
lowpass(p,'Velocity FF filter','2*pi*CTRL.position.velocity_ff.cutoff_hz','0',[320 255 450 310]);
w(p,'Reference velocity/1','Velocity FF filter/1');
constant(p,'FF enabled','CTRL.position.velocity_ff.enable',[480 325 535 350]);
constant(p,'Zero FF','zeros(3,1)',[480 380 535 405]);
switchblock(p,'Optional velocity FF',[580 260 630 325]);
w(p,'Velocity FF filter/1','Optional velocity FF/1');w(p,'FF enabled/1','Optional velocity FF/2');w(p,'Zero FF/1','Optional velocity FF/3');
sumblock(p,'Add velocity FF','++',[700 50 725 85]);
w(p,'Position PID/1','Add velocity FF/1');w(p,'Optional velocity FF/1','Add velocity FF/2');
saturation(p,'Velocity limits','CTRL.position.output_limit(:)',[775 45 870 90]);
w(p,'Add velocity FF/1','Velocity limits/1');out(p,'W_v_ref','Velocity limits/1',[930 55 960 75]);
note(p,'Native PID: measurement derivative; optional reference-velocity FF is added after PID, then bounded.',[100 -45]);
end

function velocity(m)
p=sub(m,'Velocity Controller',{'W_v_ref','W_v','euler'},{'B_F_cmd'},[660 100 840 210]);
sumblock(p,'Velocity error','+-',[130 40 155 75]);w(p,'W_v_ref/1','Velocity error/1');w(p,'W_v/1','Velocity error/2');
difference(p,'Measured acceleration','EXP.initial.velocity(:)+SENSOR.velocity.bias(:)',[110 150 265 205]);
w(p,'W_v/1','Measured acceleration/1');
pids(p,'Velocity PID','velocity',{'X','Y','Z'},[330 30 495 115]);
w(p,'Velocity error/1','Velocity PID/1','W_e_v');w(p,'Measured acceleration/1','Velocity PID/2');
constant(p,'Gravity compensation','[0;0;double(CTRL.velocity.gravity_comp.enable)*CTRL.nominal.gravity]',[330 240 500 285]);
sumblock(p,'Add gravity','++',[565 50 590 90]);w(p,'Velocity PID/1','Add gravity/1','W_a_cmd');w(p,'Gravity compensation/1','Add gravity/2');
gain(p,'Nominal mass','CTRL.nominal.mass',[640 45 735 95]);w(p,'Add gravity/1','Nominal mass/1');
add_block('simulink/Discontinuities/Saturation',[p '/Nonnegative world Z'],'UpperLimit','[inf;inf;inf]','LowerLimit','[-inf;-inf;0]','Position',[785 45 890 95]);
w(p,'Nominal mass/1','Nonnegative world Z/1');
helper(p,'World-to-Body Rotation',sprintf('function f=rotate(F,euler)\nf=v2_rotation(euler)''*F;\nend'),[950 40 1100 110]);
w(p,'Nonnegative world Z/1','World-to-Body Rotation/1','W_F_cmd');w(p,'euler/1','World-to-Body Rotation/2');
saturation(p,'Body force limits','CTRL.force_limit(:)',[1160 45 1260 95]);w(p,'World-to-Body Rotation/1','Body force limits/1');out(p,'B_F_cmd','Body force limits/1',[1320 55 1350 75]);
note(p,'Acceleration command = Velocity PID only; no acceleration trajectory feed-forward.',[110 -45]);
end

function attitude(m)
p=sub(m,'Attitude Controller',{'euler_ref','euler','B_omega'},{'B_omega_ref'},[360 360 535 470]);
helper(p,'SO3 shortest rotation error',sprintf('function e=error(ref,measured)\ne=v2_rotvec(v2_rotation(measured)''*v2_rotation(ref));\nend'),[155 35 345 110]);
w(p,'euler_ref/1','SO3 shortest rotation error/1');w(p,'euler/1','SO3 shortest rotation error/2');
pids(p,'Attitude PID','attitude',{'Roll','Pitch','Yaw'},[435 35 620 130]);
w(p,'SO3 shortest rotation error/1','Attitude PID/1','B_e_R');w(p,'B_omega/1','Attitude PID/2');out(p,'B_omega_ref','Attitude PID/1',[725 60 755 80]);
note(p,'SO(3) shortest-rotation error; D damping uses measured body rate, not Euler-angle differentiation.',[100 -45]);
end

function rate(m)
p=sub(m,'Rate Controller + DOB',{'B_omega_ref','B_omega'}, ...
    {'B_tau_cmd','B_d_hat','B_tau_pid','B_tau_nom'},[660 360 840 470]);
sumblock(p,'Rate error','+-',[135 40 160 75]);w(p,'B_omega_ref/1','Rate error/1');w(p,'B_omega/1','Rate error/2');
difference(p,'Measured angular acceleration','EXP.initial.body_rate(:)+SENSOR.gyro.bias(:)',[115 155 300 215]);
w(p,'B_omega/1','Measured angular acceleration/1');
pids(p,'Rate PID','rate',{'Roll','Pitch','Yaw'},[360 30 520 120]);
w(p,'Rate error/1','Rate PID/1','B_e_omega');w(p,'Measured angular acceleration/1','Rate PID/2');
helper(p,'Gyroscopic torque',sprintf('function t=gyro(omega,CTRL)\nt=cross(omega,CTRL.nominal.inertia*omega);\nend'),[350 210 525 270],'CTRL');
w(p,'B_omega/1','Gyroscopic torque/1');
constant(p,'Gyro FF enabled','CTRL.rate.gyroscopic_ff.enable',[560 260 625 290]);
constant(p,'Zero gyro FF','zeros(3,1)',[560 320 625 350]);
switchblock(p,'Gyroscopic Feed-Forward',[680 190 725 250]);
w(p,'Gyroscopic torque/1','Gyroscopic Feed-Forward/1');w(p,'Gyro FF enabled/1','Gyroscopic Feed-Forward/2');w(p,'Zero gyro FF/1','Gyroscopic Feed-Forward/3');
sumblock(p,'Nominal torque','++',[790 45 815 80]);w(p,'Rate PID/1','Nominal torque/1');w(p,'Gyroscopic Feed-Forward/1','Nominal torque/2');
% Strictly proper Q and sQ paths allow feedback of the final saturated command.
dob=sub(p,'Conventional DOB',{'B_omega','B_tau_effective'},{'B_d_raw'},[560 465 790 575]);
qfilter(dob,'sQ(s)','CTRL.dob.cutoff_rad_s',true,'EXP.initial.body_rate(:)+SENSOR.gyro.bias(:)',[140 30 280 100]);
qfilter(dob,'Q(s)','CTRL.dob.cutoff_rad_s',false,'zeros(3,1)',[140 180 280 250]);
w(dob,'B_omega/1','sQ(s)/1');w(dob,'B_tau_effective/1','Q(s)/1');
gain(dob,'J nominal','CTRL.nominal.inertia',[350 35 460 85],'Matrix(K*u)');w(dob,'sQ(s)/1','J nominal/1');
sumblock(dob,'Inverse model minus effective torque','+-',[550 65 580 110]);
w(dob,'J nominal/1','Inverse model minus effective torque/1');w(dob,'Q(s)/1','Inverse model minus effective torque/2');
out(dob,'B_d_raw','Inverse model minus effective torque/1',[695 75 725 95]);
note(dob,sprintf('Q(s)=wc^2/(s^2+sqrt(2)*wc*s+wc^2), wc=CTRL.dob.cutoff_rad_s\nB_d_raw = J_nominal*sQ(s)*B_omega - Q(s)*B_tau_effective'),[115 -65]);
w(p,'B_omega/1','Conventional DOB/1');
saturation(p,'Estimate limits','CTRL.dob.estimate_limit(:)',[865 480 980 535]);w(p,'Conventional DOB/1','Estimate limits/1');
add_block('simulink/Discrete/Zero-Order Hold',[p '/Sample estimate'],'SampleTime','CTRL.sample_time','Position',[1020 485 1100 530]);w(p,'Estimate limits/1','Sample estimate/1');
constant(p,'DOB enabled','CTRL.dob.enable',[1040 600 1110 630]);constant(p,'Zero disturbance','zeros(3,1)',[1040 660 1110 690]);
switchblock(p,'Publish estimate',[1170 480 1220 540]);w(p,'Sample estimate/1','Publish estimate/1');w(p,'DOB enabled/1','Publish estimate/2');w(p,'Zero disturbance/1','Publish estimate/3');
constant(p,'Compensation enabled','CTRL.dob.compensate',[1170 330 1245 360]);
switchblock(p,'Optional compensation',[1315 245 1365 305]);w(p,'Publish estimate/1','Optional compensation/1');w(p,'Compensation enabled/1','Optional compensation/2');w(p,'Zero disturbance/1','Optional compensation/3');
sumblock(p,'Subtract disturbance','+-',[1430 45 1455 85]);w(p,'Nominal torque/1','Subtract disturbance/1');w(p,'Optional compensation/1','Subtract disturbance/2');
saturation(p,'Torque saturation','CTRL.torque_limit(:)',[1510 40 1640 90]);w(p,'Subtract disturbance/1','Torque saturation/1');
sumblock(p,'Remove modeled gyro','+-',[360 465 385 510]);w(p,'Torque saturation/1','Remove modeled gyro/1');w(p,'Gyroscopic torque/1','Remove modeled gyro/2');w(p,'Remove modeled gyro/1','Conventional DOB/2','B_tau_effective',true);
out(p,'B_tau_cmd','Torque saturation/1',[1740 55 1770 75]);out(p,'B_d_hat','Publish estimate/1',[1740 495 1770 515]);
out(p,'B_tau_pid','Rate PID/1',[1740 155 1770 175]);out(p,'B_tau_nom','Nominal torque/1',[1740 215 1770 235]);
note(p,sprintf(['J*omega_dot = B_tau_cmd - cross(omega,J*omega) + d. Positive d is additive plant torque.\n' ...
    'B_tau_effective = saturated B_tau_cmd - modeled gyro, regardless of gyro FF switch.\n' ...
    'B_d_hat = enable ? bound(J*sQ*omega - Q*B_tau_effective) : 0.\n' ...
    'B_tau_cmd = saturate(B_tau_nom - (compensate ? B_d_hat : 0)). Strictly proper filters break feedback.']),[120 -115]);
end

function moce(m)
p=sub(m,'CoM Estimator',{'B_F_cmd','B_d_hat'},{'B_c_hat','B_c_filtered'},[950 480 1120 565]);
qfilter(p,'Force Q(s)','CTRL.moce.force_cutoff_rad_s',false,'zeros(3,1)',[120 30 270 105]);w(p,'B_F_cmd/1','Force Q(s)/1');
helper(p,'skew(QF)',sprintf('function S=skew(f)\nS=[0 -f(3) f(2);f(3) 0 -f(1);-f(2) f(1) 0];\nend'),[340 30 465 100]);w(p,'Force Q(s)/1','skew(QF)/1','QF');
gain(p,'J nominal inverse','inv(CTRL.nominal.inertia)',[530 35 670 95],'Matrix(K*u)');w(p,'skew(QF)/1','J nominal inverse/1');
add_block('simulink/Math Operations/Math Function',[p '/Transpose'],'Operator','transpose','Position',[725 35 815 95]);w(p,'J nominal inverse/1','Transpose/1');
add_block('simulink/Math Operations/Product',[p '/Adaptation direction'],'Inputs','**','Multiplication','Matrix(*)','Position',[880 35 970 100]);w(p,'Transpose/1','Adaptation direction/1');w(p,'B_d_hat/1','Adaptation direction/2','B_d_hat_to_MOCE',true);
gain(p,'Gamma','CTRL.moce.gamma(:)',[1030 40 1120 95]);w(p,'Adaptation direction/1','Gamma/1');
saturation(p,'Adaptation rate limits','CTRL.moce.rate_limit(:)',[1170 35 1300 95]);w(p,'Gamma/1','Adaptation rate limits/1');
helper(p,'Force and axis gates',sprintf(['function rate=gates(rate,f,CTRL)\n' ...
    'if norm(f)<CTRL.moce.min_force, rate(:)=0; end\n' ...
    'if ~CTRL.moce.estimate_z || norm(f(1:2))<CTRL.moce.min_horizontal_force, rate(3)=0; end\nend']),[1360 35 1510 115],'CTRL');
w(p,'Adaptation rate limits/1','Force and axis gates/1');w(p,'Force Q(s)/1','Force and axis gates/2');
constant(p,'MOCE enabled','CTRL.moce.enable',[1350 215 1435 250]);constant(p,'Zero adaptation','zeros(3,1)',[1350 280 1435 310]);
switchblock(p,'Enable adaptation',[1570 35 1620 105]);w(p,'Force and axis gates/1','Enable adaptation/1');w(p,'MOCE enabled/1','Enable adaptation/2');w(p,'Zero adaptation/1','Enable adaptation/3');
add_block('simulink/Discrete/Discrete-Time Integrator',[p '/Projected CoM integrator'], ...
    'IntegratorMethod','Integration: Forward Euler','SampleTime','CTRL.sample_time', ...
    'InitialCondition','CTRL.moce.initial_com(:)','LimitOutput','on', ...
    'UpperSaturationLimit','CTRL.moce.offset_limit(:)','LowerSaturationLimit','-CTRL.moce.offset_limit(:)', ...
    'Position',[1680 35 1840 105]);w(p,'Enable adaptation/1','Projected CoM integrator/1');
out(p,'B_c_hat','Projected CoM integrator/1',[2060 50 2090 70]);
% A zero time constant gives unity response, with no division by zero.
lowpass(p,'Diagnostic smoothing','1/max(CTRL.moce.output_tau_s,eps)','CTRL.moce.initial_com(:)',[1870 210 2000 265]);
w(p,'Projected CoM integrator/1','Diagnostic smoothing/1');out(p,'B_c_filtered','Diagnostic smoothing/1',[2060 225 2090 245]);
note(p,sprintf(['c_dot = Gamma*(J_nominal^-1*skew(QF))^T*B_d_hat. QF filters commanded body force.\n' ...
    'Rate bounds + force/Z gates precede a limited discrete integrator (projection).\n' ...
    'Disabled: hold initial CoM. Raw estimate goes to allocator; smoothing is diagnostic only.']),[115 -95]);
end

function pids(parent,name,group,axes,rect)
p=sub(parent,name,{'error','measurement_rate'},{'command'},rect);pr=['CTRL.' group];
add_block('simulink/Signal Routing/Demux',[p '/Axis errors'],'Outputs','3','Position',[125 30 130 205]);
add_block('simulink/Signal Routing/Demux',[p '/Measured rates'],'Outputs','3','Position',[190 285 195 460]);
w(p,'error/1','Axis errors/1');
lowpass(p,'Measurement D filter',['2*pi*' pr '.derivative_cutoff_hz'],'zeros(3,1)',[75 500 240 555]);
w(p,'measurement_rate/1','Measurement D filter/1');w(p,'Measurement D filter/1','Measured rates/1');
add_block('simulink/Signal Routing/Mux',[p '/Axis commands'],'Inputs','3','Position',[555 35 560 325]);
for k=1:3
    b=[axes{k} ' PID'];ix=sprintf('(%d)',k);y=35+120*(k-1);
    add_block('simulink/Discrete/Discrete PID Controller',[p '/' b], ...
        'Controller','PID','Form','Parallel','SampleTime','CTRL.sample_time', ...
        'P',[pr '.Kp' ix],'I',[pr '.Ki' ix],'D',[pr '.Kd' ix], ...
        'UseExternalDerivativeSource','on','UseFilter','off', ...
        'IntegratorMethod','Backward Euler','LimitOutput','on','AntiWindupMode','clamping', ...
        'UpperSaturationLimit',[pr '.output_limit' ix],'LowerSaturationLimit',['-' pr '.output_limit' ix], ...
        'LimitIntegrator','on','UpperIntegratorSaturationLimit',[pr '.integral_limit' ix], ...
        'LowerIntegratorSaturationLimit',['-' pr '.integral_limit' ix],'Position',[325 y 465 y+65]);
    w(p,sprintf('Axis errors/%d',k),[b '/1']);w(p,sprintf('Measured rates/%d',k),[b '/2']);w(p,[b '/1'],sprintf('Axis commands/%d',k));
end
column(p,'Column vector',[620 140 695 180]);w(p,'Axis commands/1','Column vector/1');out(p,'command','Column vector/1',[750 150 780 170]);
note(p,sprintf(['Native Discrete PID, CTRL.%s gains; external ydot supplies negative measurement D.\n' ...
    'Backward-Euler I with clamping and independent integral-contribution limits.\n' ...
    'The visible measurement-rate filter uses alpha=1-exp(-2*pi*fc*Ts).'],group),[95 -95]);
end

function difference(parent,name,ic,rect)
p=sub(parent,name,{'measurement'},{'rate'},rect);
add_block('simulink/Discrete/Unit Delay',[p '/Previous sample'],'SampleTime','CTRL.sample_time','InitialCondition',ic,'Position',[135 150 225 195]);
sumblock(p,'Sample difference','+-',[285 40 310 80]);add_block('simulink/Discrete/Zero-Order Hold',[p '/Sample measurement'],'SampleTime','CTRL.sample_time','Position',[75 40 115 80]);
w(p,'measurement/1','Sample measurement/1');w(p,'Sample measurement/1','Previous sample/1');w(p,'Sample measurement/1','Sample difference/1');w(p,'Previous sample/1','Sample difference/2');
gain(p,'Inverse sample time','1/CTRL.sample_time',[360 35 480 85]);w(p,'Sample difference/1','Inverse sample time/1');out(p,'rate','Inverse sample time/1',[550 50 580 70]);
end

function qfilter(parent,name,wc,derivative,initial,rect)
p=sub(parent,name,{'u'},{'y'},rect);
add_block('simulink/Signal Routing/Demux',[p '/Axes'],'Outputs','3','Position',[120 30 125 255]);
add_block('simulink/Signal Routing/Mux',[p '/Filtered axes'],'Inputs','3','Position',[445 30 450 255]);w(p,'u/1','Axes/1');
for k=1:3
    names={'X','Y','Z'};b=[names{k} ' ' name];y=30+100*(k-1);
    if derivative,C=['[' wc '^2 0]'];else,C=['[0 ' wc '^2]'];end
    % State ordering [x_dot;x]; initial omega equilibrium prevents startup impulse.
    axis=zeros(1,3);axis(k)=1;
    ic=sprintf('[0; [%d %d %d]*(%s)/((%s)^2)]',axis,initial,wc);
    add_block('simulink/Continuous/State-Space',[p '/' b], ...
        'A',['[-sqrt(2)*' wc ' -' wc '^2;1 0]'],'B','[1;0]','C',C,'D','0', ...
        'X0',ic,'Position',[220 y 365 y+60]);
    w(p,sprintf('Axes/%d',k),[b '/1']);w(p,[b '/1'],sprintf('Filtered axes/%d',k));
end
column(p,'Column vector',[510 125 585 170]);w(p,'Filtered axes/1','Column vector/1');out(p,'y','Column vector/1',[645 135 675 155]);
if derivative,eq='sQ(s) = wc^2*s / (s^2 + sqrt(2)*wc*s + wc^2)';else,eq='Q(s) = wc^2 / (s^2 + sqrt(2)*wc*s + wc^2)';end
note(p,[eq '; wc = ' wc],[100 -40]);
end

function lowpass(parent,name,bandwidth,ic,rect)
p=sub(parent,name,{'u'},{'y'},rect);
add_block('simulink/Discrete/Unit Delay',[p '/Previous filtered value'],'SampleTime','CTRL.sample_time','InitialCondition',ic,'Position',[190 175 330 220]);
sumblock(p,'Filter error','+-',[130 40 155 75]);w(p,'u/1','Filter error/1');w(p,'Previous filtered value/1','Filter error/2');
gain(p,'Exponential filter weight',['1-exp(-CTRL.sample_time*(' bandwidth '))'],[220 30 370 90]);w(p,'Filter error/1','Exponential filter weight/1');
sumblock(p,'Update','++',[440 40 465 75]);w(p,'Exponential filter weight/1','Update/1');w(p,'Previous filtered value/1','Update/2');w(p,'Update/1','Previous filtered value/1');out(p,'y','Update/1',[550 45 580 65]);
end
function p=sub(parent,name,ins,outs,rect)
p=[parent '/' name];add_block('built-in/Subsystem',p,'Position',rect,'ContentPreviewEnabled','off');
for k=1:numel(ins),add_block('built-in/Inport',[p '/' ins{k}],'Port',num2str(k),'PortDimensions','[3 1]','Position',[25 40+100*(k-1) 55 60+100*(k-1)]);end
for k=1:numel(outs),add_block('built-in/Outport',[p '/' outs{k}],'Port',num2str(k),'Position',[800 40+100*(k-1) 830 60+100*(k-1)]);end
end
function helper(p,name,script,rect,param)
add_block('simulink/User-Defined Functions/MATLAB Function',[p '/' name],'Position',rect);
r=sfroot;c=r.find('-isa','Stateflow.EMChart','Path',[p '/' name]);c.Script=script;
if nargin>4,d=c.find('-isa','Stateflow.Data','Name',param);d.Scope='Parameter';d.Tunable=false;end
end
function sumblock(p,name,signs,rect)
add_block('simulink/Math Operations/Sum',[p '/' name],'Inputs',signs,'Position',rect);
end
function gain(p,name,value,rect,multiply)
if nargin<5,multiply='Element-wise(K.*u)';end
add_block('simulink/Math Operations/Gain',[p '/' name],'Gain',value,'Multiplication',multiply,'Position',rect);
end
function constant(p,name,value,rect)
add_block('simulink/Sources/Constant',[p '/' name],'Value',value,'Position',rect,'VectorParams1D','off');
end
function saturation(p,name,limit,rect)
add_block('simulink/Discontinuities/Saturation',[p '/' name],'UpperLimit',limit,'LowerLimit',['-' limit],'Position',rect);
end
function switchblock(p,name,rect)
add_block('simulink/Signal Routing/Switch',[p '/' name],'Criteria','u2 ~= 0','Position',rect);
end
function column(p,name,rect)
add_block('simulink/Math Operations/Reshape',[p '/' name],'OutputDimensionality','Column vector (2-D)','Position',rect);
end
function out(p,name,source,rect)
set_param([p '/' name],'Position',rect);w(p,source,[name '/1']);
end
function note(p,s,xy)
a=Simulink.Annotation(p,s);a.Position=xy;a.FontSize=11;
end
function w(p,a,b,name,logging)
if nargin<4,name='';end
if nargin<5,logging=false;end
h=add_line(p,a,b,'autorouting','on');
if ~isempty(name),set_param(h,'Name',name);end
if logging,port=get_param(h,'SrcPortHandle');set_param(port,'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName',name);end
end
