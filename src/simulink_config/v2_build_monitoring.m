function v2_build_monitoring(m)
% One-way diagnostic buses keep Scope blocks out of the control lanes.
names={'W_p_ref','euler_ref','W_v_ref','B_omega_ref','B_F_cmd','B_tau_pid','B_tau_nom', ...
    'B_d_hat','B_tau_cmd','B_c_hat','B_c_filtered','thrust_cmd','thrust','servo_cmd','servo_angle'};
sources={'Command Generator/1','Command Generator/2','Position Controller/1','Attitude Controller/1', ...
    'Velocity Controller/1','Rate Controller + DOB/3','Rate Controller + DOB/4', ...
    'Rate Controller + DOB/2','Rate Controller + DOB/1','CoM Estimator/1','CoM Estimator/2', ...
    'Control Allocator/1','Propeller Dynamics/1','Control Allocator/2','Servo Dynamics/1'};
add_block('simulink/Signal Routing/Bus Creator',[m '/Control diagnostics'], ...
    'Inputs',num2str(numel(names)),'Position',[2390 80 2400 755]);
for k=1:numel(names)
    h=add_line(m,sources{k},sprintf('Control diagnostics/%d',k),'autorouting','on');
    set_param(h,'Name',names{k});port=get_param(h,'SrcPortHandle');
    set_param(port,'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName',names{k});
end
p=[m '/Monitoring'];add_block('built-in/Subsystem',p,'Position',[2510 355 2715 515],'ContentPreviewEnabled','off');
buses={'ControlDiagnosticsBus','TrueStateBus','MeasuredStateBus'};
selections={strjoin(names,','),'B_F_actual,B_tau_actual','W_p,W_v,euler,B_omega'};
for k=1:3
    add_block('built-in/Inport',[p '/' buses{k}],'Port',num2str(k),'Position',[25 50+280*(k-1) 55 70+280*(k-1)]);
    add_block('simulink/Signal Routing/Bus Selector',[p '/' buses{k} ' signals'], ...
        'OutputSignals',selections{k},'Position',[140 35+280*(k-1) 145 255+280*(k-1)]);
    add_line(p,[buses{k} '/1'],[buses{k} ' signals/1'],'autorouting','on');
end
add_line(m,'Control diagnostics/1','Monitoring/1','autorouting','on');
add_line(m,'Rigid-Body Plant/1','Monitoring/2','autorouting','on');
add_line(m,'Measurement Model/1','Monitoring/3','autorouting','on');
add_block('simulink/Sources/Constant',[p '/True combined CoM'],'Value','PLANT.derived.com(:)', ...
    'VectorParams1D','off','Position',[250 895 385 935]);
groups={'Position Tracking',{'W_p_ref','W_p'}; 'Velocity Tracking',{'W_v_ref','W_v'}; ...
    'Attitude Tracking',{'euler_ref','euler'}; 'Rate Tracking',{'B_omega_ref','B_omega'}; ...
    'Body Force',{'B_F_cmd','B_F_actual'}; ...
    'Torque and DOB',{'B_tau_pid','B_tau_nom','B_d_hat','B_tau_cmd','B_tau_actual'}; ...
    'CoM Estimation',{'B_c_hat','B_c_true','B_c_filtered'}; ...
    'Propeller Commands',{'thrust_cmd','thrust'}; 'Servo Commands',{'servo_cmd','servo_angle'}};
allnames=[names {'B_F_actual','B_tau_actual','W_p','W_v','euler','B_omega','B_c_true'}];
origins=cell(size(allnames));
for k=1:numel(names),origins{k}=sprintf('ControlDiagnosticsBus signals/%d',k);end
origins(end-6:end)={'TrueStateBus signals/1','TrueStateBus signals/2','MeasuredStateBus signals/1', ...
    'MeasuredStateBus signals/2','MeasuredStateBus signals/3','MeasuredStateBus signals/4','True combined CoM/1'};
for k=1:size(groups,1)
    n=groups{k,1}; signals=groups{k,2};y=40+125*(k-1);
    add_block('simulink/Sinks/Scope',[p '/' n],'NumInputPorts',num2str(numel(signals)), ...
        'Position',[780 y 840 y+65]);
    config=get_param([p '/' n],'ScopeConfiguration');config.ShowLegend=true;
    config.OpenAtSimulationStart=false;config.TimeSpan='10';
    config.Title=n;config.AxesScaling='Auto';config.ShowGrid=true;
    units={'Position (m)','Velocity (m/s)','Angle (rad)','Rate (rad/s)','Force (N)', ...
        'Torque (N m)','Offset (m)','Thrust (N)','Angle (rad)'};
    config.YLabel=units{k};
    for j=1:numel(signals)
        idx=find(strcmp(allnames,signals{j}));
        h=add_line(p,origins{idx},sprintf('%s/%d',n,j),'autorouting','on');
        if strcmp(signals{j},'B_c_true')
            set_param(h,'Name',signals{j});
            port=get_param(h,'SrcPortHandle');set_param(port,'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','B_c_true');
        end
    end
end
a=Simulink.Annotation(p,sprintf(['Scopes are diagnostic sinks only. Matching references and measurements share each Scope.\n' ...
    'Position: m; velocity: m/s; attitude: rad; rate: rad/s; force: N; torque: N m; CoM: m.\n' ...
    'Propeller thrust: N; servo angle: rad. All requested signals also remain in logsout.']));
a.Position=[80 -90];a.FontSize=11;
end
