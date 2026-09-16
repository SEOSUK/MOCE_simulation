function style_v2(model)
% Plain subsystem icons: no miniature implementation diagrams at top level.
blocks=find_system(model,'SearchDepth',1,'BlockType','SubSystem');
for k=1:numel(blocks)
    b=blocks{k}; name=get_param(b,'Name');
    switch name
        case 'Command Generator', title='Command\nGenerator';
        case 'Position Controller', title='Position\nController';
        case 'Velocity Controller', title='Velocity\nController';
        case 'Attitude Controller', title='Attitude\nController';
        case 'Rate Controller + DOB', title='Rate PID\n+ DOB';
        case 'CoM Estimator', title='CoM\nEstimator';
        case 'Control Allocator', title='Control\nAllocator';
        case 'Propeller Dynamics', title='Propeller\nDynamics';
        case 'Servo Dynamics', title='Servo\nDynamics';
        case 'Rigid-Body Plant', title='Rigid-Body\nPlant';
        case 'Measurement Model', title='Measurement\nModel';
        case 'Measured Feedback', title='Measured\nFeedback';
        otherwise, title=name;
    end
    display=sprintf('disp(sprintf(''%s''));\n',title);
    % Output names are already printed on the explicit outgoing signal lines.
    for direction={'Inport'}
        ports=find_system(b,'SearchDepth',1,'BlockType',direction{1});
        for j=1:numel(ports)
            if strcmp(direction{1},'Inport'),side='input';else,side='output';end
            label=get_param(ports{j},'Name');
            if endsWith(label,'StateBus')
                if ~strcmp(name,'Monitoring'),continue;end
                if strcmp(label,'TrueStateBus'),label='true state';else,label='measured state';end
            end
            display=[display sprintf('port_label(''%s'',%s,''%s'');\n', ...
                side,get_param(ports{j},'Port'),label)]; %#ok<AGROW>
        end
    end
    set_param(b,'Mask','on','MaskDisplay',display,'MaskIconOpaque','opaque', ...
        'MaskIconFrame','on','ShowName','off','FontSize','11');
    if contains(name,'Controller') || strcmp(name,'Command Generator')
        set_param(b,'BackgroundColor','[0.88 0.94 1]');
    elseif strcmp(name,'Control Allocator')
        set_param(b,'BackgroundColor','[1 0.90 0.74]');
    elseif strcmp(name,'CoM Estimator')
        set_param(b,'BackgroundColor','[1 0.96 0.76]');
    end
end
end
