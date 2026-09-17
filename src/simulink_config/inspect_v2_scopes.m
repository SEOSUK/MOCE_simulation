function inspect_v2_scopes()
% Inspect actual Scope-captured data, independently of logsout, in default hover.
here=fileparts(mfilename('fullpath'));load_system(fullfile(fileparts(here),'V2.slx'));m='V2';
w=get_param(m,'ModelWorkspace');previous=struct();
if hasVariable(w,'V2_OVERRIDES'),previous=getVariable(w,'V2_OVERRIDES');end
scopes=find_system([m '/Monitoring'],'SearchDepth',1,'BlockType','Scope');
configs=cell(size(scopes));settings=cell(size(scopes));
for k=1:numel(scopes)
    configs{k}=get_param(scopes{k},'ScopeConfiguration');c=configs{k};
    settings{k}={c.DataLogging,c.DataLoggingVariableName,c.DataLoggingSaveFormat,c.DataLoggingLimitDataPoints,c.DataLoggingDecimateData};
end
cleanup=onCleanup(@()restore(w,previous,configs,settings));
assignin(w,'V2_OVERRIDES',struct());v2_initialize_model(m);
for k=1:numel(scopes)
    c=configs{k};c.DataLogging=true;c.DataLoggingVariableName=sprintf('scope_data_%d',k);
    c.DataLoggingSaveFormat='Dataset';c.DataLoggingLimitDataPoints=false;c.DataLoggingDecimateData=false;
end
out=sim(m,'StopTime','10');
fig=figure('Visible','off','Renderer','painters','Position',[0 0 1800 1100]);
f=onCleanup(@()close(fig));tiledlayout(fig,3,3,'TileSpacing','compact');
for k=1:numel(scopes)
    data=out.get(sprintf('scope_data_%d',k));
    n=str2double(get_param(scopes{k},'NumInputPorts'));assert(data.numElements==n);
    ax=nexttile;hold(ax,'on');labels={};
    for j=1:n
        element=data.get(j);signal=element.Values;
        values=reshape(signal.Data,[],numel(signal.Time))';
        assert(all(isfinite(values),'all'));
        plot(ax,signal.Time,values,'LineWidth',.8);
        for axis=1:size(values,2),labels{end+1}=sprintf('%s[%d]',element.Name,axis);end %#ok<AGROW>
    end
    title(ax,get_param(scopes{k},'Name'));xlabel(ax,'Time (s)');grid(ax,'on');
    legend(ax,labels,'Interpreter','none','FontSize',6,'Location','best');
    if strcmp(get_param(scopes{k},'Name'),'Torque and DOB')
        export_torque(data,here);
    end
end
print(fig,'-dpng','-r110',fullfile(here,'V2_scope_diagnostics.png'));
fprintf('PASS nine Scope datasets: connected signals, finite hover traces, and waveform exports\n');
end
function export_torque(data,here)
f=figure('Visible','off','Renderer','painters','Position',[0 0 1200 800]);c=onCleanup(@()close(f));
tiledlayout(f,3,1,'TileSpacing','compact');names={'Roll','Pitch','Yaw'};
for k=1:3
    ax=nexttile;hold(ax,'on');labels={};
    for j=1:data.numElements
        e=data.get(j);s=e.Values;y=reshape(s.Data,3,[])';
        plot(ax,s.Time,y(:,k),'LineWidth',1);labels{end+1}=e.Name; %#ok<AGROW>
    end
    title(ax,[names{k} ' torque and DOB']);ylabel(ax,'Torque (N m)');grid(ax,'on');
    legend(ax,labels,'Interpreter','none','Location','best');
end
xlabel(ax,'Time (s)');print(f,'-dpng','-r120',fullfile(here,'V2_scope_torque_dob.png'));
end
function restore(w,previous,configs,settings)
assignin(w,'V2_OVERRIDES',previous);v2_initialize_model('V2');
for k=1:numel(configs)
    c=configs{k};s=settings{k};c.DataLogging=s{1};c.DataLoggingVariableName=s{2};
    c.DataLoggingSaveFormat=s{3};c.DataLoggingLimitDataPoints=s{4};c.DataLoggingDecimateData=s{5};
end
end
