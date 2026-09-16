function export_v2_diagram()
% Native Simulink rendering, useful for reviewing the top-level deliverable.
here=fileparts(mfilename('fullpath'));
load_system(fullfile(fileparts(here),'V2.slx'));
set_param('V2','PaperOrientation','landscape','PaperType','A3','PaperPositionMode','auto');
print('-sV2','-dpng','-r150',fullfile(here,'V2_top_level.png'));
end
