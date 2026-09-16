function v2_initialize_model(model)
% The only persistent model-workspace override is explicitly named for tests.
w = get_param(model,'ModelWorkspace');
overrides = struct();
if hasVariable(w,'V2_OVERRIDES'), overrides=getVariable(w,'V2_OVERRIDES'); end
init_v2(model,overrides);
end
