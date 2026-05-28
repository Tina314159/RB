%% Linkage Mass Modifications

%% Find Pin block mass
pinBlocks = find_system(model, ...
    'FollowLinks','on', ...
    'LookUnderMasks','all', ...
    'BlockType','SimscapeMultibodyBlock', ...
    'MaskType','Cylindrical Solid');

%% Set mass to 0
for i = 1:length(pinBlocks)
    set_param(pinBlocks{i}, 'BasedOnType', 'Mass')
    set_param(pinBlocks{i}, 'Mass', '0');
end

%% Find all Brick solids
LinkBlocks = find_system(model, ...
    'FollowLinks','on', ...
    'LookUnderMasks','all', ...
    'BlockType','SimscapeMultibodyBlock', ...
    'MaskType','Brick Solid');

%% Set mass to 5g each
for i = 1:length(LinkBlocks)
    set_param(LinkBlocks{i}, 'BasedOnType', 'Mass')
    set_param(LinkBlocks{i}, 'MassUnits', 'kg');
    set_param(LinkBlocks{i}, 'Mass', '0.005');
end

%% Save to make change permanent 
save_system(model)