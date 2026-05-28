%% reset joints
% Find all Revolute Joint blocks
revJoints = find_system(model, ...
    'FollowLinks','on', ...
    'LookUnderMasks','all', ...
    'BlockType','SimscapeMultibodyBlock', ...
    'MaskType','Revolute Joint');

for i = 1:length(revJoints)
    blk = revJoints{i};
    try
        % Set damping
        set_param(blk, 'DampingCoefficient', 'dampingOfJoints'); %0 or 0.001
        %fprintf("Set damping for: %s\n", blk);

    catch ME
        fprintf("Failed on: %s\n", blk);
        disp(ME.message);
    end
end
