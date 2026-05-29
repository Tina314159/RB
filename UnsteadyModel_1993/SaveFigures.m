%% Export ALL open figures into one PDF

%% pdf deletion
pdfName = 'Visuals/Iteration_60rpm_40deg.png';

% delete old PDF first
if exist(pdfName,'file')
    delete(pdfName);
end

%% folder deletion
folderName = 'slprj';

if exist(folderName,'dir')
    rmdir(folderName,'s');
end

%% folder content moveout
srcFolder = 'UnsteadyModel_1993';
dstFolder = '.';

movefile(fullfile(srcFolder,'*'), dstFolder);

%% folder creation
newFolder = 'Visuals';

% create folder if it does not exist
if ~exist(newFolder,'dir')
    mkdir(newFolder);
end

%% move file into folder
movefile('/Users/uwstuff/Desktop/MATLAB/UnsteadyModel_1993/AR_definition.png', newFolder);

%% save images to pdf
% get all open figures
figs = findall(groot,'Type','figure','Visible','on');

% sort figures by number (optional but nice)
[~,idx] = sort([figs.Number]);
figs = figs(idx);

% export each figure
for k = 1:length(figs)

    fig = figs(k);
    set(fig,'Visible','off');

    set(fig,'Units','normalized','OuterPosition',[0 0 1 1]);

    drawnow;

    if k == 1

        exportgraphics(fig, pdfName, ...
            'ContentType','image', ...
            'Resolution',300);

    else

        exportgraphics(fig, pdfName, ...
            'ContentType','image', ...
            'Resolution',300,...
            'Append', true);

    end
end

disp("Exported all open figures to " + pdfName);