%% savePDF

% delete old PDF first
if exist(pdfName,'file')
    delete(pdfName);
end

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

%% move file into folder
Folder = 'PDFs';
movefile(pdfName, Folder);
disp("Exported all open figures to " + pdfName);
