function delete_connected_lines(blockPath)
    ph = get_param(blockPath, 'PortHandles');
    f = fieldnames(ph);

    for i = 1:numel(f)
        ports = ph.(f{i});
        for k = 1:numel(ports)
            try
                lh = get_param(ports(k), 'Line');
                if lh ~= -1
                    delete_line(lh);
                end
            catch
            end
        end
    end
end