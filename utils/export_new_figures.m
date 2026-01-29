function export_new_figures(figs_before, prefix, outdir)
%EXPORT_NEW_FIGURES Export any figures created since figs_before to PNG.
%
% figs_before : array of figure handles existing before a block
% prefix      : filename prefix, e.g. 'task3_check02'
% outdir      : folder to save PNGs into

    if nargin < 3 || isempty(outdir)
        outdir = pwd;
    end

    figs_after = findall(0, 'Type', 'figure');
    new_figs   = setdiff(figs_after, figs_before);

    for k = 1:numel(new_figs)
        fig = new_figs(k);

        % Build a stable, safe filename
        figName = get(fig, 'Name');
        if isempty(figName)
            figName = sprintf('fig%02d', k);
        else
            figName = regexprep(figName, '[^a-zA-Z0-9_]', '_');
        end

        pngFile = fullfile(outdir, sprintf('%s_%s.png', prefix, figName));

        % Export without changing docking/interactive behaviour
        try
            exportgraphics(fig, pngFile, 'Resolution', 300);
        catch
            print(fig, pngFile, '-dpng', '-r300');
        end
    end
end
