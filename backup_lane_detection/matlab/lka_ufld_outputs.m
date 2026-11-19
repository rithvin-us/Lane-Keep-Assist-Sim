function o = lka_ufld_outputs(outs, cfg)
% LKA_UFLD_OUTPUTS Identifies the four UFLDv2 heads by size and orders them [grid x anchor x lane].
%
% Syntax:
%   o = lka_ufld_outputs(outs, cfg)       outs = cell of network outputs (any order / layout)
%
% Output fields: LocRow [num_grid_row x num_cls_row x num_lanes], LocCol [num_grid_col x
% num_cls_col x num_lanes], ExistRow [2 x num_cls_row x num_lanes], ExistCol [2 x num_cls_col
% x num_lanes]. The ONNX output names of deploy/pt2onnx.py are not relied on.
    want = struct('LocRow', [cfg.num_grid_row cfg.num_cls_row], 'LocCol', [cfg.num_grid_col cfg.num_cls_col], ...
        'ExistRow', [2 cfg.num_cls_row], 'ExistCol', [2 cfg.num_cls_col]);
    names = fieldnames(want);
    o = struct();
    for k = 1:numel(outs)
        y = squeeze(outs{k});
        for j = 1:numel(names)
            if isfield(o, names{j})
                continue
            end
            p = matchPermutation(size(y), [want.(names{j}) cfg.num_lanes]);
            if ~isempty(p)
                o.(names{j}) = permute(y, p);
                break
            end
        end
    end
    if numel(fieldnames(o)) ~= 4
        error('lka:vision:outputShape', 'UFLDv2 outputs not recognised (sizes %s).', ...
            strjoin(cellfun(@(y) mat2str(size(y)), outs, 'UniformOutput', false), ', '));
    end
end

function p = matchPermutation(sz, target)
    p = [];
    if numel(sz) ~= 3
        return
    end
    P = perms(1:3);
    for i = 1:size(P, 1)
        if isequal(sz(P(i, :)), target)
            p = P(i, :);
            return
        end
    end
end
