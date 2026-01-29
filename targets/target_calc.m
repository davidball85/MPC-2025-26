function [x_ss, u_ss, info] = target_calc(mode, data, model, C, T)
% target_calc
% Wrapper so checks/sims can switch between methods.
%
% Inputs
%   mode  : 'linear' or 'qp'
%   data  : struct with fields: x_hat (6x1), d_hat (scalar), x_ref (6x1)
%   model : struct with fields: A,B,Bw
%
% Output
%   x_ss (6x1), u_ss (3x1)

if nargin < 1 || isempty(mode)
    mode = 'qp';
end

switch lower(mode)
    case 'linear'
        [x_ss, u_ss, info] = target_calc_linear(data, model, C, T);
    case 'qp'
        [x_ss, u_ss, info] = target_calc_qp(data, model, C, T);
    otherwise
        error('Unknown target_calc mode: %s', mode);
end
end
