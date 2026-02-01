function varargout = auv_discretise(varargin)
% auv_discretise
% Zero-order hold discretisation (backward compatible).
%
% Supported call patterns:
%   [Ad,Bd,Cd,Dd] = auv_discretise(Ac,Bc,Cc,Dc,Ts)    (legacy)
%   mdl = auv_discretise(Ac,Bc,Cc,Dc,Ts)              (returns struct if nargout<=1)
%   mdl = auv_discretise(lin, Ts)                     (lin struct with Ac,Bc,Cc,Dc or A,B,C,D)
%
% This prevents signature mismatches across tasks/checks.

narginchk(2,5);

if nargin==2 && isstruct(varargin{1})
    lin = varargin{1};
    Ts  = varargin{2};

    if isfield(lin,'Ac'); Ac=lin.Ac; else; Ac=lin.A; end
    if isfield(lin,'Bc'); Bc=lin.Bc; else; Bc=lin.B; end
    if isfield(lin,'Cc'); Cc=lin.Cc; else; Cc=lin.C; end
    if isfield(lin,'Dc'); Dc=lin.Dc; else; Dc=lin.D; end
elseif nargin==5
    Ac = varargin{1};
    Bc = varargin{2};
    Cc = varargin{3};
    Dc = varargin{4};
    Ts = varargin{5};
else
    error('auv_discretise:BadInputs','Expected (lin,Ts) or (Ac,Bc,Cc,Dc,Ts).');
end

sys_c = ss(Ac, Bc, Cc, Dc);
sys_d = c2d(sys_c, Ts, 'zoh');

Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;

if nargout <= 1
    mdl = struct();
    mdl.A  = Ad;
    mdl.B  = Bd;
    mdl.C  = Cd;
    mdl.D  = Dd;
    mdl.Ts = Ts;
    mdl.nx = size(Ad,1);
    mdl.nu = size(Bd,2);
    mdl.ny = size(Cd,1);
    varargout{1} = mdl;
else
    varargout{1} = Ad;
    varargout{2} = Bd;
    varargout{3} = Cd;
    varargout{4} = Dd;
end

end
