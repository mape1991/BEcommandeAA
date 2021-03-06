function varargout = acot(varargin)
%ACOT (overloaded)

% Author Johan L�fberg
% $Id: acot.m,v 1.4 2007/08/02 18:16:26 joloef Exp $
switch class(varargin{1})

    case 'double'
        error('Overloaded SDPVAR/ACOT CALLED WITH DOUBLE. Report error')

    case 'sdpvar'
        varargout{1} = InstantiateElementWise(mfilename,varargin{:});

    case 'char'

        operator = struct('convexity','none','monotonicity','none','definiteness','none','model','callback');
        operator.convexhull = [];
        operator.bounds = @bounds;
        operator.derivative = @(x)(-(1 + x.^2).^-1;

        varargout{1} = [];
        varargout{2} = operator;
        varargout{3} = varargin{3};

    otherwise
        error('SDPVAR/ACOT called with CHAR argument?');
end

function [L,U] = bounds(xL,xU)
if xL<=0 & xU >=0
    L = -inf;
    U = inf;
else
    L = acot(xU);
    U = acot(xL);
end