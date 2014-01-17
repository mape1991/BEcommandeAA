function [F,strict,LMIIdentifiers] = getlist(X)
% Internal class for constraint lists

% Author Johan Löfberg
% $Id: getlist.m,v 1.4 2007/09/12 14:28:29 joloef Exp $

superiorto('sdpvar');
superiorto('double');

F = X.Evaluated;
strict = X.strict;
LMIIdentifiers = X.ConstraintID;

% FIX : treat equalities better
if isequal(X.List{2},'==') 
    F{1}=sethackflag(F{1},3);
end
