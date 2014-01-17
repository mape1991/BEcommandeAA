function [g,geq,dg,dgeq,xevaled] = fmincon_con(x,model,xevaled)

% Early bail for linear problems
g = [];
geq = [];
dg = [];
dgeq = [];
xevaled = [];
if model.linearconstraints
    return
end

if nargin<3
    xevaled = zeros(1,length(model.c));
    xevaled(model.linearindicies) = x;
    xevaled = apply_recursive_evaluation(model,xevaled);
end

if model.nonlinearinequalities
    g = model.Anonlinineq*xevaled(:)-model.bnonlinineq;
end

if model.nonlinearequalities
    geq = model.Anonlineq*xevaled(:)-model.bnonlineq;
end

if nargout == 2
    return
elseif isempty(model.evalMap) & (model.nonlinearinequalities | model.nonlinearequalities)
    allA = [model.Anonlineq;model.Anonlinineq];
    dgAll = [];
    n = length(model.c);
    linearindicies = model.linearindicies;
    mtNonlinear = model.monomtable(model.nonlinearindicies,:);
    xevaled = zeros(1,n);
    xevaled(linearindicies) = x;
    X = repmat(xevaled,size(mtNonlinear,1),1);
    %mm = [];
    %op = [];
    % FIXME: This should be vectorized
    for i = 1:length(linearindicies)
        mt = mtNonlinear;
        oldpower = mtNonlinear(:,linearindicies(i));
        mt(:,linearindicies(i)) = mt(:,linearindicies(i))-1;
     %   mm = [mm;mt];
     %   op = [op;oldpower];
        dxevaledNonLinear = prod(X.^mt,2);
        dxevaledNonLinear = dxevaledNonLinear(:)'.*oldpower';dxevaledNonLinear(isnan(dxevaledNonLinear))=0;
        dx = zeros(1,n);
        dx(linearindicies(i)) = 1;
        dx(model.nonlinearindicies) = dxevaledNonLinear;
        dgAll = [dgAll allA*dx'];
    end
    %mt;
    %op;
else
    allA = [model.Anonlineq;model.Anonlinineq];
    requested = any(allA',2);
    [i,j,k] = find((model.deppattern(find(requested),:)));
    requested(j) = 1;
    dx = apply_recursive_differentiation(model,xevaled,requested);
%     dx2 = apply_oldrecursive_differentiation(model,xevaled,requested);
%     
%     if norm(dx(requested,:)-dx2(requested,:))>1e-14
%         disp('DIFFERENTIATION WRONG')
%         error('DIFFERENTIATION WRONG')
%     end
    
    dgAll = allA*dx;
end
if  model.nonlinearequalities
    dgeq = dgAll(1:size(model.Anonlineq,1),:)';
end
if model.nonlinearinequalities
    dg = dgAll(size(model.Anonlineq,1)+1:end,:)';
end