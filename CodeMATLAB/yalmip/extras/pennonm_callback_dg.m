function [nnz,ind,val] = pennonm_callback_dg(i,x,model_in)

global latest_G
global latest_g
global latest_x_g

persistent model
persistent G

if nargin>2
    model = model_in;
    return
end

x = x(:);
if ~isequal(latest_x_g,x) 
    % Compute the nonlinear terms in the constraints
    [g,geq,dg,dgeq] = fmincon_con(x,model);
    latest_x_g = x;
    
    % Append with linear constraints
    g = [g;geq];
    if ~isempty(model.A)
        g = [g;model.A*x - model.b];
    end
    if ~isempty(model.Aeq)
        g = [g;model.Aeq*x - model.beq];
    end
        
    % Append with linear terms
    G = [dg';dgeq'];
    if ~isempty(model.A)
        G = [G;model.A];
    end
    if ~isempty(model.Aeq)
        G = [G;model.Aeq];
    end
    latest_G = G;
    latest_g = g;
else
    G = latest_G;
end
Gi = G(i+1,:);
[aux,ind,val] = find(Gi);
nnz = length(val);