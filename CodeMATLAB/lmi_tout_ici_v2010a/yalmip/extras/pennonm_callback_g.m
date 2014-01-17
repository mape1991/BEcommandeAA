function gi = pennonm_callback_g(i,x,model_in)

global latest_G
global latest_g
global latest_x_g

persistent model
persistent g
persistent geq

if nargin>2
    model = model_in;
    return
end

% Compute the nonlinear terms in the constraints
x = x(:);
if ~isequal(latest_x_g,x)
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
    g = latest_g;
end
gi = full(g(i+1));