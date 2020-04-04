function [ X,U,dU,info ] = CVXoptimizer(HorizonIter,NHorizon)
nx = 5;
nu = 1;
tic

cvx_begin
    variable x((nx+nu)*(NHorizon));
    variable u(nu*NHorizon);
    
    objective = 0;

    for i = 1:NHorizon
        objective = objective + 0.5*(quad_form(x((i-1)*(nx+nu)+[1:nx+nu]),HorizonIter(i).Qk) + quad_form(u((i-1)*nu+[1:nu]),HorizonIter(i).Rk)) + HorizonIter(i).fk'*x((i-1)*(nx+nu)+[1:nx+nu]);  % cost
    end

    minimize( objective )

    subject to              
        x(1:nx+nu) == [HorizonIter(1).x0;HorizonIter(1).u0];   % initialize initial state

        for i = 1:NHorizon
           x((i)*(nx+nu)+[1:nx+nu]) == HorizonIter(i).Ak*x((i-1)*(nx+nu)+[1:nx+nu]) + HorizonIter(i).Bk*u((i-1)*nu+[1:nu]) + HorizonIter(i).gk ; %dynamics
           HorizonIter(i).lb <= [x((i-1)*(nx+nu)+[1:nx+nu]);u((i-1)*nu+[1:nu])] <= HorizonIter(i).ub; % bounds
        end
    
cvx_end   

QPtime = toc();

x_opt = reshape(x,nx+nu,NHorizon);
u_opt = reshape(u,nu,NHorizon);

% rescale outputs
X= x_opt(1:nx,:);
U = x_opt(nx+1:end,2:end);
dU = u_opt;

if strcmp(cvx_status,'Solved')
    info.exitflag = 0;
else
    info.exitflag = 1;
end
info.QPtime = QPtime;

cvx_clear

end
