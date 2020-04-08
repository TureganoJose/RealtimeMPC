function [ X,U,info ] = CVXoptimizer(HorizonIter,NHorizon)
nx = 5;
nu = 1;
tic

cvx_begin
    variable x(nx*(NHorizon),1);
    variable u(nu*NHorizon,1);
    
    objective = 0;

    for i = 1:NHorizon
        objective = objective + 0.5*(quad_form(x((i-1)*(nx)+[1:nx],1),HorizonIter(i).Qk) + quad_form(u(i,1),HorizonIter(i).Rk)) + HorizonIter(i).fk'*x((i-1)*(nx)+[1:nx],1);  % cost
    end

    minimize( objective )

    subject to              
        x(1:nx,1) == [HorizonIter(1).x0];   % initialize initial state
        u(1:nu,1) == [HorizonIter(1).u0];
        for i = 2:NHorizon-1
           x((i-1)*nx+1:(i-1)*nx+nx,1) == HorizonIter(i).Ak*x((i-2)*nx+1:(i-2)*nx+nx,1) + HorizonIter(i).Bk*u(i-1,1) + HorizonIter(i).gk ; %dynamics
           HorizonIter(i).lb <= [x((i-2)*nx+1:(i-2)*nx+nx,1);u(i-1,1)] <= HorizonIter(i).ub; % bounds
        end
        for i=1:NHorizon
            norm(HorizonIter(i).x- x( (i-1)*nx+1:i*nx),1) <= 0.2;
        end
cvx_end   

QPtime = toc();

x_opt = reshape(x,nx,NHorizon);
u_opt = reshape(u,nu,NHorizon);

% rescale outputs
X= x_opt(1:nx,:);
U = u_opt;

if strcmp(cvx_status,'Solved')
    info.exitflag = 0;
else
    info.exitflag = 1;
end
info.QPtime = QPtime;

cvx_clear

end
