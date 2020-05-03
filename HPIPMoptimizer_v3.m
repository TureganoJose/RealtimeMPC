function [ X,U,info ] = HPIPMoptimizer_v3(HorizonIter,NHorizon)
nx = 5; % number of car states
nu = 1; % number of control states


% dims
dim = hpipm_ocp_qp_dim(NHorizon);

nbx = nx;
nbu = nu;
dim.set('nx', nx, 0, NHorizon);
dim.set('nu', nu, 0, NHorizon); % controls
dim.set('nbx', nbx,0, NHorizon); % state bounds
dim.set('nbu', nbu,0, NHorizon); % control bounds
%dim.set('ng', ng, 0, N); % general linear constraints
%dim.set('ns', ns, 0, N); % slacks


% qp
qp = hpipm_ocp_qp(dim);
%% Equility Constraints
for i = 0:NHorizon-1
   qp.set('A', HorizonIter(i+1).Ak, i); 
   qp.set('B', HorizonIter(i+1).Bk, i); 
   qp.set('b', HorizonIter(i+1).gk, i); 
end

%% Cost
for i = 0:NHorizon-1
    qp.set('Q', HorizonIter(i+1).Qk, i);
    qp.set('q', HorizonIter(i+1).fk, i);
    qp.set('R', HorizonIter(i+1).Rk, i); 
end

%% Bounds
%qp.print_C_struct();
for i = 0:NHorizon-1
    % Initial conditions here
    if i==0 
        % Lower/Upper bounds for states
        qp.set('Jbx', eye(nx), i)
        qp.set('lbx', HorizonIter(1).x0, i)
        qp.set('ubx', HorizonIter(1).x0, i)
        % Lower/upper bounds for controls
        qp.set('Jbu', eye(nu), i)
        qp.set('lbu', HorizonIter(1).u0, i)
        qp.set('ubu', HorizonIter(1).u0, i)
    else
        % Lower/Upper bounds for states
        qp.set('Jbx', eye(nx), i)
        qp.set('lbx', HorizonIter(i+1).lb(1:nx), i)
        qp.set('ubx', HorizonIter(i+1).ub(1:nx), i)
        % Lower/upper bounds for controls
        qp.set('Jbu', eye(nu), i)
        qp.set('lbu', HorizonIter(i+1).lb(nx+1:nx+nu), i)
        qp.set('ubu', HorizonIter(i+1).ub(nx+1:nx+nu), i)
    end
end
    
%qp.print_C_struct();


%%
qp_sol = hpipm_ocp_qp_sol(dim);


%% set up solver arg
%mode = 'speed_abs';
mode = 'speed';
%mode = 'balance';
%mode = 'robust';
% create and set default arg based on mode
arg = hpipm_ocp_qp_solver_arg(dim, mode);

arg.set('mu0', 1e0);
arg.set('iter_max', 200);
arg.set('tol_stat', 1e-6);
arg.set('tol_eq', 1e-6);
arg.set('tol_ineq', 1e-6);
arg.set('tol_comp', 1e-5);
arg.set('reg_prim', 1e-12);


% set up solver
solver = hpipm_ocp_qp_solver(dim, arg);


% solve qp
qptime = tic;
solver.solve(qp, qp_sol);
tmp_time = toc(qptime);

return_flag = solver.get('status');

% fprintf('solve time %e\n', tmp_time);
% 
% fprintf('HPIPM returned with flag %d ', return_flag);
% 
% if return_flag==0
%     fprintf('-> QP solved\n')
% %     qp_sol.print_C_struct()
% else
%     fprintf('-> Solver failed!\n')
% end


% extract and print sol
U = zeros(nu,NHorizon);
X = zeros(nx,NHorizon);
for i=0:NHorizon-1
    U(:,i+1) = qp_sol.get('u', i);
end
for i=0:NHorizon-1
	X(:,i+1) = qp_sol.get('x', i);
end

if return_flag == 0
    info.exitflag = 0;
else
    info.exitflag = 1;
end
info.QPtime = tmp_time;
info.cost = 0;


% % get solution statistics
% status = solver.get('status');
% time_ext = solver.get('time_ext');
% iter = solver.get('iter');
% res_stat = solver.get('max_res_stat');
% res_eq = solver.get('max_res_eq');
% res_ineq = solver.get('max_res_ineq');
% res_comp = solver.get('max_res_comp');
% stat = solver.get('stat');
% 
% status
% iter
% res_stat
% res_eq
% res_ineq
% res_comp
% fprintf('\nprint solver statistics\n');
% fprintf('solve time: %e [s]\n', tmp_time);
% fprintf('solve time of last run (measured in mex interface): %e [s]\n', time_ext);
% fprintf('iter\talpha_aff\tmu_aff\t\tsigma\t\talpha_prim\talpha_dual\tmu\t\tres_stat\tres_eq\t\tres_ineq\tres_comp\n');
% for ii=1:iter+1
% 	fprintf('%d\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\n', stat(ii,1), stat(ii,2), stat(ii,3), stat(ii,4), stat(ii,5), stat(ii,6), stat(ii,7), stat(ii,8), stat(ii,9), stat(ii,10), stat(ii,11));
% end




end