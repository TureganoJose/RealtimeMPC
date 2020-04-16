clear




for i=3:3
Fz = 500 + i*1000;
kappa = 0.0;
alpha = -0.2:0.01:0.2;
gamma = 0.0;
phit = 0.0;
Vx = 33.33;
P = 190000;
omega = 0.0;


 n_samples = size(alpha,2);
% [Fz kappa alpha gamma phit Vx P omega]
input = [ones(n_samples,1)*Fz ones(n_samples,1)*kappa alpha' ones(n_samples,1)*gamma ones(n_samples,1)*phit ones(n_samples,1)*Vx ones(n_samples,1)*P   ones(n_samples,1)*P   ];

outMF = mfeval('Pacejka_5_2\pac2002_175_70R13.tir',input,121);

plot(alpha,outMF(:,2),'b.')
hold on
end

car = Vehicle_v2();
for i =3:3
Fz_input = 500 + i*1000;
slip_a = alpha;
gamma = 0.0;
[fy_fl, fy_fr, fy_rl, fy_rr] = car.CalculateTyreForces(slip_a, gamma, Fz_input);

plot(alpha,fy_fl,'r.')
hold on
end