function out1 = CalculateJ_linear(v,r,d_phi,e,a_wheel_angle)
%CALCULATEJ_LINEAR
%    OUT1 = CALCULATEJ_LINEAR(V,R,D_PHI,E,A_WHEEL_ANGLE)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    03-May-2020 16:12:37

t3 = imag(r);
t4 = real(r);
t6 = t3.*(7.67e2./5.0e2);
t7 = imag(v);
t8 = t4.*(4.0./5.0);
t2 = t6-t7-t8+2.0e1;
t10 = t3.*(4.0./5.0);
t11 = real(v);
t18 = t4.*(7.67e2./5.0e2);
t5 = t10-t11+t18;
t13 = t3.*(5.03e2./5.0e2);
t9 = t7-t8+t13-2.0e1;
t15 = t4.*(5.03e2./5.0e2);
t12 = t10+t11+t15;
t14 = t7+t8+t13-2.0e1;
t16 = -t10+t11+t15;
t17 = t6-t7+t8+2.0e1;
t19 = t10+t11-t18;
t20 = cos(a_wheel_angle);
t21 = t9.^2;
t22 = t12.^2;
t23 = t21+t22;
t24 = 1.0./t23;
t25 = t14.^2;
t26 = t16.^2;
t27 = t25+t26;
t28 = 1.0./t27;
t29 = t2.^2;
t30 = t5.^2;
t31 = t29+t30;
t32 = 1.0./t31;
t33 = t17.^2;
t34 = t19.^2;
t35 = t33+t34;
t36 = 1.0./t35;
t37 = v.*1i;
t38 = 1.0./t9.^2;
t39 = t12.*t38.*(4.0./5.0);
t40 = 1.0./t9;
t41 = t40.*(5.03e2./5.0e2);
t42 = t39+t41;
t43 = 1.0./t14.^2;
t44 = t16.*t43.*(4.0./5.0);
t45 = 1.0./t14;
t46 = t44-t45.*(5.03e2./5.0e2);
t47 = 1.0./t2;
t48 = t47.*(7.67e2./5.0e2);
t49 = 1.0./t2.^2;
t50 = t5.*t49.*(4.0./5.0);
t51 = t48+t50;
t52 = 1.0./t17;
t53 = t52.*(7.67e2./5.0e2);
t54 = 1.0./t17.^2;
t55 = t19.*t54.*(4.0./5.0);
t56 = t53+t55;
t57 = sin(a_wheel_angle);
t58 = r.*(-8.0e-1+1.006i);
t59 = t37+t58+2.0e1;
t60 = angle(t59);
t61 = r.*(8.0e-1+1.006i);
t62 = t37+t61+2.0e1;
t63 = angle(t62);
out1 = reshape([t2.*t32.*(-4.43816746633648e1)-t17.*t36.*4.43816746633648e1+t20.*(t9.*t24.*4.373e4+t14.*t28.*4.373e4).*7.73874013310633e-4,t2.*t32.*3.695337505775612e1+t17.*t36.*3.695337505775612e1+t20.*(t9.*t24.*4.399238e4+t14.*t28.*4.399238e4).*4.200445247196203e-4,0.0,1.0,0.0,t20.*(t21.*t24.*t42.*4.373e4-t25.*t28.*t46.*4.373e4).*7.73874013310633e-4+t29.*t32.*t51.*4.43816746633648e1+t33.*t36.*t56.*4.43816746633648e1-2.0e1,t20.*(t21.*t24.*t42.*4.399238e4-t25.*t28.*t46.*4.399238e4).*4.200445247196203e-4-t29.*t32.*t51.*3.695337505775612e1-t33.*t36.*t56.*3.695337505775612e1,1.0,0.0,0.0,0.0,0.0,0.0,2.0e1,0.0,0.0,0.0,0.0,0.0,0.0,t20.*6.768302120414796e1+t57.*(a_wheel_angle.*-8.746e4+t60.*4.373e4+t63.*4.373e4).*7.73874013310633e-4,t20.*3.695751669676986e1+t57.*(a_wheel_angle.*(-8.798476e4)+t60.*4.399238e4+t63.*4.399238e4).*4.200445247196203e-4,0.0,0.0,0.0],[5,5]);
