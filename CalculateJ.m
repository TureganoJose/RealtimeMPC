function out1 = CalculateJ(v,r,d_phi,e,a_wheel_angle,dot_v)
%CALCULATEJ
%    OUT1 = CALCULATEJ(V,R,D_PHI,E,A_WHEEL_ANGLE,DOT_V)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    16-Apr-2020 11:39:09

t2 = dot_v.*2.393417940789474e-4;
t3 = r.*7.977980022033552e-3;
t4 = r.*(-8.0e-1+1.006i);
t5 = v.*1i;
t6 = t4+t5+3.3333e1;
t7 = angle(t6);
t8 = a_wheel_angle+t2+t3-t7-3.253679120072234e-3;
t9 = dot_v.*4.601607132004376e-2;
t10 = r.*1.533853705311019;
t11 = t9+t10-7.270075484541012e-1;
t12 = atan(t11);
t13 = t12.*2.0;
t14 = sin(t13);
t15 = dot_v.*2.422875e2;
t16 = r.*8.0761692375e3;
t17 = t15+t16-3.82789830472441e3;
t18 = 1.0./t17;
t19 = dot_v.*1.126886762099417e-2;
t20 = r.*3.756251644105985e-1;
t21 = t19+t20+9.389841501848343e-1;
t22 = 1.0./t21;
t23 = t8.*t14.*t18.*t22.*3.306719103040545e4;
t24 = sign(t8);
t25 = t24.*5.477e1;
t26 = t25+1.0;
t27 = atan(t23);
t28 = dot_v.*5.46543216118421e-5;
t29 = r.*1.821792502287533e-3;
t30 = t28+t29-4.008593196796505e-3;
t31 = t23-t27;
t146 = t26.*t30.*t31;
t32 = -t23-t146;
t34 = imag(r);
t35 = real(r);
t37 = t34.*(5.03e2./5.0e2);
t38 = imag(v);
t39 = t35.*(4.0./5.0);
t33 = t37+t38+t39-3.3333e1;
t41 = t34.*(4.0./5.0);
t42 = t35.*(5.03e2./5.0e2);
t43 = real(v);
t36 = -t41+t42+t43;
t40 = t33.^2;
t44 = t36.^2;
t45 = t40+t44;
t46 = 1.0./t45;
t47 = t14.*t18.*t22.*t33.*t46.*3.306719103040545e4;
t48 = r.*(8.0e-1+1.006i);
t49 = t5+t48+3.3333e1;
t50 = angle(t49);
t51 = a_wheel_angle+t2+t3-t50+3.253679120072234e-3;
t52 = t9+t10+7.270075484541012e-1;
t53 = atan(t52);
t54 = t53.*2.0;
t55 = sin(t54);
t56 = t15+t16+3.82789830472441e3;
t57 = 1.0./t56;
t58 = t19+t20-9.389841501848343e-1;
t59 = 1.0./t58;
t60 = t51.*t55.*t57.*t59.*3.306719103040545e4;
t61 = sign(t51);
t62 = t61.*5.477e1;
t63 = t62-1.0;
t64 = atan(t60);
t65 = t28+t29+4.008593196796505e-3;
t66 = t60-t64;
t149 = t63.*t65.*t66;
t67 = -t60-t149;
t68 = t37+t38-t39-3.3333e1;
t69 = t41+t42+t43;
t70 = t68.^2;
t71 = t69.^2;
t72 = t70+t71;
t73 = 1.0./t72;
t74 = t55.*t57.*t59.*t68.*t73.*3.306719103040545e4;
t75 = dot_v.*4.0394172e-6;
t76 = r.*1.346458935276e-4;
t77 = r.*(-8.0e-1-1.534i);
t78 = t5+t77+3.3333e1;
t79 = angle(t78);
t80 = t75+t76+t79+2.052375566584365e-3;
t81 = dot_v.*3.365291127284849e-2;
t82 = r.*1.121752491457859;
t83 = t81+t82-3.486780786815366e-1;
t84 = atan(t83);
t85 = t84.*2.0;
t86 = sin(t85);
t87 = t15+t16-2.510342695275591e3;
t88 = 1.0./t87;
t89 = dot_v.*7.437692459721616e-3;
t90 = r.*2.479206027599006e-1;
t91 = t89+t90+1.060252117613917;
t92 = 1.0./t91;
t93 = t80.*t86.*t88.*t92.*5.987513278290252e4;
t94 = sign(t80);
t95 = t94.*4.39616e-1;
t96 = t95+1.0;
t97 = atan(t93);
t98 = dot_v.*3.07596095625e-3;
t99 = r.*1.025310065546813e-1;
t100 = t98+t99-6.188080556878713e-1;
t101 = t93-t97;
t139 = t96.*t100.*t101;
t102 = t93-t139;
t105 = t34.*(7.67e2./5.0e2);
t103 = -t38-t39+t105+3.3333e1;
t107 = t35.*(7.67e2./5.0e2);
t104 = t41-t43+t107;
t106 = t103.^2;
t108 = t104.^2;
t109 = t106+t108;
t110 = 1.0./t109;
t111 = t86.*t88.*t92.*t103.*t110.*5.987513278290252e4;
t112 = r.*(8.0e-1-1.534i);
t113 = t5+t112+3.3333e1;
t114 = angle(t113);
t115 = t75+t76+t114-2.052375566584365e-3;
t116 = t81+t82+3.486780786815366e-1;
t117 = atan(t116);
t118 = t117.*2.0;
t119 = sin(t118);
t120 = t15+t16+2.510342695275591e3;
t121 = 1.0./t120;
t122 = t89+t90-1.060252117613917;
t123 = 1.0./t122;
t124 = t115.*t119.*t121.*t123.*5.987513278290252e4;
t125 = sign(t115);
t126 = t125.*4.39616e-1;
t127 = t126-1.0;
t128 = atan(t124);
t129 = t98+t99+6.188080556878713e-1;
t130 = t124-t128;
t142 = t127.*t129.*t130;
t131 = t124-t142;
t132 = -t38+t39+t105+3.3333e1;
t133 = t41+t43-t107;
t134 = t132.^2;
t135 = t133.^2;
t136 = t134+t135;
t137 = 1.0./t136;
t138 = t119.*t121.*t123.*t132.*t137.*5.987513278290252e4;
t140 = atan(t102);
t141 = t140.*1.3223;
t143 = atan(t131);
t144 = t143.*1.3223;
t145 = cos(a_wheel_angle);
t147 = atan(t32);
t148 = t147.*(5.87e2./4.0e2);
t150 = atan(t67);
t151 = t150.*(5.87e2./4.0e2);
t152 = sin(t148);
t153 = sin(t151);
t154 = cos(t148);
t155 = t32.^2;
t156 = t155+1.0;
t157 = 1.0./t156;
t158 = t14.^2;
t159 = 1.0./t17.^2;
t160 = 1.0./t21.^2;
t161 = t8.^2;
t162 = t158.*t159.*t160.*t161.*1.093439122641326e9;
t163 = t162+1.0;
t164 = 1.0./t163;
t165 = t8.*t14.*t18.*t160.*1.242086906739271e4;
t166 = t8.*t14.*t22.*t159.*2.670562309702964e8;
t167 = 1.0./t33.^2;
t168 = t36.*t167.*(4.0./5.0);
t169 = 1.0./t33;
t179 = t169.*(5.03e2./5.0e2);
t170 = t168-t179;
t171 = t40.*t46.*t170;
t172 = t171-7.977980022033552e-3;
t173 = t14.*t18.*t22.*t172.*3.306719103040545e4;
t174 = cos(t13);
t175 = t11.^2;
t176 = t175+1.0;
t177 = 1.0./t176;
t178 = dirac(t8);
t180 = cos(t151);
t181 = t67.^2;
t182 = t181+1.0;
t183 = 1.0./t182;
t184 = t55.^2;
t185 = 1.0./t56.^2;
t186 = 1.0./t58.^2;
t187 = t51.^2;
t188 = t184.*t185.*t186.*t187.*1.093439122641326e9;
t189 = t188+1.0;
t190 = 1.0./t189;
t191 = t51.*t55.*t57.*t186.*1.242086906739271e4;
t192 = t51.*t55.*t59.*t185.*2.670562309702964e8;
t193 = 1.0./t68.^2;
t194 = t69.*t193.*(4.0./5.0);
t195 = 1.0./t68;
t196 = t195.*(5.03e2./5.0e2);
t197 = t194+t196;
t198 = t70.*t73.*t197;
t199 = t198+7.977980022033552e-3;
t200 = cos(t54);
t201 = t52.^2;
t202 = t201+1.0;
t203 = 1.0./t202;
t204 = dirac(t51);
t205 = t55.*t57.*t59.*t199.*3.306719103040545e4;
t206 = t51.*t57.*t59.*t200.*t203.*1.014404669724293e5;
t207 = sin(t141);
t208 = sin(t144);
t209 = cos(t141);
t210 = t102.^2;
t211 = t210+1.0;
t212 = 1.0./t211;
t213 = t86.^2;
t214 = 1.0./t87.^2;
t215 = 1.0./t91.^2;
t216 = t80.^2;
t217 = t213.*t214.*t215.*t216.*3.585031525770208e9;
t218 = t217+1.0;
t219 = 1.0./t218;
t220 = t80.*t86.*t88.*t215.*1.484427900986628e4;
t221 = t80.*t86.*t92.*t214.*4.835617054725051e8;
t222 = 1.0./t103;
t223 = t222.*(7.67e2./5.0e2);
t224 = 1.0./t103.^2;
t225 = t104.*t224.*(4.0./5.0);
t226 = t223+t225;
t227 = t106.*t110.*t226;
t228 = t227-1.346458935276e-4;
t229 = t86.*t88.*t92.*t228.*5.987513278290252e4;
t230 = cos(t85);
t231 = t83.^2;
t232 = t231+1.0;
t233 = 1.0./t232;
t234 = dirac(t80);
t235 = cos(t144);
t236 = t131.^2;
t237 = t236+1.0;
t238 = 1.0./t237;
t239 = t119.^2;
t240 = 1.0./t120.^2;
t241 = 1.0./t122.^2;
t242 = t115.^2;
t243 = t239.*t240.*t241.*t242.*3.585031525770208e9;
t244 = t243+1.0;
t245 = 1.0./t244;
t246 = 1.0./t132;
t247 = t246.*(7.67e2./5.0e2);
t248 = 1.0./t132.^2;
t249 = t133.*t248.*(4.0./5.0);
t250 = t247+t249;
t251 = t134.*t137.*t250;
t252 = t251-1.346458935276e-4;
t253 = t119.*t121.*t123.*t252.*5.987513278290252e4;
t254 = t115.*t119.*t121.*t241.*1.484427900986628e4;
t255 = t115.*t119.*t123.*t240.*4.835617054725051e8;
t256 = cos(t118);
t257 = t116.^2;
t258 = t257+1.0;
t259 = 1.0./t258;
t260 = dirac(t115);
t261 = t14.*t18.*t22.*3.306719103040545e4;
t262 = t55.*t57.*t59.*3.306719103040545e4;
t263 = dot_v.*1.529114668421053e-4;
t264 = r.*5.096997924247895e-3;
t265 = t47-t14.*t18.*t22.*t33.*t46.*t164.*3.306719103040545e4;
t266 = t26.*t30.*t265;
t267 = t30.*t33.*t46.*t178.*(t23-t27).*1.0954e2;
t268 = t47+t266+t267;
t269 = t74-t55.*t57.*t59.*t68.*t73.*t190.*3.306719103040545e4;
t270 = t63.*t65.*t269;
t271 = t65.*t66.*t68.*t73.*t204.*1.0954e2;
t272 = t74+t270+t271;
t273 = t111-t86.*t88.*t92.*t103.*t110.*t219.*5.987513278290252e4;
t274 = t96.*t100.*t273;
t275 = t100.*t101.*t103.*t110.*t234.*8.79232e-1;
t276 = t138-t119.*t121.*t123.*t132.*t137.*t245.*5.987513278290252e4;
t277 = t127.*t129.*t276;
t278 = t129.*t130.*t132.*t137.*t260.*8.79232e-1;
t279 = -t138+t277+t278;
t281 = t8.*t18.*t22.*t174.*t177.*1.014404669724293e5;
t280 = t165+t166+t173-t281;
t282 = t30.*t31.*t172.*t178.*1.0954e2;
t283 = t63.*t66.*1.821792502287533e-3;
t284 = t191+t192-t205-t206;
t285 = t190.*t284;
t286 = -t191-t192+t205+t206+t285;
t287 = t63.*t65.*t286;
t288 = t65.*t66.*t199.*t204.*1.0954e2;
t289 = -t191-t192+t205+t206+t283+t287+t288;
t291 = t80.*t88.*t92.*t230.*t233.*1.34330158751182e5;
t290 = t220+t221+t229-t291;
t292 = t127.*(t124-t128).*1.025310065546813e-1;
t294 = t115.*t121.*t123.*t256.*t259.*1.34330158751182e5;
t293 = t253+t254+t255-t294;
t295 = t30.*t178.*(t23-t27).*1.0954e2;
t296 = t261-t14.*t18.*t22.*t164.*3.306719103040545e4;
t297 = t26.*t30.*t296;
t298 = t261+t295+t297;
t299 = t262-t55.*t57.*t59.*t190.*3.306719103040545e4;
t300 = t63.*t65.*t299;
t301 = t65.*t66.*t204.*1.0954e2;
t302 = t262+t300+t301;
t303 = sin(a_wheel_angle);
t304 = t263+t264+2.360419293938887e-2;
t305 = t263+t264-2.360419293938887e-2;
out1 = reshape([t145.*(t17.*t21.*t154.*t157.*t268.*(5.87e2./4.0e2)-t56.*t58.*t180.*t183.*t272.*(5.87e2./4.0e2)).*(-7.73874013310633e-4)-t87.*t91.*t209.*t212.*(-t111+t274+t275).*1.02329360780065e-3+t120.*t122.*t235.*t238.*t279.*1.02329360780065e-3,t145.*(t17.*t21.*t154.*t157.*t268.*1.476305-t56.*t58.*t180.*t183.*t272.*1.476305).*(-4.200445247196203e-4)+t87.*t91.*t209.*t212.*(-t111+t274+t275).*8.520217583063805e-4-t120.*t122.*t235.*t238.*t279.*8.520217583063805e-4,0.0,1.0,0.0,dot_v.*(-2.143662648315398e-2)-r.*7.145470705629718e-1+t87.*t207.*1.918593118401955e-4+t91.*t207.*6.2499375-t120.*t208.*1.918593118401955e-4-t122.*t208.*6.2499375-t145.*(dot_v.*4.939755538284847+r.*1.646568713576488e2-t17.*t152.*3.756251644105985e-1-t21.*t152.*8.0761692375e3+t56.*t153.*3.756251644105985e-1+t58.*t153.*8.0761692375e3-t17.*t21.*t154.*t157.*(t165+t166+t173+t282-t26.*t31.*1.821792502287533e-3+t26.*t30.*(t165+t166+t173-t164.*t280-t8.*t18.*t22.*t174.*t177.*1.014404669724293e5)-t8.*t18.*t22.*t174.*t177.*1.014404669724293e5).*(5.87e2./4.0e2)-t56.*t58.*t180.*t183.*t289.*(5.87e2./4.0e2)).*7.73874013310633e-4-t87.*t91.*t209.*t212.*(t220+t221+t229+t96.*t101.*1.025310065546813e-1-t96.*t100.*(t220+t221+t229-t219.*t290-t80.*t88.*t92.*t230.*t233.*1.34330158751182e5)-t100.*t101.*t228.*t234.*8.79232e-1-t80.*t88.*t92.*t230.*t233.*1.34330158751182e5).*1.02329360780065e-3+t120.*t122.*t235.*t238.*(t253+t254+t255+t292-t127.*t129.*(t253+t254+t255-t245.*t293-t115.*t121.*t123.*t256.*t259.*1.34330158751182e5)-t129.*t130.*t252.*t260.*8.79232e-1-t115.*t121.*t123.*t256.*t259.*1.34330158751182e5).*1.02329360780065e-3-3.3333e1,dot_v.*1.784871130739255e-2+r.*5.949510940093158e-1-t145.*(dot_v.*4.969394071514556+r.*1.656448125857947e2-t17.*t152.*3.778789153970621e-1-t21.*t152.*8.124626252925e3+t56.*t153.*3.778789153970621e-1+t58.*t153.*8.124626252925e3-t17.*t21.*t154.*t157.*(t165+t166+t173-t281+t282-t26.*t31.*1.821792502287533e-3+t26.*t30.*(t165+t166+t173-t281-t164.*t280)).*1.476305-t56.*t58.*t180.*t183.*t289.*1.476305).*4.200445247196203e-4-t87.*t207.*1.597472191513788e-4-t91.*t207.*5.203865926124669+t120.*t208.*1.597472191513788e-4+t122.*t208.*5.203865926124669-t120.*t122.*t235.*t238.*(t253+t254+t255+t292-t294-t127.*t129.*(t253+t254+t255-t294-t245.*t293)-t129.*t130.*t252.*t260.*8.79232e-1).*8.520217583063805e-4+t87.*t91.*t209.*t212.*(t220+t221+t229-t291+t96.*(t93-t97).*1.025310065546813e-1-t96.*t100.*(t220+t221+t229-t291-t219.*t290)-t100.*t101.*t228.*t234.*8.79232e-1).*8.520217583063805e-4,1.0,0.0,0.0,0.0,0.0,0.0,3.3333e1,0.0,0.0,0.0,0.0,0.0,0.0,t303.*(t17.*t304+t56.*t305-t17.*t21.*t152+t56.*t58.*t153).*7.73874013310633e-4-t145.*(t17.*t21.*t154.*t157.*t298.*(5.87e2./4.0e2)-t56.*t58.*t180.*t183.*t302.*(5.87e2./4.0e2)).*7.73874013310633e-4,t303.*(t17.*t304.*(5.03e2./5.0e2)+t56.*t305.*(5.03e2./5.0e2)-t17.*t21.*t152.*(5.03e2./5.0e2)+t56.*t58.*t153.*(5.03e2./5.0e2)).*4.200445247196203e-4-t145.*(t17.*t21.*t154.*t157.*t298.*1.476305-t56.*t58.*t180.*t183.*t302.*1.476305).*4.200445247196203e-4,0.0,0.0,0.0],[5,5]);
