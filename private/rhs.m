function RHS = rhs(x,g,A_P_AB,A_P_AC,A_P_AD,A_P_AE,mVals,A_I_AA,B_I_BB,C_I_CC,D_I_DD,E_I_EE,fVals,tVals)
%RHS
%    RHS = RHS(IN1,G,IN3,IN4,IN5,IN6,IN7,IN8,IN9,IN10,IN11,IN12,IN13,IN14)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    12-Jun-2023 15:32:28

%    x = [q;u]
%    g = Gravitational Constant
%    A_P_AB = [lB;wB;hB]
%    A_P_AC = [lC;wC;hC]
%    A_P_AD = [lD;wD;hD]
%    A_P_AE = [lE;wE;hE]
%    mVals = [mA;mB;mC;mD;mE]
%    A_I_AA = Inertia Tensor for body A
%    B_I_BB = Inertia Tensor for body B
%    C_I_CC = Inertia Tensor for body C
%    D_I_DD = Inertia Tensor for body D
%    E_I_EE = Inertia Tensor for body E
%    fVals = [fB; fC; fD; fE]
%    tVals = [tB; tC; tD; tE]
A_I_AA1_1 = A_I_AA(1);
A_I_AA1_2 = A_I_AA(4);
A_I_AA1_3 = A_I_AA(7);
A_I_AA2_1 = A_I_AA(2);
A_I_AA2_2 = A_I_AA(5);
A_I_AA2_3 = A_I_AA(8);
A_I_AA3_1 = A_I_AA(3);
A_I_AA3_2 = A_I_AA(6);
A_I_AA3_3 = A_I_AA(9);
B_I_BB1_1 = B_I_BB(1);
B_I_BB1_2 = B_I_BB(4);
B_I_BB1_3 = B_I_BB(7);
B_I_BB2_1 = B_I_BB(2);
B_I_BB2_2 = B_I_BB(5);
B_I_BB2_3 = B_I_BB(8);
B_I_BB3_1 = B_I_BB(3);
B_I_BB3_2 = B_I_BB(6);
B_I_BB3_3 = B_I_BB(9);
C_I_CC1_1 = C_I_CC(1);
C_I_CC1_2 = C_I_CC(4);
C_I_CC1_3 = C_I_CC(7);
C_I_CC2_1 = C_I_CC(2);
C_I_CC2_2 = C_I_CC(5);
C_I_CC2_3 = C_I_CC(8);
C_I_CC3_1 = C_I_CC(3);
C_I_CC3_2 = C_I_CC(6);
C_I_CC3_3 = C_I_CC(9);
D_I_DD1_1 = D_I_DD(1);
D_I_DD1_2 = D_I_DD(4);
D_I_DD1_3 = D_I_DD(7);
D_I_DD2_1 = D_I_DD(2);
D_I_DD2_2 = D_I_DD(5);
D_I_DD2_3 = D_I_DD(8);
D_I_DD3_1 = D_I_DD(3);
D_I_DD3_2 = D_I_DD(6);
D_I_DD3_3 = D_I_DD(9);
E_I_EE1_1 = E_I_EE(1);
E_I_EE1_2 = E_I_EE(4);
E_I_EE1_3 = E_I_EE(7);
E_I_EE2_1 = E_I_EE(2);
E_I_EE2_2 = E_I_EE(5);
E_I_EE2_3 = E_I_EE(8);
E_I_EE3_1 = E_I_EE(3);
E_I_EE3_2 = E_I_EE(6);
E_I_EE3_3 = E_I_EE(9);
e0 = x(4,:);
e1 = x(5,:);
e2 = x(6,:);
e3 = x(7,:);
fB = fVals(1,:);
fC = fVals(2,:);
fD = fVals(3,:);
fE = fVals(4,:);
hB = A_P_AB(3,:);
hC = A_P_AC(3,:);
hD = A_P_AD(3,:);
hE = A_P_AE(3,:);
lB = A_P_AB(1,:);
lC = A_P_AC(1,:);
lD = A_P_AD(1,:);
lE = A_P_AE(1,:);
mA = mVals(1,:);
mB = mVals(2,:);
mC = mVals(3,:);
mD = mVals(4,:);
mE = mVals(5,:);
q4 = x(8,:);
q5 = x(9,:);
q6 = x(10,:);
q7 = x(11,:);
tB = tVals(1,:);
tC = tVals(2,:);
tD = tVals(3,:);
tE = tVals(4,:);
u4 = x(18,:);
u5 = x(19,:);
u6 = x(20,:);
u7 = x(21,:);
w1 = x(15,:);
w2 = x(16,:);
w3 = x(17,:);
wB = A_P_AB(2,:);
wC = A_P_AC(2,:);
wD = A_P_AD(2,:);
wE = A_P_AE(2,:);
t2 = cos(q4);
t3 = cos(q5);
t4 = cos(q6);
t5 = cos(q7);
t6 = sin(q4);
t7 = sin(q5);
t8 = sin(q6);
t9 = sin(q7);
t10 = B_I_BB3_1.*w1;
t11 = B_I_BB3_1.*w2;
t12 = B_I_BB3_2.*w1;
t13 = B_I_BB3_2.*w2;
t14 = C_I_CC3_1.*w1;
t15 = C_I_CC3_1.*w2;
t16 = C_I_CC3_2.*w1;
t17 = C_I_CC3_2.*w2;
t18 = D_I_DD3_1.*w1;
t19 = D_I_DD3_1.*w2;
t20 = D_I_DD3_2.*w1;
t21 = D_I_DD3_2.*w2;
t22 = E_I_EE3_1.*w1;
t23 = E_I_EE3_1.*w2;
t24 = E_I_EE3_2.*w1;
t25 = E_I_EE3_2.*w2;
t26 = hB.*w3;
t27 = hC.*w3;
t28 = hD.*w3;
t29 = hE.*w3;
t30 = lB.*w1;
t31 = lC.*w1;
t32 = lD.*w1;
t33 = lE.*w1;
t34 = u4+w3;
t35 = u5+w3;
t36 = u6+w3;
t37 = u7+w3;
t38 = w2.*wB;
t39 = w2.*wC;
t40 = w2.*wD;
t41 = w2.*wE;
t42 = e0.^2;
t43 = e1.^2;
t44 = e2.^2;
t45 = e3.^2;
t46 = w1.^2;
t47 = w2.^2;
t48 = w3.^2;
t49 = e0.*e1.*2.0;
t50 = e0.*e2.*2.0;
t51 = e0.*e3.*2.0;
t52 = e1.*e2.*2.0;
t53 = e1.*e3.*2.0;
t54 = e2.*e3.*2.0;
t55 = B_I_BB1_1.*t2;
t56 = B_I_BB1_2.*t2;
t57 = B_I_BB2_1.*t2;
t58 = B_I_BB2_2.*t2;
t59 = B_I_BB3_1.*t2;
t60 = B_I_BB3_2.*t2;
t61 = C_I_CC1_1.*t3;
t62 = C_I_CC1_2.*t3;
t63 = C_I_CC2_1.*t3;
t64 = C_I_CC2_2.*t3;
t65 = C_I_CC3_1.*t3;
t66 = C_I_CC3_2.*t3;
t67 = D_I_DD1_1.*t4;
t68 = D_I_DD1_2.*t4;
t69 = D_I_DD2_1.*t4;
t70 = D_I_DD2_2.*t4;
t71 = D_I_DD3_1.*t4;
t72 = D_I_DD3_2.*t4;
t73 = E_I_EE1_1.*t5;
t74 = E_I_EE1_2.*t5;
t75 = E_I_EE2_1.*t5;
t76 = E_I_EE2_2.*t5;
t77 = E_I_EE3_1.*t5;
t78 = E_I_EE3_2.*t5;
t79 = B_I_BB1_1.*t6;
t80 = B_I_BB1_2.*t6;
t81 = B_I_BB2_1.*t6;
t82 = B_I_BB2_2.*t6;
t83 = B_I_BB3_1.*t6;
t84 = B_I_BB3_2.*t6;
t85 = C_I_CC1_1.*t7;
t86 = C_I_CC1_2.*t7;
t87 = C_I_CC2_1.*t7;
t88 = C_I_CC2_2.*t7;
t89 = C_I_CC3_1.*t7;
t90 = C_I_CC3_2.*t7;
t91 = D_I_DD1_1.*t8;
t92 = D_I_DD1_2.*t8;
t93 = D_I_DD2_1.*t8;
t94 = D_I_DD2_2.*t8;
t95 = D_I_DD3_1.*t8;
t96 = D_I_DD3_2.*t8;
t97 = E_I_EE1_1.*t9;
t98 = E_I_EE1_2.*t9;
t99 = E_I_EE2_1.*t9;
t100 = E_I_EE2_2.*t9;
t101 = E_I_EE3_1.*t9;
t102 = E_I_EE3_2.*t9;
t103 = t2.*w1;
t104 = t3.*w1;
t105 = t2.*w2;
t106 = t4.*w1;
t107 = t3.*w2;
t108 = t5.*w1;
t109 = t4.*w2;
t110 = t5.*w2;
t111 = t6.*w1;
t112 = t7.*w1;
t113 = t6.*w2;
t114 = t8.*w1;
t115 = t7.*w2;
t116 = t9.*w1;
t117 = t8.*w2;
t118 = t9.*w2;
t119 = -t11;
t120 = -t13;
t121 = -t15;
t122 = -t17;
t123 = -t19;
t124 = -t21;
t125 = -t23;
t126 = -t25;
t127 = -t52;
t128 = -t53;
t129 = -t54;
t130 = hB.*t46;
t131 = hB.*t47;
t132 = hC.*t46;
t133 = hC.*t47;
t134 = hD.*t46;
t135 = hD.*t47;
t136 = hE.*t46;
t137 = hE.*t47;
t138 = lB.*t47;
t139 = lB.*t48;
t140 = lC.*t47;
t141 = lC.*t48;
t142 = lD.*t47;
t143 = lD.*t48;
t144 = lE.*t47;
t145 = lE.*t48;
t146 = t46.*wB;
t147 = t48.*wB;
t148 = t46.*wC;
t149 = t48.*wC;
t150 = t46.*wD;
t151 = t48.*wD;
t152 = t46.*wE;
t153 = t48.*wE;
t154 = -t43;
t155 = -t44;
t156 = -t45;
t205 = t26+t30;
t206 = t27+t31;
t207 = t28+t32;
t208 = t29+t33;
t209 = t26+t38;
t210 = t27+t39;
t211 = t28+t40;
t212 = t29+t41;
t213 = t30+t38;
t214 = t31+t39;
t215 = t32+t40;
t216 = t33+t41;
t229 = t49+t54;
t230 = t50+t53;
t231 = t51+t52;
t329 = t42+t43+t44+t45;
t157 = -t81;
t158 = -t82;
t159 = -t84;
t160 = -t87;
t161 = -t88;
t162 = -t90;
t163 = -t93;
t164 = -t94;
t165 = -t96;
t166 = -t99;
t167 = -t100;
t168 = -t102;
t169 = -t111;
t170 = -t112;
t171 = -t114;
t172 = -t116;
t173 = t34.*t55;
t174 = t34.*t56;
t175 = t34.*t57;
t176 = t34.*t58;
t177 = t35.*t61;
t178 = t35.*t62;
t179 = t35.*t63;
t180 = t35.*t64;
t181 = t36.*t67;
t182 = t36.*t68;
t183 = t36.*t69;
t184 = t36.*t70;
t185 = t37.*t73;
t186 = t37.*t74;
t187 = t37.*t75;
t188 = t37.*t76;
t189 = t34.*t79;
t190 = t34.*t80;
t191 = t34.*t81;
t192 = t34.*t82;
t193 = t35.*t85;
t194 = t35.*t86;
t195 = t35.*t87;
t196 = t35.*t88;
t197 = t36.*t91;
t198 = t36.*t92;
t199 = t36.*t93;
t200 = t36.*t94;
t201 = t37.*t97;
t202 = t37.*t98;
t203 = t37.*t99;
t204 = t37.*t100;
t217 = t57+t79;
t218 = t58+t80;
t219 = t60+t83;
t220 = t63+t85;
t221 = t64+t86;
t222 = t66+t89;
t223 = t69+t91;
t224 = t70+t92;
t225 = t72+t95;
t226 = t75+t97;
t227 = t76+t98;
t228 = t78+t101;
t232 = t205.*w2;
t233 = t206.*w2;
t234 = t207.*w2;
t235 = t208.*w2;
t238 = t209.*w1;
t239 = t210.*w1;
t240 = t211.*w1;
t241 = t212.*w1;
t248 = t103+t113;
t249 = t104+t115;
t250 = t106+t117;
t251 = t108+t118;
t252 = t213.*w3;
t253 = t214.*w3;
t254 = t215.*w3;
t255 = t216.*w3;
t256 = t49+t129;
t257 = t50+t128;
t258 = t51+t127;
t259 = t229.^2;
t338 = 1.0./t329;
t340 = t42+t45+t154+t155;
t341 = t42+t44+t154+t156;
t342 = t42+t43+t155+t156;
t236 = -t173;
t237 = -t174;
t242 = -t177;
t243 = -t178;
t244 = -t181;
t245 = -t182;
t246 = -t185;
t247 = -t186;
t260 = t55+t157;
t261 = t56+t158;
t262 = t59+t159;
t263 = t61+t160;
t264 = t62+t161;
t265 = t65+t162;
t266 = t67+t163;
t267 = t68+t164;
t268 = t71+t165;
t269 = t73+t166;
t270 = t74+t167;
t271 = t77+t168;
t272 = B_I_BB2_1.*t248;
t273 = B_I_BB2_2.*t248;
t274 = B_I_BB2_3.*t248;
t275 = C_I_CC2_1.*t249;
t276 = C_I_CC2_2.*t249;
t277 = C_I_CC2_3.*t249;
t278 = D_I_DD2_1.*t250;
t279 = D_I_DD2_2.*t250;
t280 = D_I_DD2_3.*t250;
t281 = E_I_EE2_1.*t251;
t282 = E_I_EE2_2.*t251;
t283 = E_I_EE2_3.*t251;
t284 = -t232;
t285 = -t233;
t286 = -t234;
t287 = -t235;
t288 = -t238;
t289 = -t239;
t290 = -t240;
t291 = -t241;
t292 = t105+t169;
t293 = t107+t170;
t294 = t109+t171;
t295 = t110+t172;
t296 = -t252;
t297 = -t253;
t298 = -t254;
t299 = -t255;
t300 = t257.^2;
t301 = t219.*u4.*w1;
t302 = t222.*u5.*w1;
t303 = t225.*u6.*w1;
t304 = t228.*u7.*w1;
t339 = t338.^2;
t344 = t119+t175+t189;
t346 = t120+t176+t190;
t348 = t121+t179+t193;
t350 = t122+t180+t194;
t352 = t123+t183+t197;
t354 = t124+t184+t198;
t356 = t125+t187+t201;
t358 = t126+t188+t202;
t407 = g.*mB.*t338.*t340;
t408 = g.*mC.*t338.*t340;
t409 = g.*mD.*t338.*t340;
t410 = g.*mE.*t338.*t340;
t305 = B_I_BB1_1.*t292;
t306 = B_I_BB1_2.*t292;
t307 = B_I_BB1_3.*t292;
t311 = C_I_CC1_1.*t293;
t312 = C_I_CC1_2.*t293;
t313 = C_I_CC1_3.*t293;
t317 = D_I_DD1_1.*t294;
t318 = D_I_DD1_2.*t294;
t319 = D_I_DD1_3.*t294;
t323 = E_I_EE1_1.*t295;
t324 = E_I_EE1_2.*t295;
t325 = E_I_EE1_3.*t295;
t330 = t262.*u4.*w2;
t331 = t265.*u5.*w2;
t332 = t268.*u6.*w2;
t333 = t271.*u7.*w2;
t343 = t10+t191+t236;
t345 = t12+t192+t237;
t347 = t14+t195+t242;
t349 = t16+t196+t243;
t351 = t18+t199+t244;
t353 = t20+t200+t245;
t355 = t22+t203+t246;
t357 = t24+t204+t247;
t359 = t130+t131+t296;
t360 = t132+t133+t297;
t361 = t134+t135+t298;
t362 = t136+t137+t299;
t363 = t138+t139+t288;
t364 = t140+t141+t289;
t365 = t142+t143+t290;
t366 = t144+t145+t291;
t367 = t146+t147+t284;
t368 = t148+t149+t285;
t369 = t150+t151+t286;
t370 = t152+t153+t287;
t411 = -t407;
t412 = -t408;
t413 = -t409;
t414 = -t410;
t334 = -t330;
t335 = -t331;
t336 = -t332;
t337 = -t333;
t392 = -t6.*(t273-t306);
t394 = -t7.*(t276-t312);
t396 = -t8.*(t279-t318);
t398 = -t9.*(t282-t324);
t399 = -t34.*(t274-t307);
t400 = -t35.*(t277-t313);
t401 = -t36.*(t280-t319);
t402 = -t37.*(t283-t325);
t415 = fB+t411;
t416 = fC+t412;
t417 = fD+t413;
t418 = fE+t414;
t423 = -w2.*(t2.*(t273-t306)+t6.*(t272-t305));
t424 = -w2.*(t3.*(t276-t312)+t7.*(t275-t311));
t425 = -w2.*(t4.*(t279-t318)+t8.*(t278-t317));
t426 = -w2.*(t5.*(t282-t324)+t9.*(t281-t323));
t431 = -w1.*(t392+t2.*(t272-t305));
t432 = -w1.*(t394+t3.*(t275-t311));
t433 = -w1.*(t396+t4.*(t278-t317));
t434 = -w1.*(t398+t5.*(t281-t323));
et1 = t415.*wB+t416.*wC+t417.*wD+t418.*wE+t34.*(-B_I_BB3_3.*w2+B_I_BB1_3.*t6.*t34+B_I_BB2_3.*t2.*t34)+t35.*(-C_I_CC3_3.*w2+C_I_CC1_3.*t7.*t35+C_I_CC2_3.*t3.*t35)+t36.*(-D_I_DD3_3.*w2+D_I_DD1_3.*t8.*t36+D_I_DD2_3.*t4.*t36)+w1.*(A_I_AA2_1.*w3-A_I_AA3_1.*w2)+w2.*(A_I_AA2_2.*w3-A_I_AA3_2.*w2)+w3.*(A_I_AA2_3.*w3-A_I_AA3_3.*w2)+t37.*(-E_I_EE3_3.*w2+E_I_EE1_3.*t9.*t37+E_I_EE2_3.*t5.*t37)+w1.*(t2.*t344-t6.*t346)+w2.*(t2.*t346+t6.*t344)+w1.*(t3.*t348-t7.*t350)+w2.*(t3.*t350+t7.*t348)+w1.*(t4.*t352-t8.*t354)+w2.*(t4.*t354+t8.*t352)+w1.*(t5.*t356-t9.*t358)+w2.*(t5.*t358+t9.*t356)+u4.*w1.*(t2.*t261+t6.*t260);
et2 = -u4.*w2.*(t2.*t260-t6.*t261)+u5.*w1.*(t3.*t264+t7.*t263)-u5.*w2.*(t3.*t263-t7.*t264)+u6.*w1.*(t4.*t267+t8.*t266)-u6.*w2.*(t4.*t266-t8.*t267)+u7.*w1.*(t5.*t270+t9.*t269)-u7.*w2.*(t5.*t269-t9.*t270)-hB.*mB.*t367-hC.*mC.*t368-hD.*mD.*t369-hE.*mE.*t370+mB.*t359.*wB+mC.*t360.*wC+mD.*t361.*wD+mE.*t362.*wE+g.*hB.*mB.*t229.*t338+g.*hC.*mC.*t229.*t338+g.*hD.*mD.*t229.*t338+g.*hE.*mE.*t229.*t338;
et3 = -lB.*t415-lC.*t416-lD.*t417-lE.*t418+t34.*(B_I_BB3_3.*w1-B_I_BB1_3.*t2.*t34+B_I_BB2_3.*t6.*t34)+t35.*(C_I_CC3_3.*w1-C_I_CC1_3.*t3.*t35+C_I_CC2_3.*t7.*t35)+t36.*(D_I_DD3_3.*w1-D_I_DD1_3.*t4.*t36+D_I_DD2_3.*t8.*t36)-w1.*(A_I_AA1_1.*w3-A_I_AA3_1.*w1)-w2.*(A_I_AA1_2.*w3-A_I_AA3_2.*w1)-w3.*(A_I_AA1_3.*w3-A_I_AA3_3.*w1)+t37.*(E_I_EE3_3.*w1-E_I_EE1_3.*t5.*t37+E_I_EE2_3.*t9.*t37)+w1.*(t2.*t343-t6.*t345)+w2.*(t2.*t345+t6.*t343)+w1.*(t3.*t347-t7.*t349)+w2.*(t3.*t349+t7.*t347)+w1.*(t4.*t351-t8.*t353)+w2.*(t4.*t353+t8.*t351)+w1.*(t5.*t355-t9.*t357);
et4 = w2.*(t5.*t357+t9.*t355)+u4.*w1.*(t2.*t218+t6.*t217)-u4.*w2.*(t2.*t217-t6.*t218)+u5.*w1.*(t3.*t221+t7.*t220)-u5.*w2.*(t3.*t220-t7.*t221)+u6.*w1.*(t4.*t224+t8.*t223)-u6.*w2.*(t4.*t223-t8.*t224)+u7.*w1.*(t5.*t227+t9.*t226)-u7.*w2.*(t5.*t226-t9.*t227)+hB.*mB.*t363+hC.*mC.*t364+hD.*mD.*t365+hE.*mE.*t366-lB.*mB.*t359-lC.*mC.*t360-lD.*mD.*t361-lE.*mE.*t362+g.*hB.*mB.*t257.*t338+g.*hC.*mC.*t257.*t338+g.*hD.*mD.*t257.*t338+g.*hE.*mE.*t257.*t338;
mt1 = [t230.*t338.*t415+t230.*t338.*t416+t230.*t338.*t417+t230.*t338.*t418+mB.*t230.*t338.*t359-mB.*t258.*t338.*t367+mB.*t338.*t342.*t363+mC.*t230.*t338.*t360-mC.*t258.*t338.*t368+mC.*t338.*t342.*t364+mD.*t230.*t338.*t361-mD.*t258.*t338.*t369+mD.*t338.*t342.*t365+mE.*t230.*t338.*t362-mE.*t258.*t338.*t370+mE.*t338.*t342.*t366+g.*mA.*t229.*t258.*t339-g.*mA.*t230.*t339.*t340+g.*mA.*t257.*t339.*t342+g.*mB.*t229.*t258.*t339+g.*mB.*t257.*t339.*t342+g.*mC.*t229.*t258.*t339+g.*mC.*t257.*t339.*t342+g.*mD.*t229.*t258.*t339+g.*mD.*t257.*t339.*t342+g.*mE.*t229.*t258.*t339+g.*mE.*t257.*t339.*t342];
mt2 = [-t256.*t338.*t415-t256.*t338.*t416-t256.*t338.*t417-t256.*t338.*t418+mB.*t231.*t338.*t363-mB.*t256.*t338.*t359+mB.*t338.*t341.*t367+mC.*t231.*t338.*t364-mC.*t256.*t338.*t360+mC.*t338.*t341.*t368+mD.*t231.*t338.*t365-mD.*t256.*t338.*t361+mD.*t338.*t341.*t369+mE.*t231.*t338.*t366-mE.*t256.*t338.*t362+mE.*t338.*t341.*t370+g.*mA.*t231.*t257.*t339-g.*mA.*t229.*t339.*t341+g.*mA.*t256.*t339.*t340+g.*mB.*t231.*t257.*t339-g.*mB.*t229.*t339.*t341+g.*mC.*t231.*t257.*t339-g.*mC.*t229.*t339.*t341+g.*mD.*t231.*t257.*t339-g.*mD.*t229.*t339.*t341+g.*mE.*t231.*t257.*t339-g.*mE.*t229.*t339.*t341];
mt3 = [t338.*t340.*t415+t338.*t340.*t416+t338.*t340.*t417+t338.*t340.*t418-g.*mA.*t259.*t339-g.*mA.*t300.*t339-g.*mB.*t259.*t339-g.*mB.*t300.*t339-g.*mC.*t259.*t339-g.*mC.*t300.*t339-g.*mD.*t259.*t339-g.*mD.*t300.*t339-g.*mE.*t259.*t339-g.*mE.*t300.*t339+mB.*t229.*t338.*t367-mB.*t257.*t338.*t363+mB.*t338.*t340.*t359+mC.*t229.*t338.*t368-mC.*t257.*t338.*t364+mC.*t338.*t340.*t360+mD.*t229.*t338.*t369-mD.*t257.*t338.*t365+mD.*t338.*t340.*t361+mE.*t229.*t338.*t370-mE.*t257.*t338.*t366+mE.*t338.*t340.*t362-g.*mA.*t339.*t340.^2;et1+et2;et3+et4];
mt4 = [t301+t302+t303+t304+t334+t335+t336+t337+t399+t400+t401+t402+t423+t424+t425+t426+t431+t432+t433+t434+w1.*(A_I_AA1_1.*w2-A_I_AA2_1.*w1)+w2.*(A_I_AA1_2.*w2-A_I_AA2_2.*w1)+w3.*(A_I_AA1_3.*w2-A_I_AA2_3.*w1)+lB.*mB.*t367+lC.*mC.*t368+lD.*mD.*t369+lE.*mE.*t370-mB.*t363.*wB-mC.*t364.*wC-mD.*t365.*wD-mE.*t366.*wE-g.*lB.*mB.*t229.*t338-g.*lC.*mC.*t229.*t338-g.*lD.*mD.*t229.*t338-g.*lE.*mE.*t229.*t338-g.*mB.*t257.*t338.*wB-g.*mC.*t257.*t338.*wC-g.*mD.*t257.*t338.*wD-g.*mE.*t257.*t338.*wE;t301+t334+t399+t423+t431-tB;t302+t335+t400+t424+t432+tC;t303+t336+t401+t425+t433-tD;t304+t337+t402+t426+t434+tE];
RHS = [mt1;mt2;mt3;mt4];
end
