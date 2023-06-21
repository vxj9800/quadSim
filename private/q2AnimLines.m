function [x,y,z] = q2AnimLines(xi,A_P_AB,A_P_AC,A_P_AD,A_P_AE,pDia)
%q2AnimLines
%    [X,Y,Z] = q2AnimLines(IN1,IN2,IN3,IN4,IN5,pDia)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    12-Jun-2023 15:32:30

%    xi = [qi,ui] -> i'th state values
%    A_P_AB = [lB;wB;hB]
%    A_P_AC = [lC;wC;hC]
%    A_P_AD = [lD;wD;hD]
%    A_P_AE = [lE;wE;hE]
%    pDia = Propeller Diameter
e0 = xi(4,:);
e1 = xi(5,:);
e2 = xi(6,:);
e3 = xi(7,:);
hB = A_P_AB(3,:);
hC = A_P_AC(3,:);
hD = A_P_AD(3,:);
hE = A_P_AE(3,:);
lB = A_P_AB(1,:);
lC = A_P_AC(1,:);
lD = A_P_AD(1,:);
lE = A_P_AE(1,:);
q1 = xi(1,:);
q2 = xi(2,:);
q3 = xi(3,:);
q4 = xi(8,:);
q5 = xi(9,:);
q6 = xi(10,:);
q7 = xi(11,:);
wB = A_P_AB(2,:);
wC = A_P_AC(2,:);
wD = A_P_AD(2,:);
wE = A_P_AE(2,:);
t2 = conj(pDia);
t3 = cos(q4);
t4 = cos(q5);
t5 = cos(q6);
t6 = cos(q7);
t7 = sin(q4);
t8 = sin(q5);
t9 = sin(q6);
t10 = sin(q7);
t11 = e0.^2;
t12 = e1.^2;
t13 = e2.^2;
t14 = e3.^2;
t15 = e0.*e1.*2.0;
t16 = e0.*e2.*2.0;
t17 = e0.*e3.*2.0;
t18 = e1.*e2.*2.0;
t19 = e1.*e3.*2.0;
t20 = e2.*e3.*2.0;
t21 = -t18;
t22 = -t19;
t23 = -t20;
t24 = -t12;
t25 = -t13;
t26 = -t14;
t27 = (t2.*t3)./2.0;
t28 = (t2.*t4)./2.0;
t29 = (t2.*t5)./2.0;
t30 = (t2.*t6)./2.0;
t31 = (t2.*t7)./2.0;
t32 = (t2.*t8)./2.0;
t33 = (t2.*t9)./2.0;
t34 = (t2.*t10)./2.0;
t43 = t15+t20;
t44 = t16+t19;
t45 = t17+t18;
t49 = t11+t12+t13+t14;
t35 = -t27;
t36 = -t28;
t37 = -t29;
t38 = -t30;
t39 = -t31;
t40 = -t32;
t41 = -t33;
t42 = -t34;
t46 = t15+t23;
t47 = t16+t22;
t48 = t17+t21;
t50 = 1.0./t49;
t51 = t11+t14+t24+t25;
t52 = t11+t13+t24+t26;
t53 = t11+t12+t25+t26;
t54 = q1.*t44.*t50;
t55 = q2.*t45.*t50;
t56 = q3.*t43.*t50;
t57 = q1.*t48.*t50;
t58 = q2.*t46.*t50;
t59 = q3.*t47.*t50;
t63 = q1.*t50.*t53;
t64 = q2.*t50.*t52;
t65 = q3.*t50.*t51;
t60 = -t57;
t61 = -t58;
t62 = -t59;
t66 = t54+t61+t65;
t67 = t55+t62+t63;
t68 = t56+t60+t64;
t69 = hB+t66;
t70 = hC+t66;
t71 = hD+t66;
t72 = hE+t66;
t73 = lB+t67;
t74 = lC+t67;
t75 = lD+t67;
t76 = lE+t67;
t77 = t68+wB;
t78 = t68+wC;
t79 = t68+wD;
t80 = t68+wE;
t97 = t43.*t50.*t68;
t98 = t44.*t50.*t66;
t99 = t45.*t50.*t67;
t112 = t46.*t50.*t66;
t113 = t47.*t50.*t67;
t114 = t48.*t50.*t68;
t142 = t50.*t51.*t66;
t143 = t50.*t53.*t67;
t144 = t50.*t52.*t68;
t81 = t27+t73;
t82 = t28+t74;
t83 = t29+t75;
t84 = t30+t76;
t85 = t31+t77;
t86 = t32+t78;
t87 = t33+t79;
t88 = t34+t80;
t89 = t35+t73;
t90 = t36+t74;
t91 = t37+t75;
t92 = t38+t76;
t93 = t39+t77;
t94 = t40+t78;
t95 = t41+t79;
t96 = t42+t80;
t100 = t44.*t50.*t69;
t101 = t44.*t50.*t70;
t102 = t44.*t50.*t71;
t103 = t44.*t50.*t72;
t104 = t45.*t50.*t73;
t105 = t45.*t50.*t74;
t106 = t45.*t50.*t75;
t107 = t45.*t50.*t76;
t108 = t43.*t50.*t77;
t109 = t43.*t50.*t78;
t110 = t43.*t50.*t79;
t111 = t43.*t50.*t80;
t115 = t46.*t50.*t69;
t116 = t46.*t50.*t70;
t117 = t46.*t50.*t71;
t118 = t46.*t50.*t72;
t119 = t47.*t50.*t73;
t120 = t47.*t50.*t74;
t121 = t47.*t50.*t75;
t122 = t47.*t50.*t76;
t123 = t48.*t50.*t77;
t124 = t48.*t50.*t78;
t125 = t48.*t50.*t79;
t126 = t48.*t50.*t80;
t127 = -t112;
t128 = -t113;
t129 = -t114;
t145 = t50.*t51.*t69;
t146 = t50.*t51.*t70;
t147 = t50.*t51.*t71;
t148 = t50.*t51.*t72;
t149 = t50.*t53.*t73;
t150 = t50.*t53.*t74;
t151 = t50.*t53.*t75;
t152 = t50.*t53.*t76;
t153 = t50.*t52.*t77;
t154 = t50.*t52.*t78;
t155 = t50.*t52.*t79;
t156 = t50.*t52.*t80;
t130 = -t115;
t131 = -t116;
t132 = -t117;
t133 = -t118;
t134 = -t119;
t135 = -t120;
t136 = -t121;
t137 = -t122;
t138 = -t123;
t139 = -t124;
t140 = -t125;
t141 = -t126;
t157 = t97+t128+t142;
t158 = t98+t129+t143;
t159 = t99+t127+t144;
t160 = t104+t127+t153;
t163 = t105+t127+t154;
t166 = t106+t127+t155;
t169 = t107+t127+t156;
t161 = t108+t134+t142;
t162 = t98+t138+t149;
t164 = t109+t135+t142;
t165 = t98+t139+t150;
t167 = t110+t136+t142;
t168 = t98+t140+t151;
t170 = t111+t137+t142;
t171 = t98+t141+t152;
x = reshape([t158,t162,t158,t165,t158,t168,t158,t171,t162,t100+t138+t149,t165,t101+t139+t150,t168,t102+t140+t151,t171,t103+t141+t152,t100-t48.*t50.*t85+t50.*t53.*t81,t100-t48.*t50.*t93+t50.*t53.*t89,t101-t48.*t50.*t86+t50.*t53.*t82,t101-t48.*t50.*t94+t50.*t53.*t90,t102-t48.*t50.*t87+t50.*t53.*t83,t102-t48.*t50.*t95+t50.*t53.*t91,t103-t48.*t50.*t88+t50.*t53.*t84,t103-t48.*t50.*t96+t50.*t53.*t92],[2,12]);
if nargout > 1
    y = reshape([t159,t160,t159,t163,t159,t166,t159,t169,t160,t104+t130+t153,t163,t105+t131+t154,t166,t106+t132+t155,t169,t107+t133+t156,t130+t45.*t50.*t81+t50.*t52.*t85,t130+t45.*t50.*t89+t50.*t52.*t93,t131+t45.*t50.*t82+t50.*t52.*t86,t131+t45.*t50.*t90+t50.*t52.*t94,t132+t45.*t50.*t83+t50.*t52.*t87,t132+t45.*t50.*t91+t50.*t52.*t95,t133+t45.*t50.*t84+t50.*t52.*t88,t133+t45.*t50.*t92+t50.*t52.*t96],[2,12]);
end
if nargout > 2
    z = reshape([t157,t161,t157,t164,t157,t167,t157,t170,t161,t108+t134+t145,t164,t109+t135+t146,t167,t110+t136+t147,t170,t111+t137+t148,t145+t43.*t50.*t85-t47.*t50.*t81,t145+t43.*t50.*t93-t47.*t50.*t89,t146+t43.*t50.*t86-t47.*t50.*t82,t146+t43.*t50.*t94-t47.*t50.*t90,t147+t43.*t50.*t87-t47.*t50.*t83,t147+t43.*t50.*t95-t47.*t50.*t91,t148+t43.*t50.*t88-t47.*t50.*t84,t148+t43.*t50.*t96-t47.*t50.*t92],[2,12]);
end
end