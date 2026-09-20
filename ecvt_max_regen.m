function q = ecvt_max_regen(tr,wr,p)
%ECVT_MAX_REGEN Maximum achievable charging over torque fraction AND speed.
% At braking, use variables t=abs(Tr), z=Pm2. Pm1=-abs(wr)*t-z.
% Every sign region is a bounded 2-D polytope: enumerate its vertices exactly.
% Scaling a selected vertex's torque/powers preserves speeds and feasibility.
a=p(1)/p(2); b=1+a; v=abs(wr); bestPower=0; bestT=0; bestZ=0;
assert(tr*wr<=0,'Maximum regeneration requires a braking request.');
for s1=[-1 1]
    for s2=[-1 1]
        f1=1; f2=1; g1=1/p(9); g2=1/p(9);
        if s1<0, f1=p(11); g1=p(10); end
        if s2<0, f2=p(12); g2=p(10); end
        tmax=min([abs(tr),p(3)*f1/a,p(4)*f2/b]);
        A=[-1 0;1 0;s1*v s1;0 -s2; ...
            -v -1;v 1;0 1;0 -1; ...
            -v-a*p(7) -1;v-a*p(7) 1;-b*p(8) 1;-b*p(8) -1];
        h=[0;tmax;0;0;p(5)*f1;p(5)*f1;p(6)*f2;p(6)*f2;0;0;0;0];
        objective=[-g1*v g2-g1];
        for i=1:size(A,1)-1
            for j=i+1:size(A,1)
                pair=A([i j],:);
                if abs(det(pair))<1e-12, continue; end
                x=pair\h([i j]);
                if all(A*x<=h+1e-7) && objective*x<bestPower
                    bestPower=objective*x; bestT=x(1); bestZ=x(2);
                end
            end
        end
    end
end
if bestT>1e-10
    delivered=-sign(wr)*bestT; wc=bestZ/(b*delivered);
    d=ecvt_candidate_metrics(delivered,wr,wc,p);
    ws=(b*wc-wr)/a;
    q=[delivered;wr;delivered*wr;ws;wc;-a*delivered;b*delivered; ...
        d(4:8);d(3);1;tr;wr;tr-delivered;0;d(9:14)];
else
    q=ecvt_operating_point(0,wr,p);
end
end
