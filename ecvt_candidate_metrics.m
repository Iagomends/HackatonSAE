function d = ecvt_candidate_metrics(tr,wr,wc,p)
%ECVT_CANDIDATE_METRICS Existing motor, battery and aging laws for candidates.
% Rows: speed J, |I|, Ddot, Pm1, Pm2, Pe1, Pe2, Pbat,
% eta1, eta2, signed I, C-rate, kdeg [1/Ah], Ddot [1/s]. Vectorized.
a=p(1)/p(2); b=1+a; ws=(b*wc-wr)/a;
pm1=(-a*tr).*ws; pm2=(b*tr).*wc;
eta1=p(9)*ones(size(wc)); eta2=eta1;
eta1(pm1<0)=p(10); eta2(pm2<0)=p(10);
pe1=pm1./eta1; pe2=pm2./eta2;
pe1(pm1<0)=pm1(pm1<0).*eta1(pm1<0);
pe2(pm2<0)=pm2(pm2<0).*eta2(pm2<0);
power=pe1+pe2;
disc=p(14)^2-4*p(15)*power;
current=2*power./(p(14)+sqrt(max(disc,0)));
current(disc < -64*eps(p(14)^2))=inf;
crate=abs(current)/p(16);
n=p(17); x=p(18:17+n); rates=p(18+n:17+2*n);
k=interp1(x,rates,min(max(crate,x(1)),x(end)),'linear');
ddot=k.*abs(current)/3600;
ddot(~isfinite(current))=inf;
d=[(ws/p(7)).^2+(wc/p(8)).^2;abs(current);ddot;pm1;pm2; ...
    pe1;pe2;power;eta1;eta2;current;crate;k;ddot];
end
