% CHECK_ECVT_LIMITS Validate all objectives against independent dense sweeps.
initialize_model;
requests = [1e5 0; 1e5 500; -1e5 500; 0 15000; 100 15000; ...
    -100 -15000; 300 1000; -300 -1000; 0 0; 100 200; -100 200; ...
    100 -200; -100 -200; 300 0];
count=0;
for strategy=1:3
  for regenFraction=[1 0.35]
    p=ecvt_parameters; p(13)=strategy; p(11:12)=regenFraction;
    for k=1:size(requests,1)
      q=ecvt_operating_point(requests(k,1),requests(k,2),p);
      a=p(1)/p(2); b=1+a;
      assert(abs(a*q(4)+q(2)-b*q(5))<1e-8);
      assert(abs(q(8)+q(9)-q(3))<1e-6);
      f1=1; f2=1;
      if q(8)<-1e-8, f1=p(11); end
      if q(9)<-1e-8, f2=p(12); end
      assert(abs(q(6))<=p(3)*f1+1e-7 && abs(q(7))<=p(4)*f2+1e-7);
      assert(abs(q(4))<=p(7)+1e-7 && abs(q(5))<=p(8)+1e-7);
      assert(abs(q(8))<=p(5)*f1+1e-6 && abs(q(9))<=p(6)*f2+1e-6);
      if k<=6, assert(q(14)==0,'Impossible requests must be flagged.'); end
      % Sweep independently evaluates electrical/current/lookup laws.
      sweep=linspace(-p(8),p(8),40001); sun=(b*sweep-q(2))/a;
      pm1=q(6)*sun; pm2=q(7)*sweep;
      f1=ones(size(sweep)); f2=f1;
      f1(pm1<0)=p(11); f2(pm2<0)=p(12);
      pe1=pm1/p(9); pe2=pm2/p(9);
      pe1(pm1<0)=pm1(pm1<0)*p(10); pe2(pm2<0)=pm2(pm2<0)*p(10);
      pb=pe1+pe2; disc=p(14)^2-4*p(15)*pb;
      valid=abs(sun)<=p(7) & abs(q(6))<=p(3)*f1 & ...
          abs(q(7))<=p(4)*f2 & abs(pm1)<=p(5)*f1 & abs(pm2)<=p(6)*f2 & disc>=0;
      current=2*pb(valid)./(p(14)+sqrt(disc(valid)));
      cr=abs(current)/p(16);
      kd=interp1(degradation_Crate,degradation_rate, ...
          min(max(cr,degradation_Crate(1)),degradation_Crate(end)));
      objectives=[(sun(valid)/p(7)).^2+(sweep(valid)/p(8)).^2; ...
          abs(current); kd.*abs(current)/3600];
      if any(valid)
          minimum=min(objectives(strategy,:));
          assert(q(13)<=minimum+1e-11*max(abs(minimum),1e-12));
      end
      assert(abs(ecvt_electrical_power([q(1:2);p(:)])-q(12))<1e-5);
      assert(abs(q(12)-(p(14)-p(15)*q(21))*q(21))<1e-6);
      assert(abs(q(24)-q(23)*abs(q(21))/3600)<1e-18);
      count=count+1;
    end
  end
end
% Finite current at R=0, and clipped lookup tails.
p=ecvt_parameters; p(15)=0; p(16)=0.1;
q=ecvt_operating_point(100,200,p);
assert(abs(q(21)-q(12)/p(14))<1e-10);
assert(q(23)==degradation_rate(end));
fprintf('PASS: %d constrained requests, dense sweeps, zero resistance and lookup clipping.\n',count);
