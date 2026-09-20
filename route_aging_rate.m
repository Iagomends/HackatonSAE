function loss = route_aging_rate(current,p)
%ROUTE_AGING_RATE Existing clipped piecewise-linear loss/Ah times throughput.
n=p(17); x=p(18:17+n); k=p(18+n:17+2*n);
C=abs(current)/p(16);
loss=interp1(x,k,min(max(C,x(1)),x(end)),'linear').*abs(current)/3600;
end
