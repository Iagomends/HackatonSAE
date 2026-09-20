function power = ecvt_electrical_power(u)
%ECVT_ELECTRICAL_POWER Decode the e-CVT equivalent ring mechanical interface.
% Same shared operating point, reconstructed from delivered ring torque/speed.
point = ecvt_operating_point(u(1),u(2),u(3:end));
power = point(12);
end
