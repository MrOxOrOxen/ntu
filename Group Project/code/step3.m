function design = step3(observer, baseline)
% Reference feedforward using the LQR gain selected in Step 2.
if nargin<1, [~,observer] = step1; end
if nargin<2, baseline = lqr_design(observer); end
A = observer.A;
B = observer.B;
Ctheta = observer.C(1,:);
K = baseline.K;
steady = [A B; Ctheta 0] \ [zeros(4,1); 1];
Nx = steady(1:4);
Nu = steady(5);
Nbar = Nu + K*Nx;
poles = eig(A-B*K);
dcGain = -Ctheta*((A-B*K)\(B*Nbar));
assert(all(real(poles)<0), 'Reference-tracking regulator is unstable.');
assert(abs(dcGain-1)<1e-10, 'Reference prefilter must give unit DC gain.');
design = struct('Q',baseline.Q,'R',baseline.R,'Kx',K, ...
    'Nx',Nx,'Nu',Nu,'Nbar',Nbar,'poles',poles,'dcGain',dcGain, ...
    'voltageLimit',12);
fprintf('STEP3_DESIGN_OK | K = %s | Nbar = %.8f | DC gain = %.12f\n', ...
    mat2str(K,8),Nbar,dcGain);
end
