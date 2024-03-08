% Dynamic identification of the 2R robot in horizontal plane.
%
% Input data: a mat file, with the following information (all vectors
% must have the same length).
% t: time in s, with an assumed constant time interval between samples.
% q1, q2: encoder positions in radians. Position is that of the arm, so
%         if the encoder is on the motor output shaft, the gear ratio 
%         must have been taken into account already.
% Gam1, Gam2: motor torques at rotor, assumed to have been 
%             corrected for gear efficiency in case of real data
%             Must be multiplied by gear ratio.
% 
% Notations:
%   ns         : number of time samples in data file
%   q1, q2     : arm positions in radians.
%   q1f, q2f   : low pass filtered arm positions in radians
%   q1d, q2d   : 1st order derivatives of q1, q2, calculated from q1f,q2f.
%   q1dd, q2dd : 2nd order derivatives of q1, q2, calculated from q1f,q2f.
%   Gam1, Gam2 : torques applied to links, in Nm.

% Inertial and friction parameters. 
global m1 m2 I1 I2 c1 c2 n1 n2 Ia1 Ia2 l1 g Fs1 off1 Fs2 off2 Fv1 Fv2
global zz1 zz2 mx1 mx2

% Set g to zero when testing horizontal motion.
g = 0.0 ;   % g = 9.81

% Inertial parameters (nominal values from SolidWorks).
% Friction parameters have no nominal value, but they need to be
% initialized. Init value is zero.

m1 = 7.1 ;
m2 = 3.18 ;
I1 = 0.0237 ;
I2 = 0.0359 ;
c1 = 0.066 ;
c2 = 0.0825 ;
n1 = 15 ;
n2 = 15 ;
Ia1 = 38e-6 ;
Ia2 = 38e-6 ;
l1 = 0.28 ;

Fs1 = 0.0 ;  off1 = 0.0 ;  Fv1 = 0.0 ;
Fs2 = 0.0 ;  off2 = 0;0 ;  Fv2 = 0.0 ;

zz1 = I1 + m1*c1^2 ;
zz2 = I2 + m2*c2^2 ;
mx1 = m1*c1 ;
mx2 = m2*c2 ;

% Identification tunable parameters

removeTime = 0.20 ;          % Parts of signal affected by filter transient

fname = uigetfile('*.mat');
eval(['load ',fname]) ;

Gam1 = n1*Gam1 ;
Gam2 = n2*Gam2 ;

% Sampling period and sampling frequency.
ns = length(t) ;
Ts = (t(ns)-t(1))/(ns-1) ;
Fs = 1/Ts ;

disp('----------------------------------------------------------------');
disp(['File: ',fname]);
disp(['Sampling frequency: ',num2str(Fs)]);
disp('----------------------------------------------------------------');

% Butterworth filter coefficients for order 4, 20 Hz cutting frequency.
CuttingFreq = 20 ;  % Hz
Wn = CuttingFreq / (Fs/2) ;
[B,A] = butter(4,Wn) ;

% Apply low pass filter to q1enc and q2enc, using zero-phase forward/reverse filter.
q1f=filtfilt(B,A,q1);
q2f=filtfilt(B,A,q2);


% Calculate derivatives by centered difference. Pad with artificial 
% zero values to maintain vector length.

q1d  = [0 ; (q1f(3:ns)-q1f(1:ns-2))/(2*Ts) ; 0 ] ;
q2d  = [0 ; (q2f(3:ns)-q2f(1:ns-2))/(2*Ts) ; 0 ] ;
q1dd = [0 ; (q1d(3:ns)-q1d(1:ns-2))/(2*Ts) ; 0 ] ;
q2dd = [0 ; (q2d(3:ns)-q2d(1:ns-2))/(2*Ts) ; 0 ] ;

% Regressor W in horizontal plane. MXa not present.

W13 = l1*( 2*cos(q2f).*q1dd + cos(q2f).*q2dd ...
          - sin(q2f).*q2d.*q2d - 2*sin(q2f).*q1d.*q2d ) ...
          + g*cos(q1f+q2f) ;
W23 = l1*( cos(q2f).*q1dd + sin(q2f).*q1d.*q1d ) ...
          + g*cos(q1f+q2f) ;
      
W1 = [    q1dd     , q1dd+q2dd , W13 , zeros(ns,1) ,  sign(q1d)  , ones(ns,1)   ,   q1d       , zeros(ns,1) , zeros(ns,1) , zeros(ns,1) ] ;
W2 = [ zeros(ns,1) , q1dd+q2dd , W23 ,  n2^2*q2dd  , zeros(ns,1) , zeros(ns,1)  , zeros(ns,1) ,   sign(q2d) ,  ones(ns,1) ,    q2d      ] ;


% We now apply a decimate filer on the columns of W to keep less lines
% and apply a low pass filter to the columns. The subsamplig parameter
% R in decimate(X,R) is set to a the cutting frequency used above.
% See help of the decimate function for the formula below. 
% R = round(0.8*Fs/2/CuttingFreq) ; Here the value is 50.
% For values above 13, it is recommended to decimate in two or more steps.
% I have yet to determine the two decimation factors automatically, so
% the code below is OK for an overall R = 50, by decimating by 10 and 5.

% Decimate W and Gamma. Set Chebyshev cut-off frequency to the cut-off 
% frequency used to filter position.
% Note: that's when the torque gets low-pass filtered.

% Decimate data corresponding to joint 1
R = 50 ;
W1dec = [] ;
W1dec2 = [] ;
for j = 1 : size(W1,2)
    W1dec = [W1dec , decimate(decimate(W1(:,j),10),5) ] ;
    W1dec2 = [W1dec2,decimate(W1(:,j),50) ] ;
end
Gam1dec = decimate(decimate(Gam1,10),5) ;

% Decimate data corresponding to joint 2
W2dec = [] ;
for j = 1 : size(W2,2)
    W2dec = [ W2dec , decimate(decimate(W2(:,j),10),5) ] ;
end
Gam2dec = decimate(decimate(Gam2,10),5) ;


% Remove that part of the data which is affected by the transient 
% response of the Butteworth filter and Chebyshev filter used by decimate.

% Number of samples to remove, at original sampling rate.
% removeTime has been determined experimentally, but in theory, it is
% possible to determine it by analyzing the poles of the Butterworth and
% Chebyshev filters.
remSamples = round(removeTime/Ts);
% Now at decimated sampling rate:
remSamples = ceil(remSamples/R) ;

istart = 1 + remSamples ;
iend   = size(Gam1dec,1) - remSamples ;
Gam1dec  = Gam1dec (istart:iend)   ;   Gam2dec  = Gam2dec (istart:iend) ; 
W1dec    = W1dec   (istart:iend,:) ;   W2dec    = W2dec   (istart:iend,:) ;

% Regroup all equations into a single system. Now that all time filtering
% has been done, we can do it.
Wdec = [ W1dec ; W2dec ] ;
Gamdec = [ Gam1dec ; Gam2dec ] ;

% Now remove equations corresponding to a speed below a certain 
% threshold. Indeed, the Coulomb friction model is known not to be 
% accurate at low speed. Also, at low speed, sign(qd) may spark...
% (and it does).
speed_thresh = 0.1 ; % rd/s
% Filtered matrix/vector
Wf = [] ;
Gamf = [] ;
q1d_dec = decimate(q1d,R) ; 
q2d_dec = decimate(q2d,R) ;

for i = 1 : size(q1d_dec,1)
    if abs(q1d_dec(i))>speed_thresh && abs(q2d_dec(i))>speed_thresh
        Wf = [Wf;Wdec(i,:)] ;
        Gamf = [Gamf;Gamdec(i)];
    end
end

Ja = zz1 + n1^2*Ia1 + l1^2*m2 ;

Knominal = [Ja   ; zz2 ; mx2 ; Ia2 ; Fs1 ; off1 ; Fv1 ; Fs2 ; off2; Fv2] ;
parNames = ["Ja ";"zz2";"mx2";"Ia2";"Fs1";"of1" ;"Fv1";"Fs2";"of2";"Fv2"];

% Here we do not analyze possible dependency relations between parameters.
% We are expected to be working with minimal (independent) parameters.
% It is the case here.

% Solve overdetermined system:

Kident = pinv(Wdec) * Gamdec ;

% We calculate here the estimation error of the parameters.
% Typically, those identified with a high percentage of error are 
% parameters that contribute little to the torque and are later discarded.

sigma_rho = norm(Gamdec-Wdec*Kident)/sqrt(size(Wdec,1)-size(Wdec,2));
cov_mat = sigma_rho^2 * inv(Wdec.'*Wdec) ;
disp(' ');
disp('Identification with all independent parameters.');
disp('Beware: for parameters close to 0, rel. sigma may not make much sense...');
disp('Nominal values for friction parameters are unknown.');
disp(' ');
disp('Param        Nominal       Identified         sigma    rel. sigma (%)');
disp(' ');
for i = 1 : length(parNames)
    s = sprintf(' %s    %12.6g    %12.6g    %12.6g       %5.1f',...
        parNames(i),Knominal(i),Kident(i),sqrt(cov_mat(i,i)),sqrt(cov_mat(i,i))/abs(Kident(i))*100);
    disp(s);
end
disp(' '); disp(['Condition number of Wdec: ',num2str(cond(Wdec))]); disp(' ');


% Now based on above results, you have to decide which parameters are
% worth keeping in the model. To eliminate the others, you need to 
% eliminate the corresponding columns of W and the corresponding number
% of terms in K.

% ...

% Then use the identified parameters to create the (simplified) model of
% the robot. Use it to calculate the torque, and compare to the measured
% torque. Do that on the identification data fifthorder_new.mat, and 
