% When using this file, please cite the following works:
% [1] A. Deibe, J.A. Anton Nacimiento, J. Cardenal, and F. López Peña, "A
%     Kalman Filter for Nonlinear Attitude Estimation Using Time Variable
%     Matrices and Quaternions," Sensors, vol. 20, no. 23, p. 6731, Nov.
%     2020, https://doi.org/10.3390/s20236731
% [2] A. Deibe, J.A. Anton Nacimiento, J. Cardenal, and F. López Peña, "A
%     Time–Varying Kalman Filter for Low–Acceleration Attitude Estimation",
%     Measurement, 2023, https://doi.org/10.1016/j.measurement.2023.112729.
% 
function [errTVKAE]=errEvalTVKAE(stDevIn,IMUType,speedIndex,beta,verbose)
%Evaluates attitude estimation error of a defined manoeuvre.
%
%   [errTVKAE]=errEvalTVKAE(miVar,IMUIndex,speedIndex,beta,verbose)
%   This function uses the TVKAE algorithm to evaluates the attitude
%   estimation error for a defined manoeuvre. For a complete explanation of
%   the TVKAE algorithm, see [1] and [2].
%
% INPUT PARAMETERS:
%   stDevIn: standard deviations input vector:
%     (1): acceleration (a) prediction
%     (2): orientation quaternion (q) prediction
%     (3): rotation quaternion (q_r) prediction
%     (4): accelerometers
%     (5): magnetometers
%     (6): rate-gyros
%   IMUIndex: IMU Type selector. It must be:
%     1: XSense
%     2: APDM
%     3: Shimmer
%   SpeedIndex: Manoeuvre speed selector. It must be:
%     1: Slow
%     2: Medium
%     3: Fast
%   beta: acceleration weight in prediction phase. Any value in [0,1].
%   Verbose: 0 (no info shown) or 1 (final info shown).
%
% OUTPUT PARAMETERS:
%   errorTVKAE: Total manoeuvre attitude error (in degrees)

% For a detailed description of the experimental scenarios, see:
% Caruso et al., "Analysis of the Accuracy of Ten Algorithms for
% Orientation Estimation Using Inertial and Magnetic Sensing under Optimal
% Conditions: One Size Does Not Fit All", Sensors, vol. 21, no. 7, p. 2543,
% Apr. 2021, doi.org/10.3390/s21072543,
% and:
% Caruso et al., "Orientation Estimation Through Magneto-Inertial Sensor
% Fusion: A Heuristic Approach for Suboptimal Parameters Tuning", IEEE
% Sensors Journal, vol. 21, no. 3, pp. 3408-3419, 2021,
% doi.org/10.1109/JSEN.2020.3024806.

T0=clock; % to compute elapsed time

% Input parameter defaults:
if nargin<5 || isempty(verbose),    verbose=1;    end % Verbose: 0 -> no info shown
if nargin<4 || isempty(beta),       beta=0;       end % Acceleration weight in prediction phase
if nargin<3 || isempty(speedIndex), speedIndex=1; end % Index of the manoeuvre speed
if nargin<2 || isempty(IMUType),    IMUType=1;    end % Index of the IMU type
if nargin<1,                        stDevIn=[];   end % Standard deviations

% Standard deviations
stDev=[ % Default standard deviations
   6.8466e-03    % sigma_a    acceleration
   6.3894e-07    % sigma_q    orientation quaternion
   1.0000e-04    % sigma_q_r  rotation quaternion
   9.5778e-04    % sigma_a    accelerometer
   1.0965e-02    % sigma_h    magnetometer
   8.3691e-04]'; % sigma_r    rate-gyro
stDev=[stDevIn,stDev(length(stDevIn)+1:end)]; % Merge with stDev input

% Selects IMU hardware
switch IMUType
    case 1, IMUVendor='XS'; % IMUs de XSens
    case 2, IMUVendor='AP'; % IMUs de APDM
    case 3, IMUVendor='SH'; % IMUs de Shimmer
    otherwise, error('IMU index must be 1 (XSense), 2 (APDM), or 3 (Shimmer).');
end

% Selects manoeuvre speed
switch speedIndex
    case 1, speed='Slow';
    case 2, speed='Medium';
    case 3, speed='Fast';
    otherwise, error('Manoeuvre speed must be 1 (slow), 2 (medium), or 3 (fast).');
end

% Invariant values of gravitational and magnetic fields, and angle between
% them. These values have been computed averaging the first 5000 samples
% for each IMU unit and for each manoeuvre speed.
%     slow   medium   fast
g_matrix=[
     9.8442  9.8435  9.8438   % XS1
     9.7617  9.7537  9.7540   % XS2
    10.0412 10.0269 10.0255   % AP1
     9.9371  9.9376  9.9348   % AP2
     9.8966  9.8974  9.8956   % SH1
     9.6501  9.6479  9.6456]; % SH2
h_matrix=[
     1.0564  1.0560  1.0554   % XS1
     1.0438  1.0432  1.0424   % XS2
     1.1620  1.1618  1.1608   % AP1
     1.1534  1.1551  1.1529   % AP2
     1.0511  1.0464  1.0445   % SH1
     1.1206  1.1224  1.1173]; % SH2
alphaCos_matrix=[
     0.8008  0.8006  0.8007   % XS1
     0.7910  0.7905  0.7905   % XS2
     0.7878  0.7868  0.7866   % AP1
     0.8038  0.8034  0.8035   % AP2
     0.8308  0.8330  0.8325   % SH1
     0.7788  0.7786  0.7797]; % SH2

% Composes the filename and loads the selected data.
% It is expected that "Data" folder contains three files called
% "slow_version.m", "medium_version.m", and "fast_version.m", where "version" 
% could be either "v4" or "v5". These files can be found in Caruso et al.
% repositories, at:
%  - Github: "https://github.com/marcocaruso/mimu_optical_dataset_caruso_sassari/releases/tag/v5.0" or
%  - IEEEDataPort: "https://ieee-dataport.org/documents/mimuopticalsassaridataset".
%
dataFile=strcat('Data/',speed,'_v5.mat'); % Try filename ended in _v5
try
    load(dataFile); % tries to load data file
catch
    dataFile=strcat('Data/',speed,'_v4.mat'); % If _v5 fails, try filename ended in _v4
    try
        load(dataFile); % tries to load data file
    catch
        % if both tries have failed, an explanatory error message is displayed
        message=append('It is expected that the "Data" folder contains three files called\n',...
              '"slow_version.m", "medium_version.m", and "fast_version.m", where "version"\n',...
              'could be either "v4" or "v5". These files can be found in Caruso et al.\n',...
              'repositories, at:\n',...
              '  - Github: "https://github.com/marcocaruso/mimu_optical_dataset_caruso_sassari/releases/tag/v5.0" or\n',...
              '  - IEEEDataPort: "https://ieee-dataport.org/documents/mimuopticalsassaridataset"\n');
        fprintf(1,message)
        error('No data file found');
    end
end

indMov=[indz;indx;indy;indarb]'; % Indexes of steady rest intervals
Dt=0.01; % Sampling period, in s (100 Hz sampling freq.)

% Number of samples in the data
N=size(Qs,1);

% Rotates Qs (Ground Truth) with inicial value of Qs
qGTCor = quatmultiply(quatconj(Qs(1,:)),Qs);
try
    qGTCor = correctQuat(qGTCor);
catch
    % if call to correctQuat has failed, an explanatory error message is displayed
    message=append('Function "correctQuat.m" from Caruso et al. public code should be on\n',...
                   'the same directory as it is errEvalTVKAE.\n',...
                   'It can be donwloaded from Github reposiory at\n',...
                   '"https://github.com/marcocaruso/sensor_fusion_algorithm_codes/tree/main/function_utilities"\n'); 
    fprintf(1,message)
    error('correctQuat.m function needed');
end

% Compute covariances matrix of the KF prediction stage
Q=zeros(10,10);
Q(1:3,1:3)=eye(3)*stDev(1)^2;
Q(4:7,4:7)=eye(4)*stDev(2)^2;
Q(8:10,8:10)=eye(3)*stDev(3)^2;

% Compute covariances matrix of the KF correction stage
R=zeros(9,9);
R(1:3,1:3)=eye(3)*stDev(4)^2; 
R(4:6,4:6)=eye(3)*stDev(5)^2; 
R(7:9,7:9)=eye(3)*stDev(6)^2;

% State vector initialization
X=zeros(10,N);

% Runs selected manoeuvre (speed) with data from each unit of selected IMU
for IMUIndex=1:2

  % Initial KF P matrix
  P=1.0e-010*[ 80254.643423 1.6080489777 -0.0011624103 6.6835590359 -0.1653845733 -6.6045452060 0.8401027824 0 0 0; 1.6080489777 81803.506674 -1.1338277729 -8.8372189154 -144.20966422 17.081030004 410.78535616 0 0 0; -0.0011624103 -1.1338277729 80254.642462 6.6284418023 0.5309872126 6.6797740774 -0.2870266064 0 0 0; 6.6835590359 -8.8372189154 6.6284418023 1.1490260802 0.8196328695 -0.0928682472 -2.1958730860 0 0 0; -0.1653845733 -144.20966422 0.5309872126 0.8196328695 15.791807445 -1.6642549441 -39.351652566 0 0 0; -6.6045452060 17.081030004 6.6797740774 -0.0928682472 -1.6642549441 1.2918546639 4.4586795586 0 0 0; 0.8401027824 410.78535616 -0.2870266064 -2.1958730860 -39.351652566 4.4586795586 106.52981554 0 0 0; 0 0 0 0 0 0 0 1.6291216145 0 0; 0 0 0 0 0 0 0 0 1.6291216145 0; 0 0 0 0 0 0 0 0 0 1.6291216145];

  % Select the correct data set from speed and IMU type
  S=eval(sprintf('%s%d',IMUVendor,IMUIndex));

  % Bias substraction for each sample
  S(:,5:7) = S(:,5:7) - mean(S(1:5000,5:7));

  % Composition of sensors vector
  Z(1:3,:) = S(:,2:4)';  % accelerometers
  Z(4:6,:) = S(:,8:10)'; % magnatometers
  Z(7:9,:) = S(:,5:7)';  % rate-gyros

  % Initial values of sensors vector
  ZIni=Z(1:7,1);

  % Compose the inertial reference system with gravitational and
  % magnetic fields vectors, to change between earth ("e") and body ("b") systems.
  Zeb=-ZIni(1:3)/norm(ZIni(1:3)); % Normalized gravity vector: Z-axis
  Heb=ZIni(4:6)/norm(ZIni(4:6)); % Normalized magnetic field, in XZ plane
  Yeb=cross(Zeb,Heb)/norm(cross(Zeb,Heb)); % Y axis computation
  Xeb=cross(Yeb,Zeb)/norm(cross(Yeb,Zeb)); % X axis computation
  qIniConj=dcm2quat([Xeb,Yeb,Zeb])'; % Initial orientation quaternion

  % Initial state vector
  X(:,1)=[[0 0 0]'; qIniConj; [0 0 0]'];

  % Invariant values
  g=g_matrix(2*(IMUType-1)+IMUIndex,speedIndex); % Gravity
  h=h_matrix(2*(IMUType-1)+IMUIndex,speedIndex); % Magnetic
  alphaCos=alphaCos_matrix(2*(IMUType-1)+IMUIndex,speedIndex); % Angle between gravitational and magnetic fields
  %alphaCos=Zeb'*Heb;
  alphaSin=sqrt(1-alphaCos^2);

  % -----------------------------------------------------------------------
  % Manoeuvre iteration loop. TVKAE estimation.
  % -----------------------------------------------------------------------

  for k=2:N
      [X(:,k),P]=TVKAE(X(:,k-1),Z(:,k),P,Dt,Q,R,g,h,alphaCos,alphaSin,beta);
  end

  % -----------------------------------------------------------------------
  % Formatting of TVKAE output for comparison with Caruso et al. results.
  % -----------------------------------------------------------------------

  % Correct (rotate) estimated orientation quaternions with initial
  % orientation quaternion
  qCor=quatmultiply(quatconj(X(4:7,1)'),X(4:7,:)');

  % Estimation orientation errors for each IMU unit
  err=quatmultiply(quatconj(qCor),qGTCor); % Quaternion of estimation error
  if IMUIndex==1 % Transform error quaternion in angle-vector pair
    axa1=quat2axang(err); % First IMU unit
  else
    axa2=quat2axang(err); % Second IMU unit
  end

end

% Compute the average of angular errors in each iteration in standin-still
% intervals (this computation is made in accordance with Caruso et al.
% procedures, to get comparable outcomes)
errAvg=mean([axa1(indMov,end),axa2(indMov,end)],2);

% Transforms the computed angular error from radians into degrees
errTVKAE=rad2deg(rms(errAvg));

% Outcomes message
if verbose
  fprintf(1,'IMU: %s, Speed: %s,\tError: %f,  Time: %f\n',IMUVendor,speed,errTVKAE,etime(clock,T0))
end

end