% When using this file, please cite the following works:
% [1] A. Deibe, J.A. Anton Nacimiento, J. Cardenal, and F. López Peña, "A
%     Kalman Filter for Nonlinear Attitude Estimation Using Time Variable
%     Matrices and Quaternions," Sensors, vol. 20, no. 23, p. 6731, Nov.
%     2020, https://doi.org/10.3390/s20236731
% [2] A. Deibe, J.A. Anton Nacimiento, J. Cardenal, and F. López Peña, "A
%     Time–Varying Kalman Filter for Low–Acceleration Attitude Estimation",
%     Measurement, 2023, https://doi.org/10.1016/j.measurement.2023.112729.
% 
function [X,P]=TVKAE(X,Z,P,Dt,Q,R,g,h,alphaCos,alphaSin,beta)
%Time Varying Kalman Filter for Attitude Estimation
%
%   [X,P]=TVKAE(X,Z,P,Dt,Q,R,g,h,alphaCos,alphaSin,beta) Performs an
%   estimation of the attitude of a solid body using a Time Varying Kalman
%   Filter formulation. The attitude estimation in time step i+1 is
%   computed from state vector X on time step i, and sensors data in time
%   step i+1.
%
%   For a complete explanation of the TVKAE algorithm, see [1] and [2].
%
%   INPUT PARAMETERS:
%     X: state vector in time step i
%     Z: IMU sensor data in step i+1
%     P: covariance matrix in time step i
%     Q, R: system and sensors covariance matrices in time step i
%     Dt: time step
%     g: local gravity value
%     h: local magnetic field value
%     alphaSin, alphaCos: respectively, sin and cos of angle betweeen
%        gravity and magnetic fields
%     beta: acceleration weight coefficient in prediction stage
%
%   OUTPUT PARAMETERS:
%     X: estimated state vector in time step i+1
%     P: Updated covariance matrix in time step i+1

% Compute prediction matrix Phi
x=X(8); y=X(9); z=X(10); w=sqrt(1-x^2-y^2-z^2); % q_r
Phi(1:3,1:3)=beta*eye(3);
Phi(4:7,4:7)=[w -x -y -z; ...
            x  w  z -y; ...
            y -z  w  x; ...
            z  y -x  w];
Phi(8:10,8:10)=eye(3);

% TIME UPDATE
X_=Phi*X;
P_=Phi*P*Phi'+Q;

% Compute H matrix
w=X(4); x=X(5); y=X(6); z=X(7); % Orientation quaternion components
H=zeros(9,10);
H(1:3,1:3)= [ 1-2*(y^2+z^2)    2*x*y+2*w*z    2*x*z-2*y*w ;
                2*x*y-2*w*z  1-2*(x^2+z^2)    2*w*x+2*y*z ;
                2*x*z+2*w*y   -2*w*x+2*y*z  1-2*(x^2+y^2) ];
H(1:3,4:7)=-g*[-y  z -w  x ;
                x  w  z  y ;
                w -x -y  z ];
H(4:6,4:7)=h*( ...
  alphaSin*[  w,  x, -y, -z;
             -z,  y,  x, -w;
              y,  z,  w,  x] + ...
  alphaCos*[ -y,  z, -w,  x;
              x,  w,  z,  y;
              w, -x, -y,  z]);
H(7:9,8:10)=2/Dt*eye(3);

% Compute Kalman matrix
K=P_*H'/(H*P_*H'+R);

% MEASUREMENT UPDATE
X=X_+K*(Z-H*X_);

% NORMALIZATION
X(4:7)=X(4:7)/norm(X(4:7));

% Covariance matrix update
P=(eye(10)-K*H)*P_;

end