% When using this file, please cite the following works:
% [1] A. Deibe, J.A. Anton Nacimiento, J. Cardenal, and F. López Peña, "A
%     Kalman Filter for Nonlinear Attitude Estimation Using Time Variable
%     Matrices and Quaternions," Sensors, vol. 20, no. 23, p. 6731, Nov.
%     2020, https://doi.org/10.3390/s20236731
% [2] A. Deibe, J.A. Anton Nacimiento, J. Cardenal, and F. López Peña, "A
%     Time–Varying Kalman Filter for Low–Acceleration Attitude Estimation",
%     Measurement, 2023, https://doi.org/10.1016/j.measurement.2023.112729.
% 
% This script evaluates the TVKAE attitude estimation error in nine
% different experimental scenarios. The computed errors are in Deibe et
% al. [2]. For a complete explanation of the TVKAE algorithm, see also [1].
%
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

% Input parameters:
%                        +----------------------> stDevIn
%                        |            +---------> IMUIndex
%                        |            | +-------> SpeedIndex
%                        |            | | +-----> Beta
%              ----------+----------  | | | +---> Verbose
%errEvalTVKAE([5.1174e-03 6.0706e-07],1,1,0,1);

% IMU XSens
errEvalTVKAE([5.1174e-03 6.0706e-07],1,1,0,1); % Slow
errEvalTVKAE([1.3953e-02 8.9142e-07],1,2,0,1); % Medium
errEvalTVKAE([1.5597e-03 2.6590e-08],1,3,0,1); % Fast

% IMU APDM
errEvalTVKAE([1.2328e+00 3.8275e-04],2,1,0,1); % Slow
errEvalTVKAE([5.0831e+00 4.7211e-05],2,2,0,1); % Medium
errEvalTVKAE([7.8359e-10 1.2079e-07],2,3,0,1); % Fast

% IMU Shimmer
errEvalTVKAE([6.5144e-02 1.2651e-05],3,1,0,1); % Slow
errEvalTVKAE([2.5799e-10 9.7534e-08],3,2,0,1); % Medium
errEvalTVKAE([4.8094e-02 9.7584e-06],3,3,0,1); % Fast
