# TVKAE
Repository containing the MATLAB open source code for a Kalman sensor fusion algorithm based on quaternions 
and time-varying matrices (TVKAE), for estimating the orientation from inertial and magnetic sensing.

Authors: Alvaro Deibe, Jose A. Anton Nacimiento, Jesus Cardenal and Fernando López Peña.
Universidade da Coruña, Spain; 

Last modified: 01/04/2022

The TVKAE is described and discussed in: 

A. Deibe, J.A. Anton Nacimiento, J. Cardenal, and F. López Peña, 
“A Kalman Filter for Nonlinear Attitude Estimation Using Time Variable Matrices and Quaternions,” 
Sensors, vol. 20, no. 23, p. 6731, Nov. 2020 [Online]. 
Available: https://www.mdpi.com/1424-8220/20/23/6731

The extended version, tailored for low-acceleration applications, and including a weight in the acceleration upgrade, comes in:

A. Deibe, J.A. Anton Nacimiento, J. Cardenal, and F. López Peña, 
"A Time–Varying Kalman Filter for Low–Acceleration Attitude Estimation,"
Submitted to IEEE Transactions on Instrumentation and Measurements, 2022

If you use information from this repository, you must cite those documents accordingly.

The data used to perform the evaluation of the estimator presented in this last document were taken from 
version 4 of the files in the Caruso et al. repositories, at:

  - Github: "https://github.com/marcocaruso/mimu_optical_dataset_caruso_sassari" or at
  - IEEEDataPort: "https://ieee-dataport.org/documents/mimuopticalsassaridataset".

In order for the programs to work with these data, it is necessary that this folder contains the function 
"correctQuat.m", available in the GitHub repository.

# This repository contains the following folders and script.

- /Data: This directory must contain the data and scripts that feed the input for TVKAE to work.
- TVKAE.m: MATLAB code for de author's TVKF Attitude Estimator (TVKAE). 
- errEvalTVKAE.m: Matlab code for evaluation of the attitude estimation error according to Caruso et al. method.
- verification.m:  Matlab code for calculating the TVKAE error with beta=0 (TV0) in the nine scenarios defined by Caruso et al. 
This error evaluation follows Caruso et al. proposed method.
