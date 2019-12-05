%% run_imu_inverse_kinematics
% Script that sets up and runs IMU inverse kinematics.
% The methods are located in the Matlab Class orientationTracker.m
% Data has to be first converted using the convert_imu_data.m script. 
clear all; close all; clc;
%% Set file paths. 
imuFileName = 'MT_012005D6_009-001_orientations.sto';
accFileName = 'MT_012005D6_009-001_linearAccelerations.sto';
modelName = 'imuTrackingModel.osim';
baseIMUName = 'pelvis_imu'; 
baseIMUdirection = 'z';

%% Instantiate a orientationTracker
ot = orientationTracker(modelName, imuFileName, accFileName);
%% Convert the quternions to orientations
ot.convertQuaternionToRotations()
%% Rotate Data to OpenSim World
ot.rotateOrientations2OpenSimFrame()
%% Set the base IMU and direction
ot.setBaseIMUDirection(baseIMUName, baseIMUdirection);
%% Compute the rotation of the base frame given the Base IMU and direction
R_HG = ot.computeHeadingCorrection();
%% Calibrate the model
ot.calibrateModelFromOrientations();
%% Run IK
ot.InverseKinematics(1)
% %% Plot IMU Frames 
% ot.plotFrames_raw('pelvis_imu');
% ot.plotFrames_opensimRotated('pelvis_imu');
% ot.plotFrames_correctedHeading('pelvis_imu');



