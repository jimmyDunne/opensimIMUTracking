%% transform_imu_data_to_sto.m
% Transform XSENS IMU sensor data files to OpenSim .sto format. 
% Written by: James Dunne

%% Clear variables and load opensim libraries. 
clear all; close all; clc; 
import org.opensim.modeling.*

%% Build an Xsens Settings Object. 
% Instantiate the Reader Settings Class
xsensSettings = XsensDataReaderSettings('myIMUMappings.xml');
% Instantiate an XsensDataReader
xsens = XsensDataReader(xsensSettings);
% Get a table reference for the data
tables = xsens.read('IMUData/');
% get the trial name from the settings
trial = char(xsensSettings.get_trial_prefix());

%% Write IMU Quaternions to file
quatTable = xsens.getOrientationsTable(tables);
STOFileAdapterQuaternion.write(quatTable,  [trial '_orientations.sto']);

%% Write IMU Acceleration to file
accelTable = xsens.getLinearAccelerationsTable(tables);
STOFileAdapterVec3.write(accelTable, [trial '_linearAccelerations.sto']);

%% Write IMU Magnetic (North) Heading to file
magTable = xsens.getMagneticHeadingTable(tables);
STOFileAdapterVec3.write(magTable, [trial '_magneticNorthHeadings.sto']);

%% Write IMU Angular Velocity to file
angVelTable = xsens.getAngularVelocityTable(tables);
STOFileAdapterVec3.write(angVelTable, [trial '_angularVelocities.sto']);
