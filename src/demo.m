% set up Matlab workspace
close all; clear; clc;
addpath(genpath('../.'))

% load regressor chain, scenario and params
load('data/RC_NN_12c_kbg.mat', 'RC')
[setup, state] = loadSetup();
params = loadParams();

% run demo
h_func = demo_header;
h_func.continuousQueries(RC, setup, state, params)