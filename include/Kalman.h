#pragma once

float KG; 
float eEst;
float eEst_prev = 1;
float Q = 0.01;
float eMea = 1;
float x;
float x_prev = 0;

float kalman_filter(float mea){
    eEst = eEst + Q; 
    KG = eEst / (eEst + eMea);
    x = x_prev + (KG * (mea - x_prev));
    eEst = (1 - KG) * eEst;

    x_prev = x;
    eEst_prev = eEst;
    return x;
}