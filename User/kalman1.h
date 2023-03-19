#ifndef	KALMAN1_H
#define	KALMAN1_H

//#include "kalman1.h"

#include "stm32f1xx.h"
#include "string.h"
#include "stdio.h"
#include <math.h>

float _err_measure;
float _err_estimate;
float _q;
float _current_estimate;
float _last_estimate;
float _kalman_gain;

void SimpleKalmanFilter(float mea_e, float est_e, float q);

float updateEstimate(float mea);
void setMeasurementError(float mea_e);
void setEstimateError(float est_e);
void setProcessNoise(float q);
float getKalmanGain();
float getEstimateError();

//void delay_cycles(int t);
//void delay_us(int t);

void SimpleKalmanFilter(float mea_e, float est_e, float q)
{
  _err_measure=mea_e;
  _err_estimate=est_e;
  _q = q;   
}

float updateEstimate(float mea)
{
  _kalman_gain = _err_estimate/(_err_estimate + _err_measure);
  _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
  _err_estimate =  (1.0 - _kalman_gain)*_err_estimate + fabs(_last_estimate-_current_estimate)*_q;
  _last_estimate=_current_estimate;

  return _current_estimate;
}

void setMeasurementError(float mea_e)
{
  _err_measure=mea_e;
}

void setEstimateError(float est_e)
{
  _err_estimate=est_e;
}

void setProcessNoise(float q)
{
  _q=q;
}

float getKalmanGain() {
  return _kalman_gain;
}

float getEstimateError() {
  return _err_estimate;
}


#endif

