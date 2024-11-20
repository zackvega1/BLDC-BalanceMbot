#include "pid.h"

PIDController::PIDController(float P, float I, float D, float ramp, float limit)
    : P(P)
    , I(I)
    , D(D)
    , output_ramp(ramp)    // output derivative limit [volts/second]
    , limit(limit)         // output supply limit     [volts]
    , error_prev(0.0f)
    , output_prev(0.0f)
    , integral_prev(0.0f)
    , sintegral_prev(0.0f)
    , serror_prev(0.0f)
{
    timestamp_prev = _micros();
}

// PID controller function
float PIDController::operator() (float error, float serror = 0){
    // calculate the time from the last call
    unsigned long timestamp_now = _micros();
    float Ts = (timestamp_now - timestamp_prev) * 1e-6f;
    // quick fix for strange cases (micros overflow)
    if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    // u(s) = (P + I/s + Ds)e(s)
    // Discrete implementations
    // proportional part
    // u_p  = P *e(k)
    float sP = 1.5;
    float sI = 0.1;
    float sD = 0.0;

    float proportional = P * error;
    float sproportional = sP * serror;
    //Serial.println(serror);
    // Tustin transform of the integral part
    // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
    float integral = integral_prev + I*Ts*0.5f*(error + error_prev);
    float sintegral = sintegral_prev + sI*Ts*.05f*(serror + serror_prev);
    // antiwindup - limit the output
    integral = _constrain(integral, -limit, limit);
    sintegral = _constrain(sintegral, -limit, limit);
    // Discrete derivation
    // u_dk = D(ek - ek_1)/Ts
    float derivative = D*(error - error_prev)/Ts;
    float sderivative = sD*(serror - serror_prev)/Ts;
    // sum all the components
    float output = proportional + integral + derivative + sproportional + sintegral + sderivative;
    // antiwindup - limit the output variable
    output = _constrain(output, -limit, limit);

    // if output ramp defined
    if(output_ramp > 0){
        // limit the acceleration by ramping the output
        float output_rate = (output - output_prev)/Ts;
        if (output_rate > output_ramp)
            output = output_prev + output_ramp*Ts;
        else if (output_rate < -output_ramp)
            output = output_prev - output_ramp*Ts;
    }
    // saving for the next pass
    integral_prev = integral;
    sintegral_prev = sintegral;
    output_prev = output;
    error_prev = error;
    serror_prev = serror;
    timestamp_prev = timestamp_now;
    return output;
}

void PIDController::reset(){
    integral_prev = 0.0f;
    output_prev = 0.0f;
    error_prev = 0.0f;
}
