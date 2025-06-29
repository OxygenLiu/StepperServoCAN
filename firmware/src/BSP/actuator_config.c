/**
 * @ Description:
 * Actuator configuration set by the user.
 */

#include "actuator_config.h"
#include "stepper_controller.h"
#include "utils.h"

// ----- should be set by the user --------------------------------------------------------------------------------
const bool USE_VOLTAGE_CONTROL = false; // voltage or current control - voltage control recommended for hardware v0.3

// select simple or advanced parameters
// simple parameters (rated torque and current) are usually overstated by manufacturers
// use simple parameters if you don't want to measure motor_k_bemf
// or to get more accurate torque-to-current relationship use measured motor_k_bemf instead
// yes, really! motor current-to-torque relationship can be obtained from K_bemf factor
const bool USE_SIMPLE_PARAMETERS = true;

// SIMPLE PARAMETERS:
const float motor_rated_current = (float) 1.3; // A
const float motor_rated_hold_torque =  (float) 39;  // Ncm - (values in datasheets represents both phases energized with rated current AND are often inflated for small motors)

// MEASURED PARAMETERS: // todo autocalibrate and store in NVRAM
// default are for black 17HS4401S https://www.aliexpress.com/item/4001349087963.html
volatile int16_t motor_k_bemf = 750; // mV/(rev/s) -   hold F2 to measure, use MCUViewer to see new motor_k_bemf value and update manually here
// note, when motor_k_bemf is too low, the motor can have higher top speed when unloaded (unintentional field weakening via I_d), but power and torque will not be accurate
volatile int16_t phase_R = 2400;         // mOhm -      it's best to measure this with multimeter
volatile int16_t phase_L = 3230;         // uH -        use datasheet value or RLC meter to measure - correct value maximizes peak motor power

// specify gearing parameters here:
const float motor_gearbox_ratio = 5.0F+(2.0F/11.0F); // gearbox ratio - enter planetary gearbox tooth calculation for best accuracy
const float final_drive_ratio = 2.0F;                // assembly gearing ratio

const int8_t anticogging_factor = 30; //minimizes cogging under load - (0-127) -value to be chosen experimentally 

// ------  end user settings --------------------------------------------------------------------------------------




float volatile gearing_ratio;
float volatile actuatorTq_to_current; // mA/Nm - (ignores gearbox efficiency)
float volatile current_to_actuatorTq; // Nm/mA - (ignores gearbox efficiency)
volatile float motor_k_torque; // Nm/A

// interprets motor parameters
void update_actuator_parameters(bool use_simple_params){
    gearing_ratio = motor_gearbox_ratio * final_drive_ratio;

    if (use_simple_params) {
        motor_k_torque = (motor_rated_hold_torque / (SQRT2 * 100)) / motor_rated_current;
        motor_k_bemf = motor_k_torque * (V_to_mV * 2 * PI);
    }else{
        // motor torque and BEMF constant are directly correlated
        // this is the easiest way to get precise torque / current relationship without a load cell
        // as long as magnetic saturation doesn't not occur (usually below rated current), the relationship is simply:
        // k_torque[Nm/A] = k_bemf[V/rad/s]
        motor_k_torque = (float)motor_k_bemf / (V_to_mV * 2 * PI);
    }
    current_to_actuatorTq = motor_k_torque / V_to_mV * gearing_ratio;
    actuatorTq_to_current = 1 / current_to_actuatorTq;


    closeLoopMaxDes = 2000U; // position control maximum close loop current [mA] to limit stresses and heat generation

}