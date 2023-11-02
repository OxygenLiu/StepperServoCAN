 /**
 * StepperServoCAN
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <www.gnu.org/licenses/>.
 *
 */

/**
 * @ Description:
 * Calculation of actuator coefficients.
 */

#include "actuator_config.h"
#include "stepper_controller.h"

float volatile gearing_ratio;
float volatile actuatorTq_to_current;
float volatile current_to_actuatorTq;

//todo move these parameters to NVRAM
//specify motor parameters here:
const uint16_t rated_current = 1300; //mA, 1.8deg stepper, 40mm
const uint16_t rated_torque = 45;   //cNm, 1.8deg stepper, 40mm 

//specify gearing parameters here:
const float motor_gearbox_ratio = 23; //cycloidal gearbox ratio 23 
const float final_drive_ratio = 1; //assembly gearing ratio

const int8_t anticogging_factor = 30; //default 30, minimizes cogging under load (0-127)

void update_actuator_parameters(void){
    gearing_ratio = motor_gearbox_ratio * final_drive_ratio;

    actuatorTq_to_current = (float) rated_current / rated_torque * 100 / gearing_ratio;
    current_to_actuatorTq = 1 / actuatorTq_to_current;

    closeLoopMaxDes = 3000U; //2000U; //sets maximum close loop current [mA] to limit stresses and heat generation

}