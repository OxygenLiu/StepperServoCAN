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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


/**
 * @ Description:
 * Quick calculation of actuator coefficients.
 */

#ifndef ACTUATOR_CONFIG_H
#define ACTUATOR_CONFIG_H

#include <stdint.h>

extern const uint16_t rated_current; //mA
extern const uint16_t rated_torque; //cNm

//specify gearing parameters here:
extern const float motor_gearbox_ratio; //gearbox ratio
extern const float final_drive_ratio; //assembly gearing ratio

//calculate actuator parameters to be used by control_api 
extern volatile float gearing_ratio;
extern volatile float actuatorTq_to_current;
extern volatile float current_to_actuatorTq;
void update_actuator_parameters(void);


#endif // ACTUATOR_CONFIG_H
