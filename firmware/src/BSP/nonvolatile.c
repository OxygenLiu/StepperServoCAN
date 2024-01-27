 /**
 * StepperServoCAN
 *
 * Copyright (c) 2020 Makerbase.
 * Copyright (C) 2018 MisfitTech LLC.
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

#include "nonvolatile.h"
#include "board.h"
#include "stepper_controller.h"
#include "encoder.h"


volatile MotorParams_t liveMotorParams;
volatile SystemParams_t liveSystemParams;
volatile PID_t pPID; //positional current based PID control parameters
volatile PID_t vPID; //velocity PID control parameters

nvm_t nvmMirror;
static uint32_t NVM_address = PARAMETERS_FLASH_ADDR;

// Find nvm data actual address
void nonvolatile_begin(void)
{
	uint32_t i = ((FLASH_PAGE_SIZE / NONVOLATILE_STEPS) - 1); //(1024/62) = 16(0~15)
	
	NVM_address = PARAMETERS_FLASH_ADDR;

	//NVM_address search for the beginning of the last parameters segment (some sort of wear leveling)	
	for(i=((FLASH_PAGE_SIZE / NONVOLATILE_STEPS) - 1); i > 0; i--)
	{
		if( Flash_readHalfWord( (PARAMETERS_FLASH_ADDR + (i * NONVOLATILE_STEPS)) ) != invalid )
		{
			NVM_address = (PARAMETERS_FLASH_ADDR + (i * NONVOLATILE_STEPS));
			return;
		}
	}
}

void nvmWriteCalTable(void *ptrData)
{
	bool state = motion_task_isr_enabled;
	Motion_task_disable(); 
	
	Flash_ProgramPage(CALIBRATION_FLASH_ADDR, ptrData, (sizeof(FlashCalData_t)/2U));
	
	if (state) {
		Motion_task_enable();
	}
}

//NVM mirror - buffers NVM read/write
void nvmMirrorInRam(void){
	nvmMirror = *(nvm_t*)NVM_address;//copy nvm from flash to ram
}

//currently only used once - after first boot
void nvmWriteConfParms(nvm_t* ptrNVM)
{		
	bool state = motion_task_isr_enabled;
	Motion_task_disable();
	
	ptrNVM->motorParams.parametersValid  = valid;
	ptrNVM->systemParams.parametersValid = valid;
	
	//wear leveling
	if(Flash_readHalfWord(NVM_address) != invalid && ((NVM_address + NONVOLATILE_STEPS) < (PARAMETERS_FLASH_ADDR + FLASH_PAGE_SIZE)))
	{
		NVM_address += NONVOLATILE_STEPS;
		
		while( Flash_checknvmFlash(NVM_address, sizeof(nvm_t)/2U) == false )
		{
			if( (NVM_address + NONVOLATILE_STEPS) < (PARAMETERS_FLASH_ADDR + FLASH_PAGE_SIZE))
			{
				NVM_address += NONVOLATILE_STEPS;
			}
			else
			{
				NVM_address = PARAMETERS_FLASH_ADDR;
				Flash_ProgramPage(NVM_address, (uint16_t*)ptrNVM, (sizeof(nvm_t)/2U));
				return;
			}
		}
		Flash_ProgramSize(NVM_address, (uint16_t*)ptrNVM, (sizeof(nvm_t)/2U));
	}
	else 
	{
		NVM_address = PARAMETERS_FLASH_ADDR;
		Flash_ProgramPage(NVM_address, (uint16_t*)ptrNVM, (sizeof(nvm_t)/2U));
	}

	nvmMirrorInRam();
	
	if (state) {
		Motion_task_enable();	
	}
	return;
}

//parameters first boot defaults and restore on corruption
void validateAndInitNVMParams(void)
{
	nvmMirrorInRam();

	if (nvmMirror.systemParams.parametersValid != valid){ //systemParams invalid
		nvmMirror.pPID.Kp = .005f;  nvmMirror.pPID.Ki = .0002f;  nvmMirror.pPID.Kd = 0.0f;
		nvmMirror.vPID.Kp = 2.0f;   nvmMirror.vPID.Ki = 1.0f; 	 nvmMirror.vPID.Kd = 1.0f;

		nvmMirror.systemParams.microsteps = 256U; //unused
		nvmMirror.systemParams.controllerMode = CTRL_TORQUE;  //unused
		nvmMirror.systemParams.dirRotation = CCW_ROTATION;
		nvmMirror.systemParams.errorLimit = 0U;  //unused
		nvmMirror.systemParams.errorPinMode = ERROR_PIN_MODE_ACTIVE_LOW_ENABLE;  //default to !enable pin
	}

	if(nvmMirror.motorParams.parametersValid != valid){
		nvmMirror.motorParams.currentMa = 800U;
		nvmMirror.motorParams.currentHoldMa = 400U; //unused
		nvmMirror.motorParams.motorWiring = false;
		nvmMirror.motorParams.fullStepsPerRotation = invalid; //it will be detected along with swapPhase
	}

	if((nvmMirror.systemParams.parametersValid != valid) || (nvmMirror.motorParams.parametersValid != valid)){
		nvmWriteConfParms(&nvmMirror); //write default parameters
	}

	//the motor parameters are later checked in the stepper_controller code
	// as that there we can auto set much of them.

}
