/*
 * Pericom 30216C  driver IC for type C
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _PERICOM_30216C_H_
#define _PERICOM_30216C_H_

/*  pericom id
  *  0: Device mode
  *  1: Host mode
  *  2: TrySNK DRP mode
  *
  */
enum pericom_role_mode{
	DEVICE_MODE,
	HOST_MODE,
	//DRP_MODE,
	TRYSNK_DRP_MODE,
	//TRYSRC_DRP_MODE
};

enum pericom_power_mode{
	ACTIVE_MODE,
	POWERSAVING_MODE
};

#define PERICOM_I2C_RETRY_TIMES 5

#define PERICOM_POWER_SAVING_MASK 0x80
#define PERICOM_POWER_SAVING_OFFSET 7
#define PERICOM_ROLE_MODE_MASK  0x06
#define PERICOM_INTERRUPT_MASK 0x01
#define PERICOM_CC_ORI_MASK  0x03
#define PERICOM_INTERRUPT_UNMASK 0
#define PERICOM_ROLE_OFFSET 1
#define PERICOM_DRP2_TRY_SNK_OFFSET 6
#define PERICOM_DRP2_TRY_SNK 0x40

//Register 2 Control register includes: PowerSaving, Dualrole2 TrySNK or TrySRC setting, Charging current mode setting, Accessory Detection in device mode setting, Role of port setting, Interrupt Mask settin
#define PERICOM_POWER_SAVING_MODE 0x80	//PowerSaving mode, PI5USB30216C standby current is below 5uA
#define PERICOM_SRC_MODE 0x02 			//Charge and desktop computer are always DFP
#define PERICOM_TRY_SNK_DRP_MODE 0x46   //Phone is usally UFP, Always initially UFP and charging, unless connected to an accessory or wearable
#define PERICOM_TRY_SRC_DRP_MODE 0x06	//Laptop and battery bank is usally DFP, Always initially DFP to charging tablets and phones, can powered by charger
#define PERICOM_DRP_MODE 0x04			//Tablet is neutral, DFP to phones and UFP to laptops
#define PERICOM_SNK_MODE 0x00			//accessory or wearable always UFP
#define PERICOM_SNK_WITH_ACCESSORY_DETECT_MODE 0x20
#define PERICOM_INT_MASK 0x01			//Mask interrupt
#define PERICOM_INT_UNMASK 0
#define PERICOM_CHARGING_1A5_CURRENT_MODE 0x08  	//Medium current mode (1.5A)
#define PERICOM_CHARGING_3A_CURRENT_MODE 0x10		//High current mode (3A)
#define PERICOM_CHARGING_DEFAULT_CURRENT_MODE 0	//Default current mode

//Register 3 Interrupt status include: ATTACH EVENT, DETACH EVENT
#define PERICOM_ATTACH_EVENT 1
#define PERICOM_DETACH_EVENT 2

//Register 4 Status includes: Attached_Port_Status, CC_Plug polarity, Charging current detection, VBUS status
#define PERICOM_ATTACHED_PORT_STATUS 0x07
#define PERICOM_ATTACHED_PORT_STATUS_OFFSET 2
#define PERICOM_TYPEC_DEVICE_ATTACHED 1
#define PERICOM_TYPEC_HOST_ATTACHED 2
#define PERICOM_TYPEC_AUDIO_ADAPTER_ACCESSORY_ATTACHED 3
#define PERICOM_TYPEC_DEBUG_ACCESSORY_ATTACHED 4
#define PERICOM_TYPEC_DEVICE_WITH_ACTIVE_CABLE_ATTACHED 5
#define PERICOM_CC_ORI 0x03
#define PERICOM_TYPEC_CC1_CONNECTION 1
#define PERICOM_TYPEC_CC2_CONNECTION 2
#define PERICOM_ATTACHED_VBUS_STATUS_OFFSET 7
#define PERICOM_VBUS_DETECT 0x80
#define PERICOM_CURRENT_MODE_STATUS 0x03
#define PERICOM_CURRENT_MODE_OFFSET 5
#define PERICOM_DEFAULT_CURRENT_MODE 1
#define PERICOM_1A5_CURRENT_MODE 2
#define PERICOM_3A_CURRENT_MODE 3

#endif
