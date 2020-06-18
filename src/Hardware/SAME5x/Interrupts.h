/*
 * PinInterrupts.h
 *
 *  Created on: 6 Jul 2019
 *      Author: David

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 3 as published by the Free Software Foundation.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef SRC_HARDWARE_PININTERRUPTS_H_
#define SRC_HARDWARE_PININTERRUPTS_H_

#include "RepRapFirmware.h"

// Pin change interrupt support

void InitialisePinChangeInterrupts();
bool attachInterrupt(Pin pin, StandardCallbackFunction callback, InterruptMode mode, CallbackParameter param);
void detachInterrupt(Pin pin);

#endif /* SRC_HARDWARE_PININTERRUPTS_H_ */
