/***********************************************************************************
Copyright (c) Nordic Semiconductor ASA
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of other
  contributors to this software may be used to endorse or promote products
  derived from this software without specific prior written permission.

  4. This software must only be used in a processor manufactured by Nordic
  Semiconductor ASA, or in a processor manufactured by a third party that
  is used in combination with a processor manufactured by Nordic Semiconductor.


THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************************************************/
#ifndef _SERIAL_COMMAND_H__
#define _SERIAL_COMMAND_H__

typedef __packed enum
{
    SERIAL_CMD_OPCODE_ECHO                  = 0x02,
    SERIAL_CMD_OPCODE_RADIO_RESET           = 0x0E,
    
    SERIAL_CMD_OPCODE_INIT                  = 0x70,
    SERIAL_CMD_OPCODE_VALUE_SET             = 0x71,
    SERIAL_CMD_OPCODE_VALUE_ENABLE          = 0x72,
    SERIAL_CMD_OPCODE_VALUE_DISABLE         = 0x73,
    
    SERIAL_CMD_OPCODE_VALUE_GET             = 0x7A,
    SERIAL_CMD_OPCODE_BUILD_VERSION_GET     = 0x7B,
    SERIAL_CMD_OPCODE_ACCESS_ADDR_GET       = 0x7C,
    SERIAL_CMD_OPCODE_CHANNEL_GET           = 0x7D,
    SERIAL_CMD_OPCODE_HANDLE_COUNT_GET      = 0x7E,
    SERIAL_CMD_OPCODE_ADV_INT_GET           = 0x7F,
} serial_cmd_opcode_t;

#endif