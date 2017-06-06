/*
 *    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
 *    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  A131 RS485 communication protocol
 *
 *  originally found on cnczone.nl, posted by Rikkepic:
 *  http://cnczone.nl/viewtopic.php?f=35&t=11605
 *
 *  Parameters
 *
 *  PD001   2   RS485 Control of run commands
 *  PD002   2   RS485 Control of operating frequency
 *  PD023   1   Reverse run enabled
 *  PD163   1   RS485 Address: 1
 *
 *
 *
 *  == Function Read ==
 *
 *  ADDR    CMD     LEN     PAR     DATA        CRC
 *  0x01    0x01    0x03    0xA5    0x00 0x00   0x2C 0x6D       Read PD165 (165=0xA5)
 *
 *  == Function Write ==
 *
 *  ADDR    CMD     LEN     PAR     DATA        CRC
 *  0x01    0x02    0x03    0x03    0x09 0xC4   0x8F 0x8D       Write PD003 (0x9C4 = 2500 = 25.00Hz)
 *
 *  == Control Write ==
 *
 *  ADDR    CMD     LEN     DATA    CRC
 *  0x01    0x03    0x01    0x01    0x31 0x88                   Start spindle clockwise
 *
 *  ADDR    CMD     LEN     DATA    CRC
 *  0x01    0x03    0x01    0x08    0xF1 0x8E                   Stop spindle
 *
 *  ADDR    CMD     LEN     DATA    CRC
 *  0x01    0x03    0x01    0x11    0x30 0x44                   Start spindle counter-clockwise
 *
 *  == Control Read ==
 *
 *  ADDR    CMD     LEN     PAR     DATA        CRC
 *  0x01    0x04    0x03    0x00    0x00 0x00   0xF0 0x4E       Read Frequency
 *
 *  ADDR    CMD     LEN     PAR     DATA        CRC
 *  0x01    0x04    0x03    0x02    0x00 0x00   0x51 0x8E       Read Output Current
 *
 *  ADDR    CMD     LEN     PAR     DATA        CRC
 *  0x01    0x04    0x03    0x03    0x00 0x00   0x00 0x4E       Read Rotation
 *
 *  ADDR    CMD     LEN     PAR     DATA        CRC
 *  0x01    0x04    0x03    0x04    0x00 0x00   0xB1 0x8F       Read DC Volatge
 *
 *  ADDR    CMD     LEN     PAR     DATA        CRC
 *  0x01    0x04    0x03    0x05    0x00 0x00   0xE0 0x4F       Read AC Voltage
 *
 *  ADDR    CMD     LEN     PAR     DATA        CRC
 *  0x01    0x04    0x03    0x07    0x00 0x00   0x41 0x8F       Read Temperature
 *
 *  == Control Read ==
 *
 *  ADDR    CMD     LEN     DATA        CRC
 *  0x01    0x05    0x02    0x09 0xC4   0xBF 0x0F               Write Frequency (0x9C4 = 2500 = 25.00HZ)
 *
 *
 *
 ################################################### THE A131 DATA Structure #####################################
 #
 # 9.2 Protocol
 # speed：9600bps。
 # Data bits：8 bits, 1 begin bit, 1 end bit, without parity (8N1 RTU)
 # Data length sent by inverter：13 bytes
 # Data length received by inverter：9 bytes
 #
 #
 #################### Inverter sends data actively #########################################
 #
 # 1       2       3       4       5       6       7       8       9       10      11      12      13
 # 00      55      D1      D2      D3      D4      D5      D6      01      00      XOR     ADD     FF
 # Data of bytes 1,2,9,10,13 is fixed.
 # D1—D5 data displayed on digital tube，D1 highest bit.
 # Digital tube
 # D1   D2    D3    D4    D5
 # X    X     X     X     X    Every X means a number between 0 and 9.
 #
 # D6 indicator data
 # D6.0 Fault D6.1 Clockwise
 # D6.2 Anticlockwise D6.3 Analog input
 # D6.4 Multi-segment D6.5 Panel
 # D6.6 External signal D6.7 Reserved
 # XOR: xor data of byte 2~10
 # ADD: sum of byte 2~11
 #
 #
 # 9.4 Data definition received by inverter
 #
 # 1     2      3     4      5     6      7       8       9
 # 00    55     D1    D2     D3    01     XOR(54) ADD(A9)     FF
 # Data of byte 1,2,6,9 is fixed.
 #
 # Its means :
 # 1=>ADDR 0x00
 # 2=>CMD 0x55 (always the same)
 # D1:D2 frequency (12 ~1013)
 ###############################D3 Button/control data ###########
 #        D3.0 RUN D3.1 STOP
 #        D3.2 up arrow D3.3 left arrow
 #        D3.4 right arrow D3.5 down arrow
 #        D3.6 SET D3.7 change direction
 # XOR: xor data of byte 2~6 (dunno if its 2 to 6, or just 2 and 6) As its always fixed 0x55 XOR 0x01 = 0x54
 # ADD: sum of byte 2~7 (same as the XOR) As its fixed.... 0x55 ADD 0x54 = 0xA9
 */

#include "libs/Kernel.h"
#include "StreamOutputPool.h"
#include "BufferedSoftSerial.h"
#include "ModbusSpindleControl.h"
#include "A131SpindleControl.h"
#include "gpio.h"
#include "Modbus.h"

void A131SpindleControl::turn_on()
{
   // prepare data for the spindle on command
   char turn_on_msg[9] =
   {
      0x00, 0x55, 0x00, 0x00, 0x00, 0x01, 0x54, 0xA9, 0xFF
   };

   turn_on_msg[7] = turn_on_msg[2] ^ turn_on_msg[3] ^ turn_on_msg[4] ^ turn_on_msg[5] ^ turn_on_msg[6];
   turn_on_msg[8] = turn_on_msg[2] + turn_on_msg[3] + turn_on_msg[4] + turn_on_msg[5] + turn_on_msg[6] + turn_on_msg[7];

   // calculate CRC16 checksum
   //unsigned int crc = modbus->crc16(turn_on_msg, sizeof(turn_on_msg) - 2);

   //turn_on_msg[4] = crc & 0xFF;
   //turn_on_msg[5] = (crc >> 8);

   // enable transmitter
   modbus->dir_output->set();
   modbus->delay(1);
   // send the actual message
   modbus->serial->write(turn_on_msg, sizeof(turn_on_msg));
   // wait a calculated time for the data to be sent
   modbus->delay((int)ceil(sizeof(turn_on_msg) * modbus->delay_time));
   // disable transmitter
   modbus->dir_output->clear();
   // wait 50ms, required by the Modbus standard
   modbus->delay(50);
   spindle_on = true;
}

void A131SpindleControl::turn_off()
{
   // prepare data for the spindle off command
   char turn_off_msg[9] =
   {
      0x00, 0x55, 0x00, 0x00, 0x01, 0x01, 0x54, 0xA9, 0xFF
   };

   // calculate CRC16 checksum
   //unsigned int crc = modbus->crc16(turn_off_msg, sizeof(turn_off_msg) - 2);

   // turn_off_msg[4] = crc & 0xFF;
   //turn_off_msg[5] = (crc >> 8);

   // enable transmitter
   modbus->dir_output->set();
   modbus->delay(1);
   // send the actual message
   modbus->serial->write(turn_off_msg, sizeof(turn_off_msg));
   // wait a calculated time for the data to be sent
   modbus->delay((int)ceil(sizeof(turn_off_msg) * modbus->delay_time));
   // disable transmitter
   modbus->dir_output->clear();
   // wait 50ms, required by the Modbus standard
   modbus->delay(50);
   spindle_on = false;
}

void A131SpindleControl::set_speed(int target_rpm)
{
   // prepare data for the set speed command
   char set_speed_msg[9] =
   {
      0x00, 0x55, 0x00, 0x00, 0x06, 0x01, 0x54, 0xA9, 0xFF
   };

   // convert RPM into Hz
   unsigned int hz = target_rpm / 60 * 100;

   set_speed_msg[3] = (hz >> 8);
   set_speed_msg[4] = hz & 0xFF;
   // calculate CRC16 checksum
   //unsigned int crc = modbus->crc16(set_speed_msg, sizeof(set_speed_msg) - 2);
   //set_speed_msg[5] = crc & 0xFF;
   //set_speed_msg[6] = (crc >> 8);

   // enable transmitter
   modbus->dir_output->set();
   modbus->delay(1);
   // send the actual message
   modbus->serial->write(set_speed_msg, sizeof(set_speed_msg));
   // wait a calculated time for the data to be sent
   modbus->delay((int)ceil(sizeof(set_speed_msg) * modbus->delay_time));
   // disable transmitter
   modbus->dir_output->clear();
   // wait 50ms, required by the Modbus standard
   modbus->delay(50);
}

/*
 #################### Inverter sends data actively #########################################
 #
 # 1       2       3       4       5       6       7       8       9       10      11      12      13
 # 00      55      D1      D2      D3      D4      D5      D6      01      00      XOR     ADD     FF
 # Data of bytes 1,2,9,10,13 is fixed.
 # D1—D5 data displayed on digital tube，D1 highest bit.
 # Digital tube
 # D1   D2    D3    D4    D5
 # X    X     X     X     X    Every X means a number between 0 and 9.
 #
 # D6 indicator data
 # D6.0 Fault D6.1 Clockwise
 # D6.2 Anticlockwise D6.3 Analog input
 # D6.4 Multi-segment D6.5 Panel
 # D6.6 External signal D6.7 Reserved
 # XOR: xor data of byte 2~10
 # ADD: sum of byte 2~11
 #
 */
void A131SpindleControl::report_speed()
{
   // clear RX buffer before start
   while(modbus->serial->readable()){
         modbus->serial->getc();
         }

   //  prepare data for the get speed command
   //  char get_speed_msg[8] =
   //  {
   // 0x01, 0x04, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00
   //  };
   //  calculate CRC16 checksum
   //  unsigned int crc = modbus->crc16(get_speed_msg, sizeof(get_speed_msg) - 2);
   //  get_speed_msg[6] = crc & 0xFF;
   //  get_speed_msg[7] = (crc >> 8);

   // enable transmitter
   //modbus->dir_output->set();
   // modbus->delay(1);
   // send the actual message
   // modbus->serial->write(get_speed_msg, sizeof(get_speed_msg));
   // wait a calculated time for the data to be sent
   // modbus->delay((int)ceil(sizeof(get_speed_msg) * modbus->delay_time));
   // disable transmitter
   //modbus->dir_output->clear();
   // wait 50ms, required by the Modbus standard
   // modbus->delay(50);

   // wait for the complete message to be received
   //modbus->delay((int)ceil(8 * modbus->delay_time));
   // prepare an array for the answer
   char speed[13] =
   {
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
   };

   // read the answer into the buffer
   for(int i = 0; i < 13; i++){
       speed[i] = modbus->serial->getc();
       }

/*... 3    4     5     6     7
 #    D1   D2    D3    D4    D5
 #    X    X     X     X     X    Every X means a number between 0 and 9.
 */
// get the Hz value from trhe answer and convert it into an RPM value



   unsigned int hz  = (int)speed[3] * 100 + (int)speed[4] * 10 + (int)speed[5] * 1 + (int)speed[6] * 0.1 + (int)speed[7] * 0.01;
   unsigned int rpm = hz / 100 * 60;

   // report the current RPM value
   THEKERNEL->streams->printf("Current RPM: %d\n", rpm);
}
