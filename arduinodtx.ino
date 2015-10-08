/* arduinodtx.ino - Open source digital RC transmitter software for the Arduino 

Copyright (C) 2014 Gregor Schlechtriem.  All rights reserved.
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, version 3.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
Contact information: http://www.pikoder.com

This software is based on Richard Goutorbe's arduinotx (Open source RC transmitter for the Arduino) and makes use of most of the arduinotx modules. 
When changes were done I also changed the header information.
*/

/*
** Resources -----------------------------------------------
*/

#include <TimerOne.h>
#include <SoftwareSerial.h>

#include <EEPROM.h>
#include "arduinotx_lib.h"
#include "arduinotx_eeprom.h"
#include "arduinotx_led.h"
#include "arduinotx_command.h"
#include "arduinodtx_transmitter.h"
#ifdef BUZZER_ENABLED
#include "arduinotx_buzz.h"
#endif

/*
** Global declarations -----------------------------------------------
*/

// Tx manager
ArduinoTx ArduinoTx_obj;

// Command mode interpreter
ArduinotxCmd Command_obj;

// EEPROM manager
ArduinotxEeprom Eeprom_obj;

// Led manager
ArduinotxLed Led_obj(LED_PIN);

#ifdef BUZZER_ENABLED
// Buzzer manager
ArduinotxBuzz Buzzer_obj(BUZZER_PIN);
#endif

// Serial Interface to SSC
SoftwareSerial mySerial = SoftwareSerial(rx_PIN, tx_PIN);

// These 2 global variables are used to request the PPM signal values from ISR(TIMER1_COMPA_vect)
volatile byte RequestPpmCopy_bool = false;
volatile unsigned int PpmCopy_int[CHANNELS]; // pulse widths (microseconds)

/*
** Arduino specific functions -----------------------------------------------------------------
*/

void setup() {
	ArduinoTx_obj.Init();

        // initialize serial communication to SSC
        mySerial.begin(9600);       

	// configure Timer1 for update cycle
        Timer1.initialize(cUpdateCycle);
        Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt	
}

/* serialEvent() occurs whenever a new data comes in the hardware serial RX. 
** This routine is run between each time loop() runs, so using delay inside loop()
** can delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
	if (Serial.available())
		Command_obj.Input();
}

//~ volatile unsigned int DebugValue_int = 0; // debug

// Scan all channels for actual values and transmit changed values using the miniSSCII-protocol
// Warning: calling Serial.print() within this method will probably hang the program

void callback() {
	static byte Chan_idx_byt = CHANNELS;
	static unsigned int Chan_pulse_int[CHANNELS]; // pulse widths (microseconds)
	static unsigned int Sum_int = 0;
	// Read input controls and update Chan_pulse_int[]
	for (byte chan_byt = 0; chan_byt < CHANNELS; chan_byt++) {
		unsigned int control_value_int = 0;
		control_value_int = ArduinoTx_obj.ComputeChannelPulse(chan_byt, ArduinoTx_obj.ReadControl(chan_byt))/4;
                if (control_value_int != Chan_pulse_int[chan_byt]) {
                     mySerial.write(byte(0xFF));  // synch token
                     mySerial.write(chan_byt);    // channel number
                     mySerial.write(byte(control_value_int));
		     Chan_pulse_int[chan_byt] = control_value_int;
                }
	}
	if (RequestPpmCopy_bool) {
		// copy the PPM sequence values into global array for the "print ppm" command
		for (byte chan_byt = 0; chan_byt < CHANNELS; chan_byt++)
			PpmCopy_int[chan_byt] = Chan_pulse_int[chan_byt] ;
		RequestPpmCopy_bool = false;
	}
}

// The main loop is interrupted every PPM_PERIOD ms by ISR(TIMER1_COMPA_vect)
// we perform non time-critical operations in here
void loop() {
	// TXREFRESH_PERIOD defines the frequency at which Special Switches are read and corresponding transmitter state is updated
	const unsigned int TXREFRESH_PERIOD = 50L; // 50ms =20Hz, should be >= 20 ms
	static unsigned long Last_TxRefresh_uln = 0L;
	unsigned long now_uln = millis(); // timer overflows after 50 days
	if (now_uln >= Last_TxRefresh_uln + TXREFRESH_PERIOD || now_uln < Last_TxRefresh_uln) {
		ArduinoTx_obj.Refresh();
		Last_TxRefresh_uln = now_uln;
	}
	
	Led_obj.Refresh();
#ifdef BUZZER_ENABLED	
	Buzzer_obj.Refresh();
#endif

	//~ // print debug info every 0.5 second
	//~ static unsigned long Debug_time_lng = 0L;
	//~ if ( millis() > Debug_time_lng + 500L) {
		//~ Debug_time_lng = millis();
		
		//~ Serial.println(DebugValue_int);
	//~ }
}
