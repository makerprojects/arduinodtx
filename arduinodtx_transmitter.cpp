/* arduinotx_transmitter.ino - Tx manager
** 01-05-2013 class ArduinoTx
** 04-05-2013 read_potentiometer()
** 16-05-2013 ReadControl()
** 22-06-2013 ComputeChannelPulse() fixed dual rates
** 25-06-2013 CurrentDataset_byt, TxAlarm_int, get_selected_dataset()
** 27-06-2013 get_selected_dataset()
** 13-08-2013 Init() configures PPM_PIN and LED_PIN as outputs and all other digital pins as inputs with pull-up
** 17-08-2013 Init() fixed, check_throttle() improved
** 18-08-2013 check_battery()
** 23-08-2013 Init()
** 24-08-2013 ComputeChannelPulse() fixed underflow in subtrim computation, changed end points computation
** 26-08-2013 changed Morse codes flashed on the Led
** 31-08-2013 check_battery() ignore invalid samples
**
Copyright (C) 2013 Richard Goutorbe.  All right reserved.
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, version 3.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
Contact information: http://www.reseau.org/arduinorc/index.php?n=Main.Contact
*/

#include "TimerOne.h"

#include "arduinodtx_transmitter.h"
#include "arduinotx_led.h"
#include "arduinotx_command.h"
#ifdef BUZZER_ENABLED
#include "arduinotx_buzz.h"
#endif
#include "arduinotx_lib.h"

// Special Switches -----------------------------------------------------------
// they are used to configure the transmitter, not to control channels
// A switch is ON when opened because there is a pullup resistor on the corresponding input

const byte ArduinoTx::MODE_SWITCH_PIN = 9;		// 9 opened=transmission, closed=command mode
const byte ArduinoTx::THROTTLECUT_SWITCH_PIN = 11;	// opened=throttle cut, closed=throttle enabled
const byte ArduinoTx::DUALRATE_SWITCH_PIN = 12;	// opened=dual rate ON, closed=OFF
const byte ArduinoTx::MODEL_SWITCH_PIN = 8;	// opened=use model selected by the MODEL command (CDS global variable), closed=use model corresponding to the ADS global variable

// PPM signal -----------------------------------------------------------------

#if CHANNELS <= 6
const unsigned int ArduinoTx::PPM_PERIOD = 20000;	// microseconds; send PPM sequence every 20ms for 6 channels
const unsigned int ArduinoTx::PPM_LOW = 400;		// microseconds; fixed channel sync pulse width in the PPM signal
#else
const unsigned int ArduinoTx::PPM_PERIOD = 22000;	// microseconds; send PPM sequence every 22ms if more than 6 channels
const unsigned int ArduinoTx::PPM_LOW = 300;		// microseconds; fixed channel sync pulse width in the PPM signal
#endif

// Morse codes flashed on the Led --------------------------------------------
// they must be defined in ArduinotxLed::LedMorseCodes_str[] and BuzzerMorseCodes_str[]
const char ArduinoTx::LEDCHAR_INIT = '0';				// ----- undefined, never displayed
const char ArduinoTx::LEDCHAR_COMMAND = 'C';			// -.-. command mode
const char ArduinoTx::LEDCHAR_ALARM_EEPROM = 'P';		// .--. settings failed to load from EEPROM
const char ArduinoTx::LEDCHAR_ALARM_THROTTLE = 'T';	// - throttle security check failed
const char ArduinoTx::LEDCHAR_ALARM_BATTERY = 'B';		// -... low battery voltage

/* 
** Variables allocated in arduinotx.ino  --------------------------------------------
*/

// Command mode interpreter
extern ArduinotxCmd Command_obj;
// EEPROM manager
extern ArduinotxEeprom Eeprom_obj;
// Led manager
extern ArduinotxLed Led_obj;
#ifdef BUZZER_ENABLED
// Buzzer manager
extern ArduinotxBuzz Buzzer_obj;
#endif
/*
** Public interface ------------------------------------------------------------
*/

ArduinoTx::ArduinoTx() {
	// MODE_SWITCH	RunMode
	//	opened		RUNMODE_TRANSMISSION
	//	closed		RUNMODE_COMMAND
	RunMode_int = RUNMODE_INIT;

	CurrentDataset_byt = 0; // Dataset (model number) currently loaded in RAM
	TxAlarm_int = ALARM_NONE; // current alarm state
	DualRate_bool = false; // true=dual rate ON
	ThrottleCut_bool = false; // true=throttle cut off, false=throttle enabled
	EngineEnabled_bool = false; // will be updated by Refresh()
	SettingsLoaded_bool = false; // set by Init() at startup
	CommitChanges_bool = false; // set by process_command_line() after changing a variable, reset by loop()
}

void ArduinoTx::Init() {

	// Configure all unused digital pins as inputs with pull-up
	// because floating pins would increase current consumption
	for (byte idx_byt=2; idx_byt <= 13; idx_byt++) {
		switch (idx_byt) {
#ifdef BUZZER_ENABLED
			case BUZZER_PIN:
#endif
			case LED_PIN:
			case tx_PIN:
				pinMode(idx_byt, OUTPUT);
				break;
			
			default:
				pinMode(idx_byt, INPUT_PULLUP);
				break;
		}
	}
	
	// Configure all unused analog pins as inputs with pull-up
#ifndef BATCHECK_ENABLED // BATCHECK_PIN is A7
	pinMode(BATCHECK_PIN, INPUT_PULLUP);
#endif
#if NPOTS <= 6	
	pinMode(A6, INPUT_PULLUP);
#endif
#if NPOTS <= 5	
	pinMode(A5, INPUT_PULLUP);
#endif
#if NPOTS <= 4	
	pinMode(A4, INPUT_PULLUP);
#endif
#if NPOTS <= 3	
	pinMode(A3, INPUT_PULLUP);
#endif
#if NPOTS <= 2	
	pinMode(A2, INPUT_PULLUP);
#endif
#if NPOTS <= 1	
	pinMode(A1, INPUT_PULLUP);
#endif
#if NPOTS == 0	
	pinMode(A0, INPUT_PULLUP);
#endif
	
	if (Eeprom_obj.CheckEEProm() > 0) {
		load_settings();
		SettingsLoaded_bool = true;
		TxAlarm_int = ALARM_NONE;
	}
	else {
		SettingsLoaded_bool = false;
		TxAlarm_int = ALARM_EEPROM;  // clear by Reset of Arduino board
	}

}	

// called by process_command_line() after changing a variable, reset by Refresh() 
void ArduinoTx::CommitChanges() {
	CommitChanges_bool = true;
}

// Update ArduinoTx state, called by loop() state every TXREFRESH_PERIOD ms
void ArduinoTx::Refresh() {
	// set CurrentDataset_byt according to the model switch
	byte current_dataset_byt = get_selected_dataset(); 
	if (current_dataset_byt != CurrentDataset_byt) {
		// Load settings of newly selected model
		load_settings(); // updates CurrentDataset_byt
	}
	
	// Read other transmitter special switches
	DualRate_bool = digitalRead(DUALRATE_SWITCH_PIN);
	ThrottleCut_bool = digitalRead(THROTTLECUT_SWITCH_PIN);
	
	// set RunMode according to switches 
	RunMode_int = refresh_runmode();
	
	// reload settings if they were updated while in command mode
	if (RunMode_int == RUNMODE_COMMAND && CommitChanges_bool) {
		load_settings();
		CommitChanges_bool = false;
	}
	
	if (!EngineEnabled_bool) {
		// re-enable throttle if the corresponding control has been reset to its lowest value
		EngineEnabled_bool = check_throttle() == 1;
	}

#ifdef BATCHECK_ENABLED
	const unsigned long BATCHECK_PERIOD = 5000 ; // ms
	static unsigned long Last_BatCheck_uln = 0;
	unsigned long now_uln = millis(); // timer overflows after 50 days
	if (now_uln >= Last_BatCheck_uln + BATCHECK_PERIOD || now_uln < Last_BatCheck_uln) {
		check_battery();
		Last_BatCheck_uln = now_uln;
	}
#endif
	// update the morse character displayed by the Led
	refresh_led_code();
	
}

// Read the input control corresponding to given channel, using the array of assignement of potentiometers and switches
// chan_byt : 0-based, channel number - 1
// Return value: calibrated value [0, 1023]
// It takes about 100 microseconds to read an analog input
unsigned int ArduinoTx::ReadControl(byte chan_byt) {
	unsigned int retval_int = 0;
	byte ctrl_type_byt = get_channel_var(chan_byt, CHAN_ICT);
	byte ctrl_number_byt = get_channel_var(chan_byt, CHAN_ICN);
	switch (ctrl_type_byt) {
		case ICT_ANALOG: 
			if (ctrl_number_byt > 0 && ctrl_number_byt <= NPOTS)
				retval_int = read_potentiometer(ctrl_number_byt);
			break;
		case ICT_DIGITAL:
			if (ctrl_number_byt > 0 && ctrl_number_byt <= NSWITCHES)
				retval_int = digitalRead(get_switch_pin(ctrl_number_byt)) == HIGH ? 1023: 0; // switch
			break;
		case ICT_MIXER: {
			long value_lng = 0L;
			byte pot_number_byt = 0;
			--ctrl_number_byt; // get_mixer_var() expects a 0-based mixer index
			// mixer input 1
			pot_number_byt = get_mixer_var(ctrl_number_byt, MIX_N1M);
			if (pot_number_byt > 0 && pot_number_byt <= NPOTS)
				value_lng = (read_potentiometer(pot_number_byt) - 512L) * get_mixer_var(ctrl_number_byt, MIX_P1M);
			// mixer input 2
			pot_number_byt = get_mixer_var(ctrl_number_byt, MIX_N2M);
			if (pot_number_byt > 0 && pot_number_byt <= NPOTS)
				value_lng += (read_potentiometer(pot_number_byt) - 512L) * get_mixer_var(ctrl_number_byt, MIX_P2M);
			// resulting value
			retval_int = constrain(512L + (value_lng / 100L), 0, 1023);
			}
			break;
		case ICT_OFF:
			retval_int = 0; // do not read actual value and always return 0
			break;
	}
	return retval_int;
}

// Compute the channel pulse corresponding to given analog value
// chan_byt : 0-based, channel number - 1
// ana_value_int : read from input: [0,1023]
// Return value: PWM pulse width, microseconds
// Warning: calling Serial.print() within this method will probably hang the program, a safer way is to display global var DebugValue_int in loop() :
//~ extern unsigned int DebugValue_int;
unsigned int ArduinoTx::ComputeChannelPulse(byte chan_byt, unsigned int ana_value_int) {
	unsigned int retval_int = 0;
	unsigned int value_int = ana_value_int;
	
	byte throttle_channel_byt = get_model_var(MOD_THC) - 1; // 0-based throttle chan number
	if (ThrottleCut_bool || !EngineEnabled_bool)
		if (chan_byt == throttle_channel_byt)
			value_int = 0; // cut throttle
	
	// Dual rate and Exponential do not apply to the throttle channel
	if (DualRate_bool && chan_byt != throttle_channel_byt) {
		// apply exponential
		// this is the only place where floating point arithmetics is required
		byte expo_byt = get_channel_var(chan_byt, CHAN_EXP); // 0=none, 25=medium, 50=strong 100=too much
		if (expo_byt != 0) {
			float expoval_flt = expo_byt / 10.0;
			float value_flt = 2.0 * ((value_int / 1023.0) - 0.5); // map to [-1, +1] range
			value_flt = value_flt * exp(abs(expoval_flt * value_flt)) / exp(expoval_flt);
			value_int = 512 + int(511.5 * value_flt); // map to [0, 1023] range
		}
		else {
			// apply dual rate if no exponential for this channel
			unsigned int offset_int = get_channel_var(chan_byt, CHAN_DUA);
			if (offset_int != 100) {
				offset_int = offset_int << 9; // multiply by 512, max value = 100*512 = 51200
				value_int = map(value_int, 0, 1023, 51200 - offset_int, 51100 + offset_int) / 100;
			}
		}
	}
	
	// apply subtrim
	int trim_int = get_channel_var(chan_byt, CHAN_SUB);
	if (trim_int) {
		// approximate 1024/100 = 10.24 ~ 10
		value_int += 10 * trim_int;
		if (value_int > 32767)
			value_int = 0L; // underflow
		else if (value_int > 1023)
			value_int = 1023;
	}
	
	//~ // debug: display value_int in loop()
	//~ if (chan_byt == elevator_channel_byt)
		//~ DebugValue_int = value_int;
	
	// apply end points
	// approximate 512/100 = 5.12 ~ 5
	unsigned int endpoint_int = 5 * (100L - get_channel_var(chan_byt, CHAN_EPL));
	if (value_int < endpoint_int)
		value_int = endpoint_int;
	else {
		endpoint_int = 511 + (5 * get_channel_var(chan_byt, CHAN_EPH));
		if (value_int > endpoint_int)
				value_int = endpoint_int;
	}
	
	unsigned int low_int = get_channel_var(chan_byt, CHAN_PWL); // Minimal pulse width for this channel, in microseconds
	unsigned int high_int = get_channel_var(chan_byt, CHAN_PWH); // Maximal pulse width for this channel, in microseconds
	// apply reverse
	if (get_channel_var(chan_byt, CHAN_REV)) {
		unsigned int tmp_int = low_int;
		low_int = high_int;
		high_int = tmp_int;
	}
	
	// Map analog inputs to PPM rates
	retval_int =  map(value_int, 0, 1023, low_int, high_int);
	
	return retval_int;
}

/*
** Private Implementation ------------------------------------------------------------
*/

// Return the dataset number corresponding to the model switch state
byte ArduinoTx::get_selected_dataset() {
	return get_global_var(digitalRead(MODEL_SWITCH_PIN) ? GLOBAL_CDS:GLOBAL_ADS);
}
	
// load settings values from EEPROM
// updates CurrentDataset_byt
void ArduinoTx::load_settings() {
	Eeprom_obj.GetGlobal(Global_int); // load values of GLOBAL_CDS and GLOBAL_ADS
	CurrentDataset_byt = get_selected_dataset(); // Dataset (model number) currently loaded in RAM
	Eeprom_obj.GetDataset(CurrentDataset_byt, DatasetModel_int, DatasetMixers_int, DatasetChannels_int);
}

// Update the morse character displayed by the Led
void ArduinoTx::refresh_led_code() {
	static char Current_ledcode_char = LEDCHAR_INIT;
	char ledcode_char = LEDCHAR_INIT;
	
	// alarm modes have priority over normal modes
	switch(TxAlarm_int) {
		// alarm modes
		case ALARM_EEPROM:
			ledcode_char = LEDCHAR_ALARM_EEPROM;
			break;
		
		case ALARM_THROTTLE:
			ledcode_char = LEDCHAR_ALARM_THROTTLE;
			break;
		
		case ALARM_BATTERY:
			ledcode_char = LEDCHAR_ALARM_BATTERY;
			break;
		
		case ALARM_NONE:
			// normal modes
			switch (RunMode_int) {
				case RUNMODE_COMMAND:
					ledcode_char = LEDCHAR_COMMAND;
					break;
				case RUNMODE_TRANSMISSION:
					ledcode_char = CurrentDataset_byt + '0';
					break;
				case RUNMODE_INIT: // suppress compiler warning
					break;
			}
			break;
	}
	if (ledcode_char != Current_ledcode_char) {
		Led_obj.SetCode(ledcode_char);
#ifdef BUZZER_ENABLED
		if (TxAlarm_int != ALARM_NONE)
			Buzzer_obj.SetCode(ledcode_char, BUZZER_REPEAT, 400);
		else
			Buzzer_obj.SetCode(ledcode_char, 3, 800);
#endif
		Current_ledcode_char = ledcode_char;
	}
}

// set RunMode according to switches settings
// MODE_SWITCH	RunMode
//	opened		RUNMODE_TRANSMISSION
//	closed		RUNMODE_COMMAND
// Return value: current run mode
ArduinoTx::RunMode ArduinoTx::refresh_runmode() {
	RunMode retval_int = RUNMODE_INIT;
	static RunMode Last_runmode_int = RUNMODE_INIT;
	
	// read the mode switch
	byte mode_switch_bool = digitalRead(MODE_SWITCH_PIN);
	
	if (!SettingsLoaded_bool) {
		// Invalid settings have been detected in EEPROM
		// and the Alarm Led is lit.
		// Ignore actual Mode switch position and force Command mode.
		// User should enter command INIT to properly initialize all settings,
		// then user must reset the Arduino (press the reset switch or cycle the power) 
		mode_switch_bool = LOW;
	}
	
	if (mode_switch_bool)
		retval_int = RUNMODE_TRANSMISSION;
	else
		retval_int = RUNMODE_COMMAND;
	
	if (retval_int != Last_runmode_int) {
		if (retval_int == RUNMODE_COMMAND) {
			// entering command mode
			// open serial and print command prompt
			Command_obj.InitCommand();
		}
		else {
			if (Last_runmode_int == RUNMODE_COMMAND) {
				// leaving command mode: close serial
				Command_obj.EndCommand();
			}
		}
		Last_runmode_int = retval_int;
	}
	return retval_int;
}

// Throttle security check
// updates TxAlarm_int
// return values:
/// 1 if throttle is < GLOBAL_TSC or if no throttle channel
// 0 if throttle is >= GLOBAL_TSC and sets ALARM_THROTTLE
byte ArduinoTx::check_throttle() {
	byte retval_byt = 1;
	static unsigned int Average_int = 2 * get_global_var(GLOBAL_TSC); // average of last 8 analog readings
	static byte Count_byt = 0;
	int sample_int = 0;
	byte throttle_chan_byt = get_model_var(MOD_THC);
	if (throttle_chan_byt) {
		do {
			sample_int = ReadControl(throttle_chan_byt - 1);
			Average_int = (7 * Average_int + sample_int) >> 3; // >>3 divides by 8
			if (Count_byt == 30)
					break;
				else
					Count_byt++;
		} while (1);
		
		retval_byt = Average_int < (unsigned int)get_global_var(GLOBAL_TSC) ? 1:0;
		if (retval_byt == 0)
			TxAlarm_int = ALARM_THROTTLE; // Throttle security check has top priority: overwrite all other alarms
		else if (TxAlarm_int == ALARM_THROTTLE)
			TxAlarm_int = ALARM_NONE;
	}
	//~ if (retval_byt == 0)
		//~ aprintfln("check_throttle() pot=%d av=%u returns %d", sample_int, Average_int, retval_byt);
	return retval_byt;
}

#ifdef BATCHECK_ENABLED
// Battery voltage check
// updates TxAlarm_int
// return values:
// 1 if voltage is > GLOBAL_BAT
// 0 if throttle is <= GLOBAL_BAT and sets ALARM_BATTERY
byte ArduinoTx::check_battery() {
	byte retval_byt = 1;
	static unsigned int Average_int = 2 * get_global_var(GLOBAL_BAT); // average of last 8 analog readings
	static byte Count_byt = 0;
	static int Last_sample_int = 0;
	int sample_int = 0;
	do {
		sample_int = analogRead(BATCHECK_PIN);
		if (sample_int >=  Last_sample_int >> 1) {
			Average_int = (7 * Average_int + sample_int) >> 3; // >>3 divides by 8
			Last_sample_int = sample_int;
		}
		else {
			// False readings happen once in a while, may be due to analogRead() being interrupted by ISR1 ?
			//~ aprintfln("check_battery() ignore %d", sample_int);
			sample_int =  Last_sample_int; // ignore invalid sample
		}
		if (Count_byt == 30)
			break;
		else
			Count_byt++;
	} while (1);
	
	retval_byt = Average_int > (unsigned int)get_global_var(GLOBAL_BAT) ? 1:0;
	if (retval_byt == 0) {
		if (TxAlarm_int == ALARM_NONE)
			TxAlarm_int = ALARM_BATTERY;
	}
	else if (TxAlarm_int == ALARM_BATTERY)
		TxAlarm_int = ALARM_NONE;
	//~ aprintfln("check_battery() pin=%d av=%u returns %d", sample_int, Average_int, retval_byt);
	return retval_byt;
}
#endif


// Return calibrated value of given potentiometer
unsigned int ArduinoTx::read_potentiometer(byte pot_number_byt) {
	unsigned int retval_int = 0;
	unsigned int chan_cal_int = get_calibration_var(pot_number_byt, CAL_LOW); // lowest value returned by the potentiometer corresponding to given channel
	unsigned int chan_cah_int = get_calibration_var(pot_number_byt, CAL_HIGH); // highest value returned by the potentiometer corresponding to given channel
	retval_int = analogRead(get_pot_pin(pot_number_byt));
	retval_int = constrain(retval_int, chan_cal_int, chan_cah_int);
	retval_int = map(retval_int, chan_cal_int, chan_cah_int, 0, 1023);
	return retval_int;
}
