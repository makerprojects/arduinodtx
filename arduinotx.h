/* arduinotx.h - Open source RC transmitter software for the Arduino 
** 05-11-2012
** 02-03-2013 deleted TEST_MODE
*/

#ifndef arduinotx_h
#define arduinotx_h

enum RunMode {
	RUNMODE_INIT,
	RUNMODE_TRANSMISSION,	// send PPM to receiver
	RUNMODE_COMMAND		// change configuration settings through Serial link
};

byte load_settings();
RunMode update_RunMode();
byte check_throttle();
unsigned int read_control(byte chan_byt);
unsigned int compute_channel_pulse(byte chan_byt, unsigned int ana_value_int, byte enable_calibration_bool);
void send_ppm();
char get_led_code(RunMode runmode_int);
void set_led_code(const char c_char);
#endif
