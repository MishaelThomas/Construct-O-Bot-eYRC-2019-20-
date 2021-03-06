/*

*Team Id: 3302
*Author List: MISHAEL THOMAS, SAURAV PANDEY, AYUSH PAREEK, PULKIT SHARMA
*Filename: CB_Task_1.cpp
*Theme: Construct-O-Bot – eYRC Specific
*Functions: main(),init(),threadCalls(),_delay_ms(unsigned int),Task_1_1(),cleanUp(),detach()
*Global Variables: NONE

*/
#include "CB_Task_1_Sandbox.h"
/*

*Function Name : main
* Input : None
* Output : int to inform the caller that the program exited correctly or incorrectly(C code standard)
* Logic : Starts the communication with the V-REP Simulator and performs the simulations according to the functions called
* Example Call : Called automatically by the Operating System

*/

int main(int argc, char* argv[])
{
	// Establishing the connection with the simulator
	init();
	std::thread t_1(threadCalls);
	
	// Creating a delay of 4 seconds before starting the program
	_delay_ms(4000);

	// Calling Task_1_1() to execute Task_1.1
	// e_shape();
	// Task_1_1();
	// Task_1_2();

	// Ending the connection with the simulator
	cleanUp();
	t_1.detach();
}