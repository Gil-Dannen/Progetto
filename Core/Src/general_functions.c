#include "general_functions.h"
#include "state_machine.h"

static uint8_t button = 1;

uint8_t buttonToggled()
{
	uint8_t newRead = readDigital(MF_Button);
	// 1 non premuto 0 premuto

	uint8_t calc = newRead && !button;

	button = newRead;

	return calc;
}
