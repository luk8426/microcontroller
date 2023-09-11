/********************************************************************
*
*		Test_LCD_Output16BitWord
*
*		Tests für Funktion LCD_Output16BitWord 
*		(LCD Aufgabe des Mikrocomputerpraktikums)
*
*		Benutzung: die Testfunktion ins Hauptprogramm einbinden
*		Beispiel:

#include "Test_LCD_Output16BitWord.h"

int failed_tests;

void main()
{
		mcpr_SetSystemCoreClock();
		
		// Testfunktion:
		failed_tests = test_LCD_Output16BitWord();
		// ...
}

* 	Im Debugger können Sie einen Break-Point setzten und
*		den Rückgabewert failed überprüfen.
*		failed = 0:		kein Fehler, ihre Funktion ist fehlerfrei!
*		failed > 0:		es sind Fehler aufgetreten. 
*
************************************************************************/

#include <inttypes.h>
#include "STM32F4xx.h"

#include "Test_LCD_Output16BitWord.h"

extern void LCD_Output16BitWord( uint16_t data );

typedef struct {
	uint16_t data;
	uint32_t dpre, epre, dpost, epost;
} gpiotest_t;

gpiotest_t gpio_testcases[] = {
	{ 0xFFFF, 0, 0, 0xC703, 0xFF80 },
	{ 0x0001, 0, 0, 0x4000, 0x0000 },  						// D0 -> PD14
	{ 0x0001, 0xFFFF, 0xFFFF, 0x78FC, 0x007F },  	// D0 -> PD14
	{ 0x0002, 0, 0, 0x8000, 0x0000 },  						// D1 -> PD15
	{ 0x0002, 0xFFFF, 0xFFFF, 0xB8FC, 0x007F },  	// D1 -> PD15
	{ 0x0004, 0, 0, 0x0001, 0x0000 },  						// D2 -> PD0
	{ 0x0004, 0xFFFF, 0xFFFF, 0x38FD, 0x007F },  	// D2 -> PD0
	{ 0x0008, 0, 0, 0x0002, 0x0000 },  						// D3 -> PD1
	{ 0x0008, 0xFFFF, 0xFFFF, 0x38FE, 0x007F },  	// D3 -> PD1
	{ 0x0010, 0, 0, 0x0000, 0x0080 },  						// D4 -> PE7
	{ 0x0010, 0xFFFF, 0xFFFF, 0x38FC, 0x00FF },  	// D4 -> PE7
	{ 0x0020, 0, 0, 0x0000, 0x0100 },  						// D5 -> PE8
	{ 0x0020, 0xFFFF, 0xFFFF, 0x38FC, 0x017F },  	// D5 -> PE8
	{ 0x0040, 0, 0, 0x0000, 0x0200 },  						// D6 -> PE9
	{ 0x0040, 0xFFFF, 0xFFFF, 0x38FC, 0x027F },  	// D6 -> PE9
	{ 0x0080, 0, 0, 0x0000, 0x0400 },  						// D7 -> PE10
	{ 0x0080, 0xFFFF, 0xFFFF, 0x38FC, 0x047F },  	// D7 -> PE10
	{ 0x0100, 0, 0, 0x0000, 0x0800 },  						// D8 -> PE11
	{ 0x0100, 0xFFFF, 0xFFFF, 0x38FC, 0x087F },  	// D8 -> PE11
	{ 0x0200, 0, 0, 0x0000, 0x1000 },  						// D9 -> PE12
	{ 0x0200, 0xFFFF, 0xFFFF, 0x38FC, 0x107F },  	// D9 -> PE12
	{ 0x0400, 0, 0, 0x0000, 0x2000 },  						// D10 -> PE13
	{ 0x0400, 0xFFFF, 0xFFFF, 0x38FC, 0x207F },  	// D10 -> PE13
	{ 0x0800, 0, 0, 0x0000, 0x4000 },  						// D11 -> PE14
	{ 0x0800, 0xFFFF, 0xFFFF, 0x38FC, 0x407F },  	// D11 -> PE14
	{ 0x1000, 0, 0, 0x0000, 0x8000 },  						// D12 -> PE15
	{ 0x1000, 0xFFFF, 0xFFFF, 0x38FC, 0x807F },  	// D12 -> PE15
	{ 0x2000, 0, 0, 0x0100, 0x0000 },  						// D13 -> PD8
	{ 0x2000, 0xFFFF, 0xFFFF, 0x39FC, 0x007F },  	// D13 -> PD8
	{ 0x4000, 0, 0, 0x0200, 0x0000 },  						// D14 -> PD9
	{ 0x4000, 0xFFFF, 0xFFFF, 0x3AFC, 0x007F },  	// D14 -> PD9
	{ 0x8000, 0, 0, 0x0400, 0x0000 },  						// D15 -> PD10
	{ 0x8000, 0xFFFF, 0xFFFF, 0x3CFC, 0x007F }  	// D15 -> PD10
};

int test_LCD_Output16BitWord(void)
{
	int ncount = sizeof(gpio_testcases)/sizeof(gpio_testcases[0]);
	int i, ok = 0, failed = 0;
	
	// Enable GPIOE und GPIOD
	RCC->AHB1ENR |= 0x00000018;
	
	for(i = 0; i< ncount; i++ ) {
		// setup
		GPIOD->ODR = gpio_testcases[i].dpre;
		GPIOE->ODR = gpio_testcases[i].epre;
		// test case
		LCD_Output16BitWord( gpio_testcases[i].data );
		// validate
		if( (GPIOD->ODR == gpio_testcases[i].dpost) && (GPIOE->ODR == gpio_testcases[i].epost) ) {
			ok++;
		} else {
			failed++;
		}
	}
	return failed;
	
}

