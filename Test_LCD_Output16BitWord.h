#ifndef _Test_LCD_Output16BitWord_HEADER
#define _Test_LCD_Output16BitWord_HEADER
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

int failed;

void main()
{
		mcpr_SetSystemCoreClock();
		
		// Testfunktion:
		failed = Test_LCD_Output16BitWord();
		// ...
}

* 	Im Debugger können Sie einen Break-Point setzten und
*		den Rückgabewert failed überprüfen.
*		failed = 0:		kein Fehler, ihre Funktion ist fehlerfrei!
*		failed > 0:		es sind Fehler aufgetreten. 
*
************************************************************************/
extern int test_LCD_Output16BitWord(void);

#endif
