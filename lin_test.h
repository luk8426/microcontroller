#ifndef LIN_TEST_H_
#define LIN_TEST_H_
/**********************************************************************************


Die Dateien lin_test.c und lin_test.h ermöglichen einen einfachen Test der Prüfsummenfunktion.

Voraussetzungen: 
- die Prüfsumme für das LIN-Telegramm wird mit einer eigenen Funktion berechnet
- die Funktion hat die folgende Signatur:

Signatur:
=========
  uint8_t func(uint8_t * dataptr, uint8_t size, uint8_t identifier)

Parameter:
==========
dataptr:    ein Zeiger auf die Daten, die mit dem LIN versendet werden sollen. 
            Die Daten müssen zusammenhängend im Speicher liegen. Man kann auch direkt
            eine Adresse angeben.

size:       die Anzahl der Bytes, die übertragen werden sollen
identifier: der LIN-Identifier

Rückgabewert:
=============
Die Prüfsumme.

Habe ich z.B. folgende Funktion implementiert:

uint8_t meine_lin_pruefsumme( uint8_t * dataptr, uint8_t size, uint8_t identifier)

so kann ich sie mit folgendem Aufruf testen:

  int testok;

  testok = test_lin_checksum(meine_lin_pruefsumme);

Falls testok == 0 ist, wurde die Prüfsumme in allen Testfällen richtig berechnet. 
Andernfalls funktionert die Funktion nicht korrekt.

*************************************************************************************/

#include <inttypes.h>

typedef uint8_t TEST_FUNC(uint8_t*ptr, uint8_t size, uint8_t identifier);

int test_lin_checksum(TEST_FUNC tfunc);
#endif
