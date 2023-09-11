#include <inttypes.h>
#include "lin_test.h"

typedef struct {
	uint8_t identifier;		// the identifier that needs to be tested
	uint8_t size;			// the size of the data vector
	uint8_t testdata[4];		// the data vector
	uint8_t checksum;		// the checksum
} testvector;

testvector tv[] = {
	{ 0x32, 2, { 0x00, 0x00, 0, 0 }, 0xCD },
	{ 0x32, 2, { 0xFF, 0xFF, 0, 0 }, 0xCD },
	{ 0x32, 1, { 0xFF, 0xFF, 0xFF, 0xFF }, 0xCD },
	{ 0x32, 2, { 0xFF, 0xFF, 0xFF, 0xFF }, 0xCD },
	{ 0x32, 3, { 0xFF, 0xFF, 0xFF, 0xFF }, 0xCD },
	{ 0x32, 4, { 0xFF, 0xFF, 0xFF, 0xFF }, 0xCD },
	{ 0x32, 2, { 0x01, 0x01, 0x01, 0x01 }, 0xCB },
	{ 0x11, 4, { 0x80, 0x80, 0x3F, 0xC1 }, 0xEC },
	{ 0x11, 4, { 0xEE, 0xFF, 0x01, 0x00 }, 0xFE },
	{ 0x43, 4, { 'E', 'N', 'D', 'E' }, 0x9F },
};

int test_lin_checksum(TEST_FUNC tfunc)
{
	int i;
	int tv_size = sizeof(tv) / sizeof( tv[0] );
	int result_failed = 0;
	uint8_t result;
	
	for( i = 0; i < tv_size; i++ ) {
		result = tfunc( tv[i].testdata, tv[i].size, tv[i].identifier );
		if( result != tv[i].checksum ) 
			result_failed++;
	}
	
	return result_failed;
}
