//write:commander flash --patch 0x0FE00000:0x11223344:4
//read:commander readmem --range 0x0FE00000:+100
//when execute the function, you can use the command above the verify right or not.
#include "em_msc.h"
#include "sl_se_manager.h"
#include "sl_se_manager_util.h"
void test_direct_write_user_data_region(void)
{
	// User data to write (256 x 32-bit words = 1024 bytes)
	SL_ALIGN(4) static const uint32_t user_data[256] SL_ATTRIBUTE_ALIGN(4) = {
	    0x11223344, 0xAABBCCDD, 0x55AA55AA, 0xAA55AA55,
	    // ... fill in your actual data here ...
	    // remaining entries will default to 0x00000000
	};
	// Command context
	    sl_se_command_context_t cmd_ctx;

	    // Switch SYSCLK to 38 MHz HFRCO (required for SE operations)
	    CMU_HFRCODPLLBandSet(cmuHFRCODPLLFreq_38M0Hz);

	    // Initialize SE Manager
	    sl_se_init();

	    // Step 1: Erase the user data section via SE
	    sl_se_erase_user_data(&cmd_ctx);

	    // Step 2: Write user data via SE
	    // Parameters: context, offset (0 = start of user data), data pointer, size in bytes
	    sl_se_write_user_data(&cmd_ctx, 0, (uint32_t *)user_data, sizeof(user_data)); //sl_se_get_user_data
	    // Restore your application clock if needed
	    CMU_HFRCODPLLBandSet(cmuHFRCODPLLFreq_80M0Hz);  // example: restore to 80 MHz
}
