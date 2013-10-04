/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */


//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include <board.h>
#include <pio/pio.h>
#include <pit/pit.h>
#include <aic/aic.h>


//------------------------------------------------------------------------------
//         Local definitions
//------------------------------------------------------------------------------
const Pin pinLED[]={PINS_LEDS};

/// PIT period value in µseconds.
#define PIT_PERIOD          1000

//------------------------------------------------------------------------------
//         Local variables
//------------------------------------------------------------------------------

/// Global timestamp in milliseconds since start of application.
volatile unsigned int timestamp = 0;



//------------------------------------------------------------------------------
//         Local functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Handler for PIT interrupt. Increments the timestamp counter.
//------------------------------------------------------------------------------
void ISR_Pit(void)
{
    unsigned int status;

    // Read the PIT status register
    status = PIT_GetStatus() & AT91C_PITC_PITS;
    if (status != 0) {

        // Read the PIVR to acknowledge interrupt and get number of ticks
        timestamp += (PIT_GetPIVR() >> 20);
    }
}
//------------------------------------------------------------------------------
/// Configure the periodic interval timer to generate an interrupt every
/// millisecond.
//------------------------------------------------------------------------------
void ConfigurePit(void)
{
    // Initialize the PIT to the desired frequency
    PIT_Init(PIT_PERIOD, BOARD_MCK / 1000000);

    // Configure interrupt on PIT
    AIC_DisableIT(AT91C_ID_SYS);
    AIC_ConfigureIT(AT91C_ID_SYS, AT91C_AIC_PRIOR_LOWEST, ISR_Pit);
    AIC_EnableIT(AT91C_ID_SYS);
    PIT_EnableIT();

    // Enable the pit
    PIT_Enable();
}


//------------------------------------------------------------------------------
/// Waits for the given number of milliseconds (using the timestamp generated
/// by the PIT).
/// \param delay  Delay to wait for, in milliseconds.
//------------------------------------------------------------------------------
void Wait(unsigned long delay)
{
    volatile unsigned int start = timestamp;
    unsigned int elapsed;
    do {
        elapsed = timestamp;
        elapsed -= start;
    }
    while (elapsed < delay);
}

//------------------------------------------------------------------------------
//         Exported functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Application entry point. Configures the DBGU, PIT, TC0, LEDs and buttons
/// and makes LED\#1 blink in its infinite loop, using the Wait function.
/// \return Unused (ANSI-C compatibility).
//------------------------------------------------------------------------------
int main(void)
{
         
    // Configuration
	 PIO_Configure(&pinLED,2);
   ConfigurePit();
	 
    
    // Main loop
    while (1) {
			//Set Led 0
			PIO_Set(&pinLED[0]);
      // Wait for 200ms
      Wait(200);
			//Set Led 1
			PIO_Set(&pinLED[1]);
			// Wait for 200ms
			Wait(200);
			//Clear Led 1
			PIO_Clear(&pinLED[1]);
			// Wait for 200ms
			Wait(200);
			//Clear Led 0
			PIO_Clear(&pinLED[0]);
			// Wait for 200ms
			Wait(200);  
    }
}

