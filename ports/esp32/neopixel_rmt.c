/*
 * Driver based on the FastLED implementation. Original contributors:
 * Copyright (c) 2018 Samuel Z. Guyer
 * Copyright (c) 2017 Thomas Basler
 * Copyright (c) 2017 Martin F. Falatic
 * 
 * Ported to micropython.
 * Copyright (c) 2020 Carsten B. L. Tschense
 * 
 * /

/*
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#include "neopixel_rmt.h"
#include "driver/rmt.h"
#include "freertos/semphr.h"
#include "soc/rmt_struct.h"
#include "esp_intr_alloc.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"

#include "py/runtime.h"


// Configure these based on your project needs ********
#define LED_RMT_TX_CHANNEL RMT_CHANNEL_0
#define LED_RMT_TX_GPIO 4
// ****************************************************

#define BITS_PER_LED_CMD 24 

#define T0H 16  // 0 bit high time
#define T0L 34  // 0 bit low time

#define T1H 32  // 1 bit high time
#define T1L 18  // 1 bit high time

// -- Configuration constants
#define DIVIDER             2 /* 4, 8 still seem to work, but timings become marginal */
#define MAX_PULSES         64 /* A channel has a 64 "pulse" buffer */
#define PULSES_PER_FILL    24 /* One pixel's worth of pulses */

// -- Convert ESP32 CPU cycles to RMT device cycles, taking into account the divider
#define F_CPU_RMT                   (  80000000L)
#define RMT_CYCLES_PER_SEC          (F_CPU_RMT/DIVIDER)
#define RMT_CYCLES_PER_ESP_CYCLE    (F_CPU / RMT_CYCLES_PER_SEC)
#define ESP_TO_RMT_CYCLES(n)        ((n) / (RMT_CYCLES_PER_ESP_CYCLE))

// -- Number of cycles to signal the strip to latch
#define NS_PER_CYCLE                ( 1000000000L / RMT_CYCLES_PER_SEC )
#define NS_TO_CYCLES(n)             ( (n) / NS_PER_CYCLE )
#define RMT_RESET_DURATION          NS_TO_CYCLES(50000)

// -- Number of RMT channels to use (up to 8)
//    Redefine this value to 1 to force serial output
#ifndef FASTLED_RMT_MAX_CHANNELS
#define FASTLED_RMT_MAX_CHANNELS 8
#endif

static xSemaphoreHandle gTX_sem = NULL;
static intr_handle_t gRMT_intr_handle = NULL;
static bool gInitialized = false;
volatile uint32_t * mRMT_mem_ptr;
uint16_t mCurPulse;
rmt_channel_t  mRMT_channel;

static int gNumControllers = 0;
static int gNumStarted = 0;
static int gNumDone = 0;
static int gNext = 0;

typedef struct {
    bool enabled;
    uint16_t curPixel;
    uint8_t *leds;
    uint16_t length;
    gpio_num_t pin;
    rmt_channel_t channel;
} controller;

rmt_item32_t   mZero = {{{T0H, 1, T0L, 0}}};
rmt_item32_t   mOne = {{{T1H, 1, T1L, 0}}};


static controller gOnChannel[FASTLED_RMT_MAX_CHANNELS];

__attribute__ ((always_inline)) inline static uint32_t __clock_cycles() {
  uint32_t cyc;
  __asm__ __volatile__ ("rsr %0,ccount":"=a" (cyc));
  return cyc;
}

static void IRAM_ATTR interruptHandler(void *arg);
void IRAM_ATTR fillNext(uint16_t channel);
static void IRAM_ATTR doneOnChannel(rmt_channel_t channel, void * arg);
static void IRAM_ATTR startNext(int channel);

void pixel_init(void)
{
  rmt_config_t config;
  config.rmt_mode = RMT_MODE_TX;
  config.channel = LED_RMT_TX_CHANNEL;
  config.gpio_num = LED_RMT_TX_GPIO;
  config.mem_block_num = 1;
  config.tx_config.loop_en = false;
  config.tx_config.carrier_en = false;
  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level = 0;
  config.clk_div = 2;

  gOnChannel[0].enabled = true;
  gOnChannel[0].pin = LED_RMT_TX_GPIO;
  gOnChannel[0].curPixel = 0;
  gOnChannel[0].channel = LED_RMT_TX_CHANNEL;
  gOnChannel[1].enabled = false;
  gOnChannel[2].enabled = false;
  gOnChannel[3].enabled = false;
  gOnChannel[4].enabled = false;
  gOnChannel[5].enabled = false;
  gOnChannel[6].enabled = false;
  gOnChannel[7].enabled = false;

  gNumControllers = 1;

  ESP_ERROR_CHECK(rmt_config(&config));
  ESP_ERROR_CHECK(rmt_set_tx_thr_intr_en(LED_RMT_TX_CHANNEL, true, PULSES_PER_FILL));

  if (gTX_sem == NULL) {
        gTX_sem = xSemaphoreCreateBinary();
        xSemaphoreGive(gTX_sem);
    }
                
    if (gRMT_intr_handle == NULL) {
        ESP_ERROR_CHECK(esp_intr_alloc(ETS_RMT_INTR_SOURCE, ESP_INTR_FLAG_LEVEL3, interruptHandler, 0, &gRMT_intr_handle));
    }

    gInitialized = true;
}

static void IRAM_ATTR interruptHandler(void *arg)
{
    // -- The basic structure of this code is borrowed from the
    //    interrupt handler in esp-idf/components/driver/rmt.c
    uint32_t intr_st = RMT.int_st.val;
    uint8_t channel;

    for (channel = 0; channel < FASTLED_RMT_MAX_CHANNELS; channel++) {
        int tx_done_bit = channel * 3;
        int tx_next_bit = channel + 24;


        if (gOnChannel[channel].enabled == true) {

            // -- More to send on this channel
            if (intr_st & BIT(tx_next_bit)) {
                RMT.int_clr.val |= BIT(tx_next_bit);
                
                // -- Refill the half of the buffer that we just finished,
                //    allowing the other half to proceed.
                fillNext(channel);
            } else {
                // -- Transmission is complete on this channel
                if (intr_st & BIT(tx_done_bit)) {
                    RMT.int_clr.val |= BIT(tx_done_bit);
                    doneOnChannel(channel, 0);
                }
            }
        }
    }
}

void IRAM_ATTR showPixels(uint8_t *pixels, uint16_t channel_in, uint16_t length)
{
    if (gNumStarted == 0) {
        // -- First controller: make sure everything is set up
        // -- Only need to do this once
        if ( ! gInitialized) {
            pixel_init();
        }
        xSemaphoreTake(gTX_sem, portMAX_DELAY);
    }

    // -- Initialize the local state, save a pointer to the pixel
    //    data. We need to make a copy because pixels is a local
    //    variable in the calling function, and this data structure
    //    needs to outlive this call to showPixels.
    /*for (int i = 0; i < NUM_LEDS; i++)
        gOnChannel[channel_in].leds[i] = pixels.leds[i];*/
    gOnChannel[channel_in].leds = pixels;
    gOnChannel[channel_in].length = length;

    // -- Keep track of the number of strips we've seen
    gNumStarted++;

    // -- The last call to showPixels is the one responsible for doing
    //    all of the actual worl
    if (gNumStarted == gNumControllers) {
        gNext = 0;

        // -- First, fill all the available channels
        int channel = 0;
        while (channel < FASTLED_RMT_MAX_CHANNELS && gNext < gNumControllers) {
            startNext(channel);
            channel++;
        }

        // -- Start them all
        for (int i = 0; i < channel; i++) {
            ESP_ERROR_CHECK(rmt_tx_start(LED_RMT_TX_CHANNEL, true));
        }

        // -- Wait here while the rest of the data is sent. The interrupt handler
        //    will keep refilling the RMT buffers until it is all sent; then it
        //    gives the semaphore back.
        //xSemaphoreTake(gTX_sem, portMAX_DELAY);
        //xSemaphoreGive(gTX_sem);

        // -- Reset the counters
        gNumStarted = 0;
        gNumDone = 0;
        gNext = 0;
    }
}

void IRAM_ATTR startOnChannel(int channel)
{
    // -- Assign this channel and configure the RMT
    mRMT_channel = LED_RMT_TX_CHANNEL;

    // -- Store a reference to this controller, so we can get it
    //    inside the interrupt handler
    gOnChannel[channel].enabled = true;

    // -- Assign the pin to this channel
    //ESP_ERROR_CHECK(rmt_set_pin(mRMT_channel, RMT_MODE_TX, gOnChannel[channel].pin));

    // -- Use our custom driver to send the data incrementally

    // -- Initialize the counters that keep track of where we are in
    //    the pixel data.
    mRMT_mem_ptr = & (RMTMEM.chan[mRMT_channel].data32[0].val);
    mCurPulse = 0;

    // -- Store 2 pixels worth of data (two "buffers" full)
    fillNext(channel);
    fillNext(channel);

    // -- Turn on the interrupts
    ESP_ERROR_CHECK(rmt_set_tx_intr_en(mRMT_channel, true));
}

static void IRAM_ATTR startNext(int channel)
{
    if (gNext < gNumControllers) {
        startOnChannel(channel);
        gNext++;
    }
}

static void IRAM_ATTR doneOnChannel(rmt_channel_t channel, void * arg)
{
    portBASE_TYPE HPTaskAwoken = 0;

    // -- Turn off output on the pin
    //gpio_matrix_out(gOnChannel[channel].pin, 0x100, 0, 0);

    gOnChannel[channel].enabled = false;
    gOnChannel[channel].curPixel = 0;
    gNumDone++;

    if (gNumDone == gNumControllers) {
        // -- If this is the last controller, signal that we are all done
        xSemaphoreGiveFromISR(gTX_sem, &HPTaskAwoken);
        if(HPTaskAwoken == pdTRUE) portYIELD_FROM_ISR();
        } else {
        // -- Otherwise, if there are still controllers waiting, then
        //    start the next one on this channel
        if (gNext < gNumControllers) {
            startNext(channel);
            // -- Start the RMT TX operation
            //    (I'm not sure if this is necessary here)
            rmt_tx_start(channel, true);
        }
    }
}

void IRAM_ATTR fillNext(uint16_t channel)
{
    if (gOnChannel[channel].curPixel < gOnChannel[channel].length) {
        uint32_t one_val = mOne.val;
        uint32_t zero_val = mZero.val;

        // -- Get a pixel's worth of data
        register uint32_t pixel = gOnChannel[channel].leds[gOnChannel[channel].curPixel] << 8 | gOnChannel[channel].leds[gOnChannel[channel].curPixel + 1] << 16 | gOnChannel[channel].leds[gOnChannel[channel].curPixel + 2];
        gOnChannel[channel].curPixel += 3;

        // -- Use locals for speed
        volatile register uint32_t * pItem =  mRMT_mem_ptr;
        register uint16_t curPulse = mCurPulse;
        
        // Shift bits out, MSB first, setting RMTMEM.chan[n].data32[x] to the 
        // rmt_item32_t value corresponding to the buffered bit value
        for (register uint32_t j = 0; j < 24; j++) {
            uint32_t val = (pixel & 0x800000L) ? one_val : zero_val;
            *pItem++ = val;
            // Replaces: RMTMEM.chan[mRMT_channel].data32[mCurPulse].val = val;

            pixel <<= 1;
            curPulse++;

            if (curPulse == MAX_PULSES) {
                pItem = & (RMTMEM.chan[mRMT_channel].data32[0].val);
                curPulse = 0;
            }
        }

        // -- Store the new values back into the object
        mCurPulse = curPulse;
        mRMT_mem_ptr = pItem;
    } else {
        // -- No more data; signal to the RMT we are done
        for (uint32_t j = 0; j < 8; j++) {
            * mRMT_mem_ptr++ = 0;
        }
    }   
}
