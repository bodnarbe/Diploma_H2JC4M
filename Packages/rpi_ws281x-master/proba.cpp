#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <iostream>
#include <stdio.h>
#include <wiringPiI2C.h>
#include <cstdint>
#include "ws2811.h"


/*
    Defines für ws2811 LED Stripes
*/

#define TARGET_FREQ                             WS2811_TARGET_FREQ
#define GPIO_PIN                                18
#define DMA                                     5
#define STRIP_TYPE                              WS2811_STRIP_RGB        // WS2812/SK6812RGB integrated chip+leds
//#define STRIP_TYPE                            WS2811_STRIP_GBR        // WS2812/SK6812RGB integrated chip+leds
//#define STRIP_TYPE                            SK6812_STRIP_RGBW       // SK6812RGBW (NOT SK6812RGB)
#define LED_COUNT                               10

ws2811_led_t dotcolors[] =
{
    0x00200000,  // red
    0x00201000,  // orange
    0x00202000,  // yellow
    0x00002000,  // green
    0x00002020,  // lightblue
    0x00000020,  // blue
    0x00100010,  // purple
    0x00200010,  // pink
};

ws2811_led_t dotcolors_rgbw[] =
{
    0x00200000,  // red
    0x10200000,  // red + W
    0x00002000,  // green
    0x10002000,  // green + W
    0x00000020,  // blue
    0x10000020,  // blue + W
    0x00101010,  // white
    0x10101010,  // white + W

};

ws2811_t ledstring
{
    0,
    ,
    TARGET_FREQ,
    DMA,
    {
        [0] =
        {
            .gpionum = GPIO_PIN,
            .count = LED_COUNT,
            .invert = 0,
            .brightness = 255,
            .strip_type = STRIP_TYPE,
        },
        [1] =
        {
            .gpionum = 0,
            .count = 0,
            .invert = 0,
            .brightness = 0,
        }
    }
    

};

using namespace std;
using namespace boost;
using namespace boost::filesystem;

int main()
{
    ws2811_init(&ledstring);

    int c = 0;

    for (int i = 0; i < LED_COUNT; i++)
    {
        if (ledstring.channel[0].strip_type == SK6812_STRIP_RGBW) {
            ledstring.channel[0].leds[i] = dotcolors_rgbw[c];
        }
        else {
            ledstring.channel[0].leds[i] = dotcolors[c];
        }
        
        c++;
        
        ws2811_render(&ledstring);

        if (c == sizeof(dotcolors))
        {
            c = 0;
        }

        usleep(1000000);
    }
    return 0;
}
