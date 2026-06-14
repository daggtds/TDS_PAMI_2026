#pragma once
//  ----------------------------------------------------------------------------------------------
#include <Arduino.h>
#include "config.h"
#include <FastLED.h>
#define N_LEDS  12
CRGB    leds[N_LEDS];
//  ----------------------------------------------------------------------------------------------
class LED
{
    public:
        enum LED_MODE
        {
            LED_BLINK,
            LED_CONSTANT
        };

        LED()
        {
            FastLED.addLeds<NEOPIXEL, PIN_LED_DATA>(leds, N_LEDS);
            setConfig(LED_CONSTANT,CRGB::Black);
        };

        void setConfig(LED_MODE mode, CRGB color, uint32_t tempo=0)
        {
            led_mode    =   mode;
            led_color   =   color;
            led_tempo   =   tempo;
            led_timer   =   0;
            led_state   =   true;
        };

        void update()
        {
            if(led_mode == LED_BLINK)
            {
                if(led_timer > led_tempo)
                {
                    led_state = !led_state;
                    if(led_state)
                    {
                        FastLED.showColor(led_color);
                    }
                    else
                    {
                        FastLED.showColor(CRGB::Black);
                    }
                    led_timer = 0;
                }
            }
            else
            {
                FastLED.showColor(led_color);  
            }
        };

    private:
        bool            led_state = false;
        elapsedMillis   led_timer = 0;
        uint32_t        led_tempo = 0;
        CRGB            led_color = CRGB::Black;
        LED_MODE        led_mode  = LED_CONSTANT;
};