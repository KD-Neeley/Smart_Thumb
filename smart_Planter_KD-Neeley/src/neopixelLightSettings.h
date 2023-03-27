/*
 * Project: Header file for Smart Thumb
 * Description: Control the Neopixel light settings by choosing from a variety of preset light arrays 
 * Author: Katie Neeley
 * Date: 03/26/2023
 */
#include "kdsRainbows.h"
#include <neopixel.h>
#include <math.h>

int _pixelCount=16;  
int _pixelPin;

Adafruit_NeoPixel pixel2(_pixelCount, _pixelPin, WS2812B);

class smartPixelPresets {

enum smartPixelState{
    PRIMARIES,
    SECONDARIES,
    FULLCOLORS,
    TRADITIONAL,
    FULLRAINBOW,
    RAINBOW, 
    TRADSECONDARIES,
    TRADPRIMARIES,
    TRADRAINBOW
};

smartPixelState _currentState = PRIMARIES;
smartPixelState _targetState = TRADRAINBOW;

public:

    void goToNext() {
        switch (_currentState) {
            case PRIMARIES:
                primariesF();
                _currentState = PRIMARIES;
                _targetState = SECONDARIES;
                break;
             case SECONDARIES:
                secondariesF();
                _currentState = SECONDARIES;
                _targetState = FULLCOLORS;
                break;
            case FULLCOLORS:
                fullColorsF();
                _currentState = FULLCOLORS;
                _targetState = TRADITIONAL;
                break;
            case TRADITIONAL:
                traditionalF();
                _currentState = TRADITIONAL;
                _targetState = FULLRAINBOW;
                break;
            case FULLRAINBOW:
                fullRainbowF();
                _currentState = FULLRAINBOW;
                _targetState = RAINBOW;
                break;
            case RAINBOW:
                rainbowF();
                _currentState = RAINBOW;
                _targetState = TRADSECONDARIES;
                break;
            case TRADSECONDARIES:
                traditionalSecondariesF();
                _currentState = TRADSECONDARIES;
                _targetState = TRADPRIMARIES;
                break;
            case TRADPRIMARIES:
                traditionalPrimariesF();
                _currentState = TRADPRIMARIES;
                _targetState = TRADRAINBOW;
                break;
            case TRADRAINBOW:
                traditionalRainbowF();
                _currentState = TRADRAINBOW;
                _targetState = PRIMARIES;
        }
    }
    void primariesF() {
        int r=random(2);
        for(int i=0; i<_pixelCount; i++) { 
            for(int cycle=0; cycle<(_pixelCount+1);cycle++ ){
            pixel2.setPixelColor(i+cycle, primaries[r]);
            pixel2.show();
            }
        }
    }
    void secondariesF() {
        int r=random(2);
        for(int i=0; i<_pixelCount; i++) { 
            for(int cycle=0; cycle<(_pixelCount+1);cycle++ ){
            pixel2.setPixelColor(i+cycle, secondaries[r]);
            pixel2.show();
            }
        }
    }
    void fullColorsF() {
        int r=random(5);
        for(int i=0; i<_pixelCount; i++) { 
            for(int cycle=0; cycle<(_pixelCount+1);cycle++ ){
            pixel2.setPixelColor(i+cycle, fullcolors[r]);
            pixel2.show();
            }
        }
    }
    void traditionalF() {
        int r=random(5);
        for(int i=0; i<_pixelCount; i++) { 
            for(int cycle=0; cycle<(_pixelCount+1);cycle++ ){
            pixel2.setPixelColor(i+cycle, traditional[r]);
            pixel2.show();
            }
        }
    }
    void fullRainbowF() {
      int r=random(5);
        for(int i=0; i<_pixelCount; i++) { 
            for(int cycle=0; cycle<(_pixelCount+1);cycle++ ){
            pixel2.setPixelColor(i+cycle, fullrainbow[r]);
            pixel2.show();
            }
        }
    }
    void rainbowF() {
        int r=random(8);
        for(int i=0; i<_pixelCount; i++) { 
            for(int cycle=0; cycle<(_pixelCount+1);cycle++ ){
            pixel2.setPixelColor(i+cycle, rainbow[r]);
            pixel2.show();
            }
        }
    }
    void traditionalSecondariesF() {
        int r=random(2);
        for(int i=0; i<_pixelCount; i++) { 
            for(int cycle=0; cycle<(_pixelCount+1);cycle++ ){
            pixel2.setPixelColor(i+cycle, traditionalsecondaries[r]);
            pixel2.show();
            }
        }
    }
    void traditionalPrimariesF() {
        int r=random(2);
        for(int i=0; i<_pixelCount; i++) { 
            for(int cycle=0; cycle<(_pixelCount+1);cycle++ ){
            pixel2.setPixelColor(i+cycle, traditionalprimaries[r]);
            pixel2.show();
            }
        }
    }
    void traditionalRainbowF() {
        int r=random(8);
        for(int i=0; i<_pixelCount; i++) { 
            for(int cycle=0; cycle<(_pixelCount+1);cycle++ ){
            pixel2.setPixelColor(i+cycle, traditionalrainbow[r]);
            pixel2.show();
            }
        }
    }


};