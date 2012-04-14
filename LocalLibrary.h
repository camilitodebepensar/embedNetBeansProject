//
//  LocalLibrary.h
//  Library header
//  ----------------------------------
//  Developed with embedXcode
//
//  Project embedExample
//  Created by Rei VILO on 14/04/12
//  Copyright (c) 2012 http://sites.google.com/site/vilorei
//

// Core library
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega2560__) // Arduino specific
#include "Arduino.h"	
#elif defined(__PIC32MX__) // chipKIT specific 
#include "WProgram.h"
#elif defined(__AVR_ATmega644P__) // Wiring specific
#include "Wiring.h"
#elif defined(__MSP430G2452__) || defined(__MSP430G2553__) || defined(__MSP430G2231__) // LaunchPad specific
#include "Energia.h"
#endif

#ifndef embedExample_LocalLibrary_h
#define embedExample_LocalLibrary_h



#endif
