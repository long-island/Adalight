// Arduino "bridge" code between host computer and WS2801-based digital
// RGB LED pixels (e.g. Adafruit product ID #322).  Intended for use
// with USB-native boards such as Teensy or Adafruit 32u4 Breakout;
// works on normal serial Arduinos, but throughput is severely limited.
// LED data is streamed, not buffered, making this suitable for larger
// installations (e.g. video wall, etc.) than could otherwise be held
// in the Arduino's limited RAM.

// Some effort is put into avoiding buffer underruns (where the output
// side becomes starved of data).  The WS2801 latch protocol, being
// delay-based, could be inadvertently triggered if the USB bus or CPU
// is swamped with other tasks.  This code buffers incoming serial data
// and introduces intentional pauses if there's a threat of the buffer
// draining prematurely.  The cost of this complexity is somewhat
// reduced throughput, the gain is that most visual glitches are
// avoided (though ultimately a function of the load on the USB bus and
// host CPU, and out of our control).

// LED data and clock lines are connected to the Arduino's SPI output.
// On traditional Arduino boards, SPI data out is digital pin 11 and
// clock is digital pin 13.  On both Teensy and the 32u4 Breakout,
// data out is pin B2, clock is B1.  LEDs should be externally
// powered -- trying to run any more than just a few off the Arduino's
// 5V line is generally a Bad Idea.  LED ground should also be
// connected to Arduino ground.

// --------------------------------------------------------------------
//   This file is part of Adalight.

//   Adalight is free software: you can redistribute it and/or modify
//   it under the terms of the GNU Lesser General Public License as
//   published by the Free Software Foundation, either version 3 of
//   the License, or (at your option) any later version.

//   Adalight is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//   GNU Lesser General Public License for more details.

//   You should have received a copy of the GNU Lesser General Public
//   License along with Adalight.  If not, see
//   <http://www.gnu.org/licenses/>.
// --------------------------------------------------------------------

#include <SPI.h>
#include "Adafruit_WS2801.h"


Adafruit_WS2801 strip = Adafruit_WS2801(20, WS2801_GRB);// WS2801_GRB is the GRB order required by the 36mm pixels.

static const uint8_t magic[] = {'A','d','a'};
static const unsigned long serialTimeout = 15000; // 15 seconds
	
#define MAGICSIZE  sizeof(magic)
#define HEADERSIZE (MAGICSIZE + 3)
#define MODE_HEADER 0
#define MODE_HOLD   1
#define MODE_DATA   2
#define spi_out(n) (void)SPI.transfer(n)

void colorWipe(uint32_t c, uint8_t wait);

//static const unsigned long serialTimeout = 15000;

int LED_PIN = 13;
void setup() {
	
	pinMode(LED_PIN, OUTPUT);
  // put your setup code here, to run once:
  strip.begin();
  strip.show(); // Update LED contents, to start they are all 'off'
  
  uint8_t buffer[256], indexIn=0, indexOut=0, mode=MODE_HEADER, hi, lo, chk, i, spiFlag;
  int16_t bytesBuffered = 0, hold=0, c;
  int32_t bytesRemaining;
  unsigned long startTime, lastByteTime, lastAckTime, t;
  
 //colorWipe(Color(255, 0, 0), 50);
 //colorWipe(Color(0, 255, 0), 50);
 //colorWipe(Color(0, 0, 255), 50);
 //rainbow(50);
 //rainbowCycle(1);
  colorWipe(Color(0, 255, 0),10); //red
  
  //colorWipe(Color(0, 0, 255),10); //blue
  Serial.begin(115200);
  Serial.print("Ada\n"); // Send ACK string to host
  colorWipe(Color(255, 0, 0),10); //green
  startTime    = micros();
  lastByteTime = lastAckTime = millis();
  
  for(;;) {
	  t = millis();
	  if((bytesBuffered < 256) && ((c = Serial.read()) >= 0)) {
		  buffer[indexIn++] = c;
		  bytesBuffered++;
		  lastByteTime = lastAckTime = t;
		  colorWipe(Color(0, 0, 255), 10);//blue//remove
	  } else {
		  //colorWipe(Color(0, 0, 255), 10);//blue
		  if((t - lastAckTime) > 1000) {
			  Serial.print("Ada\n");
			  lastAckTime = t;
		  }
		  if((t - lastByteTime) > serialTimeout) {
			  for(c=0; c<32767; c++) {
				  colorWipe(Color(0, 0, 0),0);
			  }
			  delay(1); // One millisecond pause = latch
			  lastByteTime = t; // Reset counter
		  }
	    }
		
		switch(mode) {
			case MODE_HEADER:
			if(bytesBuffered >= HEADERSIZE) {
				for(i=0; (i<MAGICSIZE) && (buffer[indexOut++] == magic[i++]););
				if(i == MAGICSIZE) {
					hi  = buffer[indexOut++];
					lo  = buffer[indexOut++];
					chk = buffer[indexOut++];
					if(chk == (hi ^ lo ^ 0x55)) {
						bytesRemaining = 3L * (256L * (long)hi + (long)lo + 1L);
						bytesBuffered -= 3;
						spiFlag        = 0;         // No data out yet
						mode           = MODE_HOLD;
				    } else {
						// Checksum didn't match; search resumes after magic word.
						indexOut  -= 3; // Rewind
				      }
		        }
				bytesBuffered -= i;
			}
			break;
			
			case MODE_HOLD:
			// Ostensibly "waiting for the latch from the prior frame
			// to complete" mode, but may also revert to this mode when
			// underrun prevention necessitates a delay.
			if((micros() - startTime) < hold) break; // Still holding; keep buffering
			// Latch/delay complete.  Advance to data-issuing mode...
			//LED_PORT &= ~LED_PIN;  // LED off
			 digitalWrite(LED_PIN, LOW);
			mode=MODE_DATA; // ...and fall through (no break):
			
			case MODE_DATA:
			//while(spiFlag && !(SPSR & _BV(SPIF))); // Wait for prior byte
			if(bytesRemaining > 0) {
				if(bytesBuffered > 0) {
					spi_out((byte)buffer[indexOut++]);
					bytesBuffered--;
					bytesRemaining--;
					spiFlag = 1;
				}
				if((bytesBuffered < 32) && (bytesRemaining > bytesBuffered)) {
					startTime = micros();
					hold      = 100 + (32 - bytesBuffered) * 10;
					mode      = MODE_HOLD;
				}
				
			} else {
				startTime  = micros();
				hold       = 1000;        // Latch duration = 1000 uS
		//		LED_PORT  |= LED_PIN;     // LED on
				digitalWrite(LED_PIN, HIGH);
				mode       = MODE_HEADER;				
			  }
	    }// end switch
  }// end for(;;)
}//  end setup()

void loop() {
  // put your main code here, to run repeatedly:

}


void rainbow(uint8_t wait) {
	int i, j;
	
	for (j=0; j < 256; j++) {     // 3 cycles of all 256 colors in the wheel
		for (i=0; i < strip.numPixels(); i++) {
			strip.setPixelColor(i, Wheel( (i + j) % 255));
		}
		strip.show();   // write all the pixels out
		delay(wait);
	}
}

// Slightly different, this one makes the rainbow wheel equally distributed
// along the chain
void rainbowCycle(uint8_t wait) {
	int i, j;
	
	for (j=0; j < 256 * 5; j++) {     // 5 cycles of all 25 colors in the wheel
		for (i=0; i < strip.numPixels(); i++) {
			// tricky math! we use each pixel as a fraction of the full 96-color wheel
			// (thats the i / strip.numPixels() part)
			// Then add in j which makes the colors go around per pixel
			// the % 96 is to make the wheel cycle around
			strip.setPixelColor(i, Wheel( ((i * 256 / strip.numPixels()) + j) % 256) );
		}
		strip.show();   // write all the pixels out
		delay(wait);
	}
}

// fill the dots one after the other with said color
// good for testing purposes
void colorWipe(uint32_t c, uint8_t wait) {
	int i;
	
	for (i=0; i < strip.numPixels(); i++) {
		strip.setPixelColor(i, c);
		strip.show();
		delay(wait);
	}
}

/* Helper functions */

// Create a 24 bit color value from R,G,B
uint32_t Color(byte r, byte g, byte b)
{
	uint32_t c;
	c = r;
	c <<= 8;
	c |= g;
	c <<= 8;
	c |= b;
	return c;
}

//Input a value 0 to 255 to get a color value.
//The colours are a transition r - g -b - back to r
uint32_t Wheel(byte WheelPos)
{
	if (WheelPos < 85) {
		return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
		} else if (WheelPos < 170) {
		WheelPos -= 85;
		return Color(255 - WheelPos * 3, 0, WheelPos * 3);
		} else { 
		WheelPos -= 170;
		return Color(0, WheelPos * 3, 255 - WheelPos * 3);
	}
}
