#include <Adafruit_NeoPixel.h>

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN    4

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 12

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)


static int rise = 2;
static int fall = -1;
static int pulseInterval = 100;


void setup() {
  // put your setup code here, to run once:
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.fill();
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(20); // Set BRIGHTNESS to about 1/5 (max = 255)
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  for(int x=0;x<strip.numPixels();x++){
    pulseWave(x, random(0,3), random(0,3), random(0,3), 254, 1);
    delay(500);
  }
  //pulseWave(random(0,strip.numPixels()-1), random(0,3), random(0,3), random(0,3), 255, 10);
  //pulseWave(6, random(0,3), random(0,3), random(0,3), 254, 10);
  delay(5000);
}


void pulseWave(int startPixel, int redRise, int greenRise, int blueRise, int colorMax, int wait) {
  //set up variables
  colorMax = constrain(colorMax,5,254);
  startPixel = constrain(startPixel,0,strip.numPixels()-1);
  int pixelMatrix[strip.numPixels()] ;
  int pixelMatrixDirection[strip.numPixels()] ;
    for(int i=0;i<strip.numPixels();i++){ //clear variables after initialization
      pixelMatrix[i] = 0;
      pixelMatrixDirection[i]=0;
    }
  pixelMatrixDirection[startPixel] = rise;
  strip.setPixelColor(startPixel,redRise,greenRise,blueRise);
  strip.show();
  int stopPixel = (startPixel + strip.numPixels()/2)%strip.numPixels();//find opposite side
  int stripAdvance = 0;
  int stripAdvanceOld = 0;
  

  while((pixelMatrix[stopPixel]>0 || pixelMatrixDirection[stopPixel]>-1)) {            //this loop actually does the pulse
    
      for(int a=0;a<strip.numPixels();a++){           //change the pixel brightness based on direction
          pixelMatrix[a] = pixelMatrix[a]+pixelMatrixDirection[a];
          if((pixelMatrixDirection[a]>0)&&pixelMatrix[a]>colorMax){   //check if it's time to advance
            pixelMatrixDirection[a] = fall;
            stripAdvance++;
          }
        pixelMatrix[a]=constrain(pixelMatrix[a],0,255);
        strip.setPixelColor(a, constrain(redRise*pixelMatrix[a],0,255), constrain(greenRise*pixelMatrix[a],0,255), constrain(blueRise*pixelMatrix[a],0,255));
      }

      
      if(stripAdvance > stripAdvanceOld) {                  //advance the wave
        //Serial.print("Advance: ");
        for(int u=0;u<strip.numPixels();u++){
          if(pixelMatrixDirection[u]==0) {

             if(pixelMatrixDirection[(u+1)%(strip.numPixels())]<0){
              pixelMatrixDirection[u]=rise;
             }
             if(pixelMatrixDirection[(u-1+strip.numPixels())%(strip.numPixels())]<0 && u!=stopPixel){
                pixelMatrixDirection[u]=rise;
             }

          }
          //Serial.print(pixelMatrixDirection[u]);
          //Serial.print(", ");
        }
        //Serial.println();
        stripAdvanceOld = stripAdvance;
      }
      
      strip.show();
      delay(wait);
    }
  
  //clear everything when done
  strip.fill();
  strip.show();
}

    // Returns the Red component of a 32-bit color
    uint8_t Red(uint32_t color)
    {
        return (color >> 16) & 0xFF;
    }

    // Returns the Green component of a 32-bit color
    uint8_t Green(uint32_t color)
    {
        return (color >> 8) & 0xFF;
    }

    // Returns the Blue component of a 32-bit color
    uint8_t Blue(uint32_t color)
    {
        return color & 0xFF;
    }
