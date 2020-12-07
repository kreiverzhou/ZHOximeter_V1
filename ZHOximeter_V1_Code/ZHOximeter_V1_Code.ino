#include <LiquidCrystal.h>
#include <ezButton.h>
#include <millisDelay.h>

//Initializes "lcd" LiquidCrystal object with format
//lcd(rs,enable,d4,d5,d6,d7)
LiquidCrystal lcd(12, 11, 4, 5, 6, 7);

//Initialize LED pins
const int yellowLEDPin = 8;
const int greenLEDPin = 2;

//Initialize ezButton to Pin 13
ezButton button(13);
int buttonState = 0;

//Initialize delay for each 
millisDelay startDelay;
millisDelay welcomeDelay;
millisDelay dataCollectionDelay;
millisDelay showDataDelay;

//Initialize counter for storing measurements
int counter;

//Initialize sums for blood pressure and O2
float sumBPM,
      sumSaO2;

//Initialize String objects for BPM and SaO2 to disp in LCD
String stringBPM;
String stringSaO2;

//Include fn prototypes
void printStart();
void printProgress();

/***************************************************************/
/**** Everything bloe this block is original Maxim_Ox_2.ino ****/
/***************************************************************/

#include <float.h>
#include <max30102.h>
#include <CircularBuffer.h>
#include <KickSort.h>

#define E_HB_RED 6510   // Extinction coefficient of red channel for Hb
#define E_HB_IR 602     // Extinction coefficient of ir channel for Hb
#define E_HBO2_RED 942  // Extinction coefficient of red channel for HbO2
#define E_HBO2_IR 1204  // Extinction coefficient of ir channel for HbO2

/** Interrupt pin for the Maxim pulse oximeter sensor. */
const byte interruptPin = 10;

/** Size of the buffer for incoming red and IR data. */
const byte BUFFER_SIZE = 63;

/** Window size for data calculated during each heart beat event. */
const byte BEATS_WINDOW_SIZE = 10;

/** Buffer to store signal differentials for the IR channel. */
CircularBuffer<int, BUFFER_SIZE> irDiffs;

/** Buffer to store signal differentials for the red channel. */
CircularBuffer<int, BUFFER_SIZE> redDiffs;

/** Buffer storing most recent SaO2 calculations with each heart beat. */
CircularBuffer<float, BEATS_WINDOW_SIZE> SaO2s;

/** Buffer storing most recent heart rate calculations each heart beat. */
CircularBuffer<int, BEATS_WINDOW_SIZE> beatIntervals;

/** 
 * Most recent red and IR channel raw signals. These are populated from the
 * Maxim sensor's FIFO buffer during each read.
 */
uint32_t irDatum, redDatum = 0;

/** Red and IR channel raw signals captured during the previous read. */
uint32_t irLastDatum, redLastDatum = 0;

/** 
 *  Aggregated differentials between consecutive reads whose differentials
 *  have the same mathematical sign. This is used to track the total
 *  change in signal between inflections in the data.
 */
int aggIRDiff, aggRedDiff = 0;

/** Timestap for the most recent data point. */
unsigned long timestamp = 0;

/**
 * The timestamp associated with the last change in sign for the derivative
 * of the IR signal.
 */
unsigned long lastTimestamp = 0;

/**
 * Elapsed time in milliseconds since the last heart beat.
 */
unsigned int lastBeat = 0;

/** Flag indicating whether a heart beat event has been triggered. */
bool beatEvent = false;

/** 
 *  Intensity ratios for the red and IR channels, which are calculated
 *  independently in time based on when each signal reaches a local
 *  minimum after a heart beat event has been identified.
 */
float redIntensityRatio, irIntensityRatio = 0;

/** 
 * Flags indicating whether the intensity ratios for the red and IR 
 * channels have been calculated and captured. Once both have been captured,
 * the values are used to calculate the total signal ratio and SpO2.
 */
bool redBeatCaptured, irBeatCaptured = false;

/***************************************************************/
/***************************************************************/

void setup() {
  /***************************************************************/
  /**** Everything bloe this block is original Maxim_Ox_2.ino ****/
  /***************************************************************/
  
  // Reset the MAX30102
  maxim_max30102_reset();
  delay(1000);

  // Read/clear the interrupt status register
  uint8_t registerStatus = 0;
  maxim_max30102_read_reg(REG_INTR_STATUS_1, &registerStatus);  

  // Initialize the MAX30102
  maxim_max30102_init();

  // Initialize serial communication line
  Serial.begin(115200);
  delay(1000); 

  /***************************************************************/
  /***************************************************************/

  //Set Button debounce time to 100ms
  button.setDebounceTime(100);
  lcd.begin(16,2);
  printStart();

  pinMode(yellowLEDPin, OUTPUT);
  pinMode(greenLEDPin, OUTPUT);
}

void loop() {
  //Calls ezButton loop fn required for repeating timers
  button.loop();
  if(button.isPressed()){
        //Clear LCD and print
        printProgress();
        //Turns on LCD cursor blink to show user that something's happening
        lcd.blink(); 

        //Start collecting data and storing them as sums
        //so that the avg can be displayed to LCD
        counter = 0;
        sumBPM = 0;
        sumSaO2 = 0;

        digitalWrite(yellowLEDPin, HIGH); // turns on yellow LED
        
        dataCollectionDelay.start(15000); //start 15s timer for data collection

        while(dataCollectionDelay.remaining()){
            /***********************************************************/
            /*Maxim "void loop" code goes underneath*/
            /***********************************************************/

            // Wait for the Maxim sensor to signal that it's ready. This is signaled
            // whenever the interrupt pin goes from HIGH to LOW
            while(digitalRead(interruptPin) == HIGH);

            timestamp = millis();
            maxim_max30102_read_fifo(&redDatum, &irDatum);  

            int redDiff = redDatum - redLastDatum;
            int irDiff = irDatum - irLastDatum;

            redDiffs.push(redDiff);
            irDiffs.push(irDiff);

            // Check if signs of differences match the aggregate differences. If they
            // match, then continue to add to the aggregate. If they do not, this
            // represents an inflection in the data, and the aggregate values should
            // be reset. An inflection after a beat event has been flagged also signals
            // the end of the heart beat, in which case the relative change in signal
            // during the heart beat event is calculated and stored. This is determined
            // for the red and IR channels independently in time, as one can lag behind
            // the other.
            if (redDiff >= 0 != aggRedDiff >= 0) {
                if (beatEvent && !redBeatCaptured) {
                redIntensityRatio = (float) aggRedDiff / redLastDatum;
                redBeatCaptured = true;     
                }
                
                aggRedDiff = redDiff;
            } else {
                aggRedDiff += redDiff;
            }

            if (irDiff >= 0 != aggIRDiff >= 0) {
                if (beatEvent && !irBeatCaptured) {
                irIntensityRatio = (float) aggIRDiff / irLastDatum;
                irBeatCaptured = true;

                // Push the elapsed time between this beat and the last and then 
                // update the timestamp for the last beat. The IR channel was
                // chosen to track heart beats as the change in signal is greater
                // compared to the red channel.
                beatIntervals.push(lastTimestamp - lastBeat);
                lastBeat = lastTimestamp;
                }
                
                aggIRDiff = irDiff;
                lastTimestamp = timestamp;
            } else {
                aggIRDiff += irDiff;
            }

            /* Debug prints to serial
            Serial.print("red:");
            Serial.print(redDatum);
            Serial.print(" ir:");
            Serial.print(irDatum);
            Serial.print(" red_agg:");
            Serial.print(aggRedDiff);
            Serial.print(" ir_agg:");
            Serial.print(aggIRDiff);  
            Serial.println();
            */

            // Check to see if beat data for both the red and IR channels are available
            // and calculate SpO2, if so.
            if (redBeatCaptured && irBeatCaptured) {
                float intensityRatio = 2 * redIntensityRatio / irIntensityRatio;
                float SaO2 = (E_HB_RED - (E_HB_IR * intensityRatio)) / ((E_HB_RED - E_HBO2_RED) - (E_HB_IR - E_HBO2_IR) * intensityRatio);
                
                SaO2s.push(SaO2);

                float meanSaO2 = calculateMean(SaO2s);
                float meanHeartRate = 60.0 / (calculateMean(beatIntervals) / 1000);

                //Below goes into if (redBeatCaptured && irBeatCaptured)
                sumBPM += meanHeartRate;
                sumSaO2 += meanSaO2;
                counter++;

                Serial.println();
                Serial.print(" sa02:");
                Serial.print(meanSaO2);
                Serial.print(" hr:");
                Serial.print(meanHeartRate); 
                Serial.print(" counter:");
                Serial.print(counter);   

                beatEvent = false;
                redBeatCaptured = false;
                irBeatCaptured = false;
            }

            // Calculate the average and stdev for the current window of data once the
            // buffer is full. This is used to determine whether the most recent change
            // in signal is statistically significant to trigger a heart beat event.
            // The IR channel is chosen for this purpose as the signal has a larger  
            // range compared to the red channel.
            float mean = 0;
            float stdev = 0;

            if(irDiffs.available() == 0) {
                int dataLength = irDiffs.size();
                for(int index = 0;index < dataLength; index++) {
                mean += (float)(irDiffs[index]) / dataLength;
                }

                float variance = 0;
                for(int index = 0;index < dataLength; index++) {
                variance += pow(irDiffs[index] - mean, 2);
                }

                stdev = sqrt(variance / dataLength);
            }
            else if(irDiffs.size() == 1) {
                Serial.print("Collecting baseline data...");
            }
            else {
                Serial.print(".");
            }

            // Determine if last data point qualifies as a heart beat event.
            // A heart beat is identified whenever the change in signal is negative
            // and greater than two standard deviations away from the mean change
            // across the current window of data.
            float threshold = stdev * -2;
            if (stdev != 0 && irDiff < 0 && irDiff < threshold) {
                beatEvent = true;
            }  

            redLastDatum = redDatum;
            irLastDatum = irDatum; 
            
        }
        /***********************************************************/
        /***********************************************************/
        
        if (dataCollectionDelay.justFinished()){
            lcd.clear();
            lcd.noBlink();
            //Format LCD with data display
            lcd.setCursor(0,0);
            lcd.print("HR: ");
            lcd.setCursor(7,0);
            lcd.print("bpm");

            lcd.setCursor(0,1);
            lcd.print("SaO2: ");
            lcd.setCursor(9,1);
            lcd.print("%");

            stringBPM = String(sumBPM/counter, 0);
            stringSaO2 = String((sumSaO2/counter)*100,0);

            serial.print("Displayed HR: ");
            serial.print(stringBPM);
            serial.print("Displayed SpO2");
            serial.print("stringSaO2);
            
            showDataDelay.start(10000); //starts timer 
            while (showDataDelay.remaining()){
                digitalWrite(yellowLEDPin, LOW); //turns off yellow LED
                digitalWrite(greenLEDPin, HIGH); //turns on green LED

                lcd.setCursor(4,0);
                lcd.print(stringBPM);
                lcd.setCursor(7,1);
                lcd.print(stringSaO2);
            }

            //Once finished showing output, turn off LEDs and go back to Start message
            if (showDataDelay.justFinished()){
                digitalWrite(greenLEDPin, LOW); //turns off green LED
                printStart();
            }
        }
    }
  
}

/***************************************************************/
/**** Everything bloe this block is original Maxim_Ox_2.ino ****/
/***************************************************************/

/** 
 * Calculate the median of a dataset. 
 * 
 * @param data        A pointer to the beginning of the dataset array.
 * @param dataLength  The size of the data set.
 * 
 * @returns   The median of the data set.
 */
template<typename T>
float median(T* data, byte dataLength) {
  if (dataLength % 2 == 0) {
    return (data[dataLength / 2 - 1] + data[dataLength / 2]) / 2.0;
  }

  return data[dataLength / 2];
}

/**
 * Calculate the mean for the dataset using only values that do not
 * qualify as outliers. Outliers are considered values that fall
 * outside the Tukey fences (1.5 * IQR above and below the 3rd and 
 * 1st quartiles, respectively).
 * 
 * @param data    The dataset to analyze as CircularBuffer.
 * 
 * @returns   The mean value of the dataset, excluding outliers.
 */
template<typename T, size_t LENGTH>
float calculateMean(CircularBuffer<T, LENGTH> &data) {
  T *sortedData = (T*) malloc(sizeof(data[0] * LENGTH));
  for (int index = 0; index < LENGTH; index++) {
    sortedData[index] = data[index];
  }

  KickSort<T>::quickSort(sortedData, LENGTH, KickSort_Dir::ASCENDING);
  float Q1 = median(sortedData, LENGTH / 2);
  float Q3 = median(sortedData + (int) ceil(LENGTH / 2.0), LENGTH / 2);
  float IQR = Q3 - Q1;
  
  float lowerLimit = Q1 - (1.5 * IQR);
  float upperLimit = Q3 + (1.5 * IQR);

  int validPoints = 0;
  float sum = 0;
  for (int index = 0; index < LENGTH; index++) {
    if (data[index] >= lowerLimit && data[index] <= upperLimit) {
      sum += data[index];
      validPoints++;
    }
  }
  
  free(sortedData);

  return sum / validPoints;
}

/**
 * Algorithm:
 *  1. Keep track of differences back 2.5s and last absolute value
 *  2. Wait until buffer is full
 *  3. Analyze buffer for stdev
 *  4. If latest values triggers threshold, set flag
 *  5. Continue to read differences until it becomes positive
 *      a. Use last absolute value as baseline
 *      b. Move backwards in buffer and add all negative consecutive values
 *      c. Calculate delta/baseline for channel
 *  6. Check if delta/baseline for both channels are set, and if so:
 *      a. Calculate SpO2 and add to SpO2 window
 *      b. Reset both to 0
 *
 */

/***************************************************************/
/***************************************************************/

void printStart(){
    lcd.clear();

    lcd.setCursor(2,0);
    lcd.print("Press handle");

    lcd.setCursor(0,1);
    lcd.print("button to start!");
}

void printProgress(){
    lcd.clear();

    lcd.setCursor(1,0);
    lcd.print("In progress...");
    
    lcd.setCursor(4,1);
    lcd.print("Wait 15s");
}
