/*
MIT License

Copyright (c) 2020 Group of Electronic Technology and Communications. University of A Coruna.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


////////////////////////////////////////////////
//// POZYX - Local ranging and CIR
////////////////////////////////////////////////

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <avr/wdt.h>

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

uint16_t otherDevice = 0x6E5B;
uint16_t ownDevice = 0x6757;
bool resetOnFail = false;
int maxResets = 100;
bool logDebug = false;


////////////////////////////////////////////////

uint8_t seq = 0;
int resetCount = 0;
UWB_settings_t tagUWBSettings;
void(* resetFunc) (void) = 0;

void setup()
{
    Serial.begin(115200);

    if (logDebug)
    {
        Serial.println("Starting...");
    }

    if (Pozyx.begin() == POZYX_FAILURE)
    {
        Serial.println("ERROR: Unable to connect to POZYX shield");
        Serial.println("Reset required");
        delay(500);
        abort();
    }

    uint8_t fwVersion;
    Pozyx.getFirmwareVersion(&fwVersion);

    uint8_t major = (fwVersion >> 4)  & 255;
    uint8_t minor = fwVersion  & 15;

    Serial.print("Own Version: ");
    Serial.print(major);
    Serial.print(".");
    Serial.println(minor);


    Pozyx.getFirmwareVersion(&fwVersion, otherDevice);
    major = (fwVersion >> 4)  & 255;
    minor = fwVersion  & 15;

    Serial.print("Other Version: ");
    Serial.print(major);
    Serial.print(".");
    Serial.println(minor);


    if (Pozyx.getUWBSettings(&tagUWBSettings) == POZYX_FAILURE)
    {
        Serial.println("ERROR: Unable to get UWB settings");
        Serial.println("Reset required");
        delay(500);
        abort();
    }

    delay(100);
    if (logDebug)
    {
        Serial.println("START Core Ranging:");
    }

    seq = -1;
}

void loop()
{


    // UWB ///////////////////////////////////////////////////
    device_range_t range;
    int status;
    int currentCirLength=0;

    if (logDebug)
    {
        Serial.print("Seq:");
        Serial.println(seq);
    }

    seq += 1;
    if (seq > 255)
    {
        seq = 0;
    }

    if (logDebug)
    {
        Serial.print("RANGING WITH:");
        Serial.println(otherDevice, HEX);
    }

    status = Pozyx.doRanging(otherDevice, &range);

    if (status == POZYX_SUCCESS)
    {
        //NOW WE GET THE LAST CIR MEASUREMENTS
        if (logDebug)
        {
            Serial.print("RANGING Success\r\n");
        }

        if (tagUWBSettings.prf==1){
            currentCirLength = 996;
        } else if (tagUWBSettings.prf==2){
            currentCirLength = 1016;
        } 

        if (logDebug)
        {
            Serial.print("Current CIR length: ");
            Serial.println(currentCirLength);
        }
        

        printRangeWithSettingsAndCir(range, ownDevice, otherDevice, seq, tagUWBSettings, currentCirLength);
        resetCount = 0;
    }
    else
    {
        if (resetOnFail) {
            if (logDebug)
            {
                Serial.print("RANGING ERROR\r\n");
            }
            resetCount += 1;
        }
    }

    if (resetCount >= maxResets && resetOnFail)
    {
        if (logDebug)
        {
            Serial.print("Reset\r\n");
        }
        resetCount = 0;
        Pozyx.resetSystem();
    }
    delay(2);
}


void printHex(int num, int precision)
{
    char tmp[16];
    char format[128];
    sprintf(format, "%%.%dX", precision);
    sprintf(tmp, format, num);
    Serial.print(tmp);
}


void printFloat(float32_t value) {
    byte buf[4]; //Quaternion X
    byte *b = (byte*) &value;
    memcpy(buf, b, sizeof(buf));
    Serial.write(buf, sizeof(buf));
}

void printRangeWithSettingsAndCir(device_range_t range, uint16_t tagId, uint16_t anchorId, int seq, UWB_settings_t tagUWBSettings, int currentCirLength)
{
    if (logDebug)
    {
        Serial.print("Range: ");
        Serial.print(range.distance);
        Serial.println("");

        printCir(currentCirLength);
    } else {
        Serial.write(0xFA); //Header
        Serial.write(0xFA); //Header

        byte buf_tagId[2]; //ID origin
        buf_tagId[0] = tagId & 255;
        buf_tagId[1] = (tagId >> 8)  & 255;
        Serial.write(buf_tagId, sizeof(buf_tagId));

        Serial.write(0x00); //Type origin 0 anchor, 1 tag

        byte buf_anchorId[2]; //ID destination
        buf_anchorId[0] = anchorId & 255;
        buf_anchorId[1] = (anchorId >> 8)  & 255;
        Serial.write(buf_anchorId, sizeof(buf_anchorId));


        Serial.write(0x01); //Type destination

        byte buf_distance[4]; //Distance
        buf_distance[0] = range.distance & 255;
        buf_distance[1] = (range.distance >> 8)  & 255;
        buf_distance[2] = (range.distance >> 16) & 255;
        buf_distance[3] = (range.distance >> 24) & 255;
        Serial.write(buf_distance, sizeof(buf_distance));

        byte buf_timestamp[4]; //timestamp
        buf_timestamp[0] = range.timestamp & 255;
        buf_timestamp[1] = (range.timestamp >> 8)  & 255;
        buf_timestamp[2] = (range.timestamp >> 16) & 255;
        buf_timestamp[3] = (range.timestamp >> 24) & 255;
        Serial.write(buf_timestamp, sizeof(buf_timestamp));

        Serial.write((byte) seq & 255); //seq

        byte buf_rss[2]; //rss
        buf_rss[0] = range.RSS & 255;
        buf_rss[1] = (range.RSS >> 8)  & 255;
        Serial.write(buf_rss, sizeof(buf_rss));

        Serial.write((byte) tagUWBSettings.channel & 255); //Channel
        Serial.write((byte) tagUWBSettings.bitrate & 255); //bitrate
        Serial.write((byte) tagUWBSettings.prf & 255); //prf

        //Print CIR
        printCir(currentCirLength);

        Serial.write(0xBB); //Tail
        Serial.write(0xBB); //Tail  
    }
}

void printCir(int currentCirLength) {

    uint8_t params[3];
    byte buf[2];

    params[0] = 0;
    params[1] = 0;
    params[2] = 10;

    uint8_t measurementsPerChunk = 6;
    int index = 0;

    uint8_t result[24];

    params[2] = measurementsPerChunk;

    int baseIndex = 0;
    while (index < currentCirLength) {

        if (currentCirLength-index<measurementsPerChunk){
            //Last chunk
            measurementsPerChunk = currentCirLength-index;
            params[2] = measurementsPerChunk;
        }
        params[0] = index & 255;
        params[1] = (index >> 8)  & 255;

        int success = Pozyx.regFunction(POZYX_CIR_DATA, params, 3, result, 24);

        if (success == 1) {
            if (logDebug)
            {
                Serial.print("Success chunk ");
                Serial.print(index);
                Serial.print(" ");
                Serial.println(index+measurementsPerChunk);
            }
            int indexMes = 0;
            for (int i = 0; i < measurementsPerChunk; i++) {

                int real = int((uint8_t)(result[indexMes + 1]) << 8 | (uint8_t)(result[indexMes]));
                int cx = int((uint8_t)(result[indexMes + 3]) << 8 | (uint8_t)(result[indexMes + 2]));
                indexMes = indexMes + 4;

                if (!logDebug){
                    buf[0] = real & 255;
                    buf[1] = (real >> 8)  & 255;
                    Serial.write(buf, sizeof(buf));

                    buf[0] = cx & 255;
                    buf[1] = (cx >> 8)  & 255;
                    Serial.write(buf, sizeof(buf));  
                }


                baseIndex +=4;

                //   if (logDebug)
                // {
                //      Serial.print(real);
                //      Serial.print(",");
                //      Serial.println(cx);
                // }

            }
        }

        index = index + measurementsPerChunk;
    }

    if (logDebug){
        Serial.print("CIR Measurements sent: ");
        Serial.println(baseIndex);
    }
}



