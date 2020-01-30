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
//// GTEC TAG CORE ANCHOR 0 - POXYZ
////////////////////////////////////////////////

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <avr/wdt.h>

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

uint16_t tagsIds[] = {0x6E5B};
uint16_t ownAnchorId = 0x6757;
uint16_t anchorIds[] = {0x6752, 0x6710, 0x6732, 0x6708, 0x6047};
int maxResets = 10;
bool logDebug = false;

////////////////////////////////////////////////

uint8_t seq = 0;
int numTags, numAnchors;
int resetCount = 0;
int currentIndexAnchor = 0;
int currentIndexTag = 0;
UWB_settings_t tagUWBSettings;
void(* resetFunc) (void) = 0;

void setup()
{
    Serial.begin(115200);

    numTags = sizeof(tagsIds) / sizeof(tagsIds[0]);
    numAnchors = sizeof(anchorIds) / sizeof(anchorIds[0]);


    if (logDebug)
    {
        Serial.println("Starting...");
    }

    if(Pozyx.begin() == POZYX_FAILURE)
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

    Serial.print("Version: ");
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

    currentIndexAnchor = -1;
    currentIndexTag = 0;
    seq = -1;
}

void loop()
{

    device_range_t range;
    int status;

    if (logDebug)
    {
        Serial.print("Seq:");
        Serial.println(seq);
    }

    currentIndexAnchor += 1;
    if (currentIndexAnchor >= numAnchors)
    {
        seq += 1;
        if (seq > 255)
        {
            seq = 0;
        }
        currentIndexAnchor = 0;
        currentIndexTag += 1 ;
        if (currentIndexTag >= numTags)
        {
            currentIndexTag = 0;
        }
    }

    if (logDebug)
    {
        Serial.print("REMOTE RANGING WITH:");
        Serial.println(anchorIds[currentIndexAnchor], HEX);
    }

    status = Pozyx.doRemoteRanging(tagsIds[currentIndexTag], anchorIds[currentIndexAnchor], &range);

    if (status == POZYX_SUCCESS)
    {
        printRangeWithSettings(range, tagsIds[currentIndexTag], anchorIds[currentIndexAnchor], seq, tagUWBSettings);
        resetCount = 0;
    }
    else
    {
        if (logDebug)
        {
            Serial.print("REMOTE RANGIN ERROR\r\n");
        }
        resetCount += 1;
    }

    if (resetCount >= maxResets)
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

void printRangeWithSettings(device_range_t range, uint16_t tagId, uint16_t anchorId, int seq, UWB_settings_t tagUWBSettings)
{
    if (logDebug)
    {
        Serial.print(" r");
        printHex(range.distance, 8);
        Serial.println("");
    }

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

    Serial.write(0xBB); //Tail
    Serial.write(0xBB); //Tail

}


