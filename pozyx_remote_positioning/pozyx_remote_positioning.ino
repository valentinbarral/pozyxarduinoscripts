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
//// Pozyx - Remote Ranging and IMU
////////////////////////////////////////////////

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <avr/wdt.h>

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

uint16_t tagsIds[] = {0x6a39};
uint16_t ownAnchorId = 0x607d;
uint16_t anchorIds[] = {0x6f2f, 0x6e74, 0x6040, 0x6e5b, 0x6030};
int32_t anchors_x[] = {0, 1000, 4069, 3993, 3700};    // anchor x-coorindates in mm
int32_t anchors_y[] = {0, 4487, 4487, 2988, 842};        // anchor y-coordinates in mm
int32_t heights[] = {1172, 206, 1203, 1045, 375}; // anchor z-coordinates in mm

bool resetOnFail = true;
int maxResets = 10;
bool logDebug = false;

uint8_t algorithm = POZYX_POS_ALG_TRACKING;
uint8_t dimension = POZYX_3D;
int32_t tagHeight = 1000;

////////////////////////////////////////////////

uint8_t seq = 0;
int numTags, numAnchors;
int resetCount = 0;
int currentIndexAnchor = 0;
int currentIndexTag = 0;
UWB_settings_t tagUWBSettings;
void (*resetFunc)(void) = 0;

void setup()
{

    Serial.begin(115200);

    numTags = sizeof(tagsIds) / sizeof(tagsIds[0]);
    numAnchors = sizeof(anchorIds) / sizeof(anchorIds[0]);

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

    uint8_t major = (fwVersion >> 4) & 255;
    uint8_t minor = fwVersion & 15;

    Serial.print("Version: ");
    Serial.print(major);
    Serial.print(".");
    Serial.println(minor);

    Pozyx.getFirmwareVersion(&fwVersion, tagsIds[0]);
    major = (fwVersion >> 4) & 255;
    minor = fwVersion & 15;

    Serial.print("Tag Version: ");
    Serial.print(major);
    Serial.print(".");
    Serial.println(minor);
    Serial.println(minor);

    if (Pozyx.getUWBSettings(&tagUWBSettings) == POZYX_FAILURE)
    {
        Serial.println("ERROR: Unable to get UWB settings");
        Serial.println("Reset required");
        delay(500);
        abort();
    }

    delay(100);

    if (logDebug){
        Serial.println("Clearing devices: ");
    }
    Pozyx.clearDevices();

    if (logDebug){
        Serial.println("Devices cleared.");
    }
  
    setAnchorsManual();
    //printCalibrationResult();
    delay(2000);

    if (logDebug){
        Serial.println(F("Starting positioning: "));
    }
    
    currentIndexTag = 0;
    seq = -1;
}

void setAnchorsManual()
{
    if (logDebug){
        Serial.println(F("Setting anchors: "));
    }
    for (int i = 0; i < numAnchors; i++)
    {
        device_coordinates_t anchor;
        anchor.network_id = anchorIds[i];
        anchor.flag = 0x1;
        anchor.pos.x = anchors_x[i];
        anchor.pos.y = anchors_y[i];
        anchor.pos.z = heights[i];
        Pozyx.addDevice(anchor, tagsIds[0]);
    }

    if (numAnchors > 4)
    {
        Pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, numAnchors);
    }

    if (logDebug){
        Serial.println(F("End setting anchors."));
    }
}

void loop()
{

    coordinates_t position;
    int status = Pozyx.doRemotePositioning(tagsIds[0], &position, dimension, tagHeight, algorithm);

    seq += 1;

    if (seq > 255)
    {
        seq = 0;
    }

    if (status == POZYX_SUCCESS)
    {
        printCoordinates(position, tagsIds[currentIndexTag], seq);
        resetCount = 0;
    }
    else
    {
        if (resetOnFail)
        {
            if (logDebug)
            {
                Serial.print("REMOTE RANGING ERROR\r\n");
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

    delay(20);
}

void printHex(int num, int precision)
{
    char tmp[16];
    char format[128];
    sprintf(format, "%%.%dX", precision);
    sprintf(tmp, format, num);
    Serial.print(tmp);
}

void printFloat(float32_t value)
{
    byte *b = (byte *)&value;
    Serial.write(b[0]);
    Serial.write(b[1]);
    Serial.write(b[2]);
    Serial.write(b[3]);
}

void printCoordinates(coordinates_t coor, uint16_t tagId, int seq)
{

    // get the network id and print it
    uint16_t network_id;
    Pozyx.getNetworkId(&network_id);

    if (logDebug)
    {
        Serial.print("POS,0x");
        //Serial.print(network_id, HEX);
        Serial.print(tagId, HEX);
        Serial.print(",");
        Serial.print(coor.x);
        Serial.print(",");
        Serial.print(coor.y);
        Serial.print(",");
        Serial.print(coor.z);
        Serial.print(",");
    }
    else
    {

        Serial.write(0xFA); //Header
        Serial.write(0xFA); //Header

        byte buf_tagId[2]; //TAG ID
        buf_tagId[0] = tagId & 255;
        buf_tagId[1] = (tagId >> 8) & 255;
        Serial.write(buf_tagId, sizeof(buf_tagId));

        byte buf_coor_x[4]; //coor.x
        buf_coor_x[0] = coor.x & 255;
        buf_coor_x[1] = (coor.x >> 8) & 255;
        buf_coor_x[2] = (coor.x >> 16) & 255;
        buf_coor_x[3] = (coor.x >> 24) & 255;
        Serial.write(buf_coor_x, sizeof(buf_coor_x));

        byte buf_coor_y[4]; //coor.y
        buf_coor_y[0] = coor.y & 255;
        buf_coor_y[1] = (coor.y >> 8) & 255;
        buf_coor_y[2] = (coor.y >> 16) & 255;
        buf_coor_y[3] = (coor.y >> 24) & 255;
        Serial.write(buf_coor_y, sizeof(buf_coor_y));

        byte buf_coor_z[4]; //coor.z
        buf_coor_z[0] = coor.z & 255;
        buf_coor_z[1] = (coor.z >> 8) & 255;
        buf_coor_z[2] = (coor.z >> 16) & 255;
        buf_coor_z[3] = (coor.z >> 24) & 255;
        Serial.write(buf_coor_z, sizeof(buf_coor_z));
    }

    // get information about the positioning error and print it
    pos_error_t pos_error;
    Pozyx.getPositionError(&pos_error);

    if (logDebug)
    {
        Serial.print(pos_error.x);
        Serial.print(",");
        Serial.print(pos_error.y);
        Serial.print(",");
        Serial.print(pos_error.z);
        Serial.print(",");
        Serial.print(pos_error.xy);
        Serial.print(",");
        Serial.print(pos_error.xz);
        Serial.print(",");
        Serial.println(pos_error.yz);
    }
    else
    {
        byte buf_var_x[2]; //var_x
        buf_var_x[0] = pos_error.x & 255;
        buf_var_x[1] = (pos_error.x >> 8) & 255;
        Serial.write(buf_var_x, sizeof(buf_var_x));

        byte buf_var_y[2]; //var_y
        buf_var_y[0] = pos_error.y & 255;
        buf_var_y[1] = (pos_error.y >> 8) & 255;
        Serial.write(buf_var_y, sizeof(buf_var_y));

        byte buf_var_z[2]; //var_z
        buf_var_z[0] = pos_error.z & 255;
        buf_var_z[1] = (pos_error.z >> 8) & 255;
        Serial.write(buf_var_z, sizeof(buf_var_z));

        byte buf_cov_xy[2]; //cov_xy
        buf_cov_xy[0] = pos_error.xy & 255;
        buf_cov_xy[1] = (pos_error.xy >> 8) & 255;
        Serial.write(buf_cov_xy, sizeof(buf_cov_xy));

        byte buf_cov_xz[2]; //cov_xz
        buf_cov_xy[0] = pos_error.xz & 255;
        buf_cov_xy[1] = (pos_error.xz >> 8) & 255;
        Serial.write(buf_cov_xy, sizeof(buf_cov_xz));

        byte buf_cov_yz[2]; //cov_yz
        buf_cov_yz[0] = pos_error.yz & 255;
        buf_cov_yz[1] = (pos_error.yz >> 8) & 255;
        Serial.write(buf_cov_yz, sizeof(buf_cov_yz));

        Serial.write(0xBB); //Tail
        Serial.write(0xBB); //Tail
    }

    // read out the ranges to each anchor and print it
    // if (logDebug)
    // {
    //     for (int i = 0; i < numAnchors; i++)
    //     {
    //         device_range_t range;
    //         Pozyx.getDeviceRangeInfo(anchorIds[i], &range);
    //         Serial.print(",");
    //         Serial.print(range.distance);
    //         Serial.print(",");
    //         Serial.print(range.RSS);
    //     }
    // }
    // Serial.println();
}