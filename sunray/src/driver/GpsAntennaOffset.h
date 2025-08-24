// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// GPS antenna offset correction
// Corrects GPS position based on antenna mounting position relative to robot center

#ifndef GPS_ANTENNA_OFFSET_H
#define GPS_ANTENNA_OFFSET_H

#include <Arduino.h>
#include "../../config.h"

class GpsAntennaOffset {
public:
    GpsAntennaOffset();
    
    // Set antenna offset relative to robot center (meters)
    void setOffset(float offsetX, float offsetY, float offsetZ);
    
    // Apply antenna offset correction to GPS position
    // Input: raw GPS position (North, East, Down) and robot heading
    // Output: corrected GPS position accounting for antenna offset
    void correctPosition(float &posN, float &posE, float &posD, float robotHeading);
    
    // Get current offset values
    void getOffset(float &offsetX, float &offsetY, float &offsetZ);
    
    // Check if offset correction is enabled
    bool isEnabled();
    
    // Enable/disable offset correction
    void setEnabled(bool enabled);
    
private:
    float antennaOffsetX;  // Forward offset from robot center (positive = forward)
    float antennaOffsetY;  // Sideways offset from robot center (positive = right)
    float antennaOffsetZ;  // Vertical offset from robot center (positive = up)
    bool offsetEnabled;
};

#endif