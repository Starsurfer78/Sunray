// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// GPS antenna offset correction implementation

#include "GpsAntennaOffset.h"
#include "../../helper.h"
#include <math.h>

GpsAntennaOffset::GpsAntennaOffset() {
    antennaOffsetX = 0.0;
    antennaOffsetY = 0.0;
    antennaOffsetZ = 0.0;
    offsetEnabled = false;
}

void GpsAntennaOffset::setOffset(float offsetX, float offsetY, float offsetZ) {
    antennaOffsetX = offsetX;
    antennaOffsetY = offsetY;
    antennaOffsetZ = offsetZ;
}

void GpsAntennaOffset::correctPosition(float &posN, float &posE, float &posD, float robotHeading) {
    if (!offsetEnabled) {
        return; // No correction if disabled
    }
    
    // Transform antenna offset from robot frame to world frame
    // Robot frame: X=forward, Y=right, Z=up
    // World frame: N=north, E=east, D=down
    
    // Calculate offset in world coordinates using robot heading
    float offsetN = antennaOffsetX * cos(robotHeading) - antennaOffsetY * sin(robotHeading);
    float offsetE = antennaOffsetX * sin(robotHeading) + antennaOffsetY * cos(robotHeading);
    float offsetD = -antennaOffsetZ; // GPS uses down as positive, robot uses up as positive
    
    // Apply correction: subtract antenna offset to get robot center position
    posN -= offsetN;
    posE -= offsetE;
    posD -= offsetD;
}

void GpsAntennaOffset::getOffset(float &offsetX, float &offsetY, float &offsetZ) {
    offsetX = antennaOffsetX;
    offsetY = antennaOffsetY;
    offsetZ = antennaOffsetZ;
}

bool GpsAntennaOffset::isEnabled() {
    return offsetEnabled;
}

void GpsAntennaOffset::setEnabled(bool enabled) {
    offsetEnabled = enabled;
}