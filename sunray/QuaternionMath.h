/*
  Sunray Robot
  Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
  Licensed GPLv3 for open source use
  or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)
*/

#ifndef QUATERNION_MATH_H
#define QUATERNION_MATH_H

#include <Arduino.h>

/**
 * Zentrale Utility-Klasse für Quaternion-Berechnungen
 * Vereinheitlicht die verschiedenen Quaternion-Operationen im IMU-Modul
 */
class QuaternionMath {
public:
    /**
     * Konvertiert Quaternion zu Euler-Winkeln (Roll, Pitch, Yaw)
     * @param w Quaternion w-Komponente
     * @param x Quaternion x-Komponente  
     * @param y Quaternion y-Komponente
     * @param z Quaternion z-Komponente
     * @param roll Ausgabe: Roll-Winkel in Radiant
     * @param pitch Ausgabe: Pitch-Winkel in Radiant
     * @param yaw Ausgabe: Yaw-Winkel in Radiant
     */
    static void toEulerAngles(float w, float x, float y, float z, float& roll, float& pitch, float& yaw);
    
    /**
     * Berechnet die Magnitude (Länge) eines Quaternions
     * @param w Quaternion w-Komponente
     * @param x Quaternion x-Komponente
     * @param y Quaternion y-Komponente
     * @param z Quaternion z-Komponente
     * @return Magnitude des Quaternions
     */
    static float magnitude(float w, float x, float y, float z);
    
    /**
     * Normalisiert ein Quaternion
     * @param w Quaternion w-Komponente (wird modifiziert)
     * @param x Quaternion x-Komponente (wird modifiziert)
     * @param y Quaternion y-Komponente (wird modifiziert)
     * @param z Quaternion z-Komponente (wird modifiziert)
     */
    static void normalize(float& w, float& x, float& y, float& z);
    
    /**
     * Berechnet das Konjugat eines Quaternions
     * @param w Quaternion w-Komponente
     * @param x Quaternion x-Komponente
     * @param y Quaternion y-Komponente
     * @param z Quaternion z-Komponente
     * @param conjW Ausgabe: Konjugat w-Komponente
     * @param conjX Ausgabe: Konjugat x-Komponente
     * @param conjY Ausgabe: Konjugat y-Komponente
     * @param conjZ Ausgabe: Konjugat z-Komponente
     */
    static void conjugate(float w, float x, float y, float z, float& conjW, float& conjX, float& conjY, float& conjZ);
    
    /**
     * Multipliziert zwei Quaternions
     * @param w1 Erstes Quaternion w-Komponente
     * @param x1 Erstes Quaternion x-Komponente
     * @param y1 Erstes Quaternion y-Komponente
     * @param z1 Erstes Quaternion z-Komponente
     * @param w2 Zweites Quaternion w-Komponente
     * @param x2 Zweites Quaternion x-Komponente
     * @param y2 Zweites Quaternion y-Komponente
     * @param z2 Zweites Quaternion z-Komponente
     * @param resultW Ausgabe: Ergebnis w-Komponente
     * @param resultX Ausgabe: Ergebnis x-Komponente
     * @param resultY Ausgabe: Ergebnis y-Komponente
     * @param resultZ Ausgabe: Ergebnis z-Komponente
     */
    static void multiply(float w1, float x1, float y1, float z1, 
                        float w2, float x2, float y2, float z2,
                        float& resultW, float& resultX, float& resultY, float& resultZ);
    
    /**
     * Rotiert einen 3D-Vektor mit einem Quaternion
     * @param qw Quaternion w-Komponente
     * @param qx Quaternion x-Komponente
     * @param qy Quaternion y-Komponente
     * @param qz Quaternion z-Komponente
     * @param vx Vektor x-Komponente (wird modifiziert)
     * @param vy Vektor y-Komponente (wird modifiziert)
     * @param vz Vektor z-Komponente (wird modifiziert)
     */
    static void rotateVector(float qw, float qx, float qy, float qz, float& vx, float& vy, float& vz);
};

#endif // QUATERNION_MATH_H