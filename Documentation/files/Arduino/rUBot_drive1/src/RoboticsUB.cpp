/******************************************************************************
RoboticsUB.cpp - MPU Arduino library.
Copyright (C) 2020  Albert Álvarez-Carulla
Repository: https://github.com/Albert-Alvarez/roboticsub-imu

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
******************************************************************************/

#include "RoboticsUB.h"

IMU::IMU() {}

void IMU::Install(void) {

    if (begin() != INV_SUCCESS) {
        while (1) {
            delay(5000);
        }
    }

    dmpBegin(
        DMP_FEATURE_6X_LP_QUAT |    // Enable 6-axis quat
        DMP_FEATURE_GYRO_CAL,       // Use gyro calibration
        10                          // Set DMP FIFO rate to 10 Hz
    );     

    enableInterrupt();

}

void IMU::ReadSensor(void) {

    if (fifoAvailable()) {
        dmpUpdateFifo();
    }

}

float * IMU::GetQuaternion(void) {

    static float q[4];

    q[0] = calcQuat(qw);
    q[1] = calcQuat(qx);
    q[2] = calcQuat(qy);
    q[3] = calcQuat(qz);
    
    return q;
}

float * IMU::GetRPW(void) {

    static float rpw[3];

    computeEulerAngles(true);

    rpw[0] = roll;
    rpw[1] = pitch;
    rpw[2] = yaw;
    
    return rpw;
}