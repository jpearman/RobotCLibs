/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                 2012-2015                                   */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     gyroLib.c                                                    */
/*    Author:     James Pearman                                                */
/*    Created:    2 Oct 2012                                                   */
/*                                                                             */
/*    Revisions:  V0.1                                                         */
/*                                                                             */
/*                V0.2 17 Feb 2013                                             */
/*                     Clear absolute angle and local parameters when          */
/*                     starting task.                                          */
/*                                                                             */
/*                V0.3 3 May 2015                                              */
/*                     Cleanup, doxygen comments, update license               */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    The author is supplying this software for use with the VEX IQ            */
/*    control system. This file can be freely distributed and teams are        */
/*    authorized to freely use this program , however, it is requested that    */
/*    improvements or additions be shared with the Vex community via the vex   */
/*    forum.  Please acknowledge the work of the authors when appropriate.     */
/*    Thanks.                                                                  */
/*                                                                             */
/*    Licensed under the Apache License, Version 2.0 (the "License");          */
/*    you may not use this file except in compliance with the License.         */
/*    You may obtain a copy of the License at                                  */
/*                                                                             */
/*      http://www.apache.org/licenses/LICENSE-2.0                             */
/*                                                                             */
/*    Unless required by applicable law or agreed to in writing, software      */
/*    distributed under the License is distributed on an "AS IS" BASIS,        */
/*    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. */
/*    See the License for the specific language governing permissions and      */
/*    limitations under the License.                                           */
/*                                                                             */
/*    The author can be contacted on the vex forums as jpearman                */
/*    or electronic mail using jbpearman_at_mac_dot_com                        */
/*    Mentor for team 8888 RoboLancers, Pasadena CA.                           */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

// Stop recursive includes
#ifndef __GYROLIB__
#define __GYROLIB__

/*-----------------------------------------------------------------------------*/
/** @file    gyroLib2.h
  * @brief   VEX gyro wrapper functions for ROBOTC
*//*---------------------------------------------------------------------------*/

// Structure to hold global info for the gyro
typedef struct _gyroData {
    tSensors    port;           ///< analog port the gyro is connected to
    bool        valid;          ///< indicates gyro is initialized
    float       angle;          ///< angle in range 0 to 360 deg
    float       abs_angle;      ///< absolute angle, both positive and negative
    int         drift_error;    ///< accumulated error due to drift
    } gyroData;

// local storage for the gyro calculations
static  gyroData    theGyro;

#define GYRO_DRIFT_THRESHOLD    3

/*-----------------------------------------------------------------------------*/
/** @brief display current gyro angle on the LCD for debug pruposes            */
/*-----------------------------------------------------------------------------*/

void
GyroDebug( int displayLine )
{
    string str;

    if( theGyro.valid )
        {
        // display current value
        sprintf(str,"Gyro %5.1f   ", theGyro.angle );
        displayLCDString(displayLine, 0, str);
        }
    else
        displayLCDString(displayLine, 0, "Init Gyro.." );
}

/*-----------------------------------------------------------------------------*/
/** @brief  Task that polls the Gyro and calculates the angle of rotation      */
/*-----------------------------------------------------------------------------*/

task GyroTask()
{
    int     gyro_value;
    int     lastDriftGyro = 0;

    float   angle;
    float   old_angle = 0.0;
    float   delta_angle = 0.0;

    long    nSysTimeOffset;

    // Gyro readings invalid
    theGyro.valid = false;

    // clear absolute
    theGyro.abs_angle = 0;

    // clear drift error
    theGyro.drift_error = 0;

    // Cause the gyro to reinitialize
    SensorType[theGyro.port] = sensorNone;

    // Wait 1/2 sec
    wait1Msec(500);

    // Gyro should be motionless here
    SensorType[theGyro.port] = sensorGyro;

    // Wait 1/2 sec
    wait1Msec(500);

    // Save the current system timer
    nSysTimeOffset = nSysTime;

    // loop forever
    while(true)
        {
        // get current gyro value (deg * 10)
        gyro_value = SensorValue[theGyro.port];

        // Filter drift when not moving
        // check this every 250mS
        if( (nSysTime - nSysTimeOffset) > 250 )
            {
            if( abs( gyro_value - lastDriftGyro ) < GYRO_DRIFT_THRESHOLD )
                theGyro.drift_error += (lastDriftGyro - gyro_value);

            lastDriftGyro = gyro_value;

            nSysTimeOffset = nSysTime;
            }

        // Create float angle, remove offset
        angle = (gyro_value + theGyro.drift_error)  / 10.0;

        // normalize into the range 0 - 360
        if( angle < 0 )
            angle += 360;

        // store in struct for others
        theGyro.angle = angle;

        // work out change from last time
        delta_angle = angle - old_angle;
        old_angle   = angle;

        // fix rollover
        if(delta_angle > 180)
          delta_angle -= 360;
        if(delta_angle < -180)
          delta_angle += 360;

        // store absolute angle
        theGyro.abs_angle = theGyro.abs_angle + delta_angle;

        // We can use the angle
        theGyro.valid = true;

        // Delay
        wait1Msec( 20 );
        }
}

/*-----------------------------------------------------------------------------*/
/** @brief    Initialize the Gyro                                              */
/** param[in] port the analog port that the gyro is connected to               */
/*-----------------------------------------------------------------------------*/
void
GyroInit( tSensors port = in1  )
{
    theGyro.port  = port;
    theGyro.valid = false;
    theGyro.angle = 0.0;
    theGyro.abs_angle = 0.0;

    startTask( GyroTask );
}

/*-----------------------------------------------------------------------------*/
/** @brief Reinitialize the gyro task                                          */
/*  Cause the gyro to be reinitialized by stopping and then restarting the     */
/*  polling task                                                               */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/** @details
 *   Cause the gyro to be reinitialized by stopping and then restarting the
 *   polling task
 */
void
GyroReinit()
{
    stopTask( GyroTask );
    startTask( GyroTask );
}

/*-----------------------------------------------------------------------------*/
/** @brief    Get the current gyro angle in degrees                            */
/** @returns  gyro angle in the range 0 to 360 deg                             */
/*-----------------------------------------------------------------------------*/
float
GyroAngleDegGet()
{
    return( theGyro.angle );
}

/*-----------------------------------------------------------------------------*/
/** @brief    Get the current gyro angle in radians                            */
/** @returns  gyro angle in the range 0 to 2PI radians                         */
/*-----------------------------------------------------------------------------*/
float
GyroAngleRadGet()
{
    return( theGyro.angle / 180.0 * PI );
}

/*-----------------------------------------------------------------------------*/
/** @brief    Get the current absolute gyro angle                              */
/** @returns  the accumuated absolute gyro angle in degrees                    */
/*-----------------------------------------------------------------------------*/
float
GyroAngleAbsGet()
{
    return( theGyro.abs_angle );
}

/*-----------------------------------------------------------------------------*/
/** @brief    Get the validity of the gyro                                     */
/** @returns  true if the gyro is initialized and returning valid data         */
/*-----------------------------------------------------------------------------*/
bool
GyroValidGet()
{
    return( theGyro.valid );
}

/*-----------------------------------------------------------------------------*/
/** @brief    ROBOTC gyro warning elination                                    */
/*-----------------------------------------------------------------------------*/
/** @details
 * The ROBOTC warnings about unused functions drive me crazy, so we call
 * everything here, including this function, to remove them
 * do not use or call this function from your code !
 */

void
GyroWarningEliminate()
{
    GyroDebug(0);
    GyroReinit();
    GyroAngleDegGet();
    GyroAngleRadGet();
    GyroAngleAbsGet();
    GyroValidGet();
    GyroWarningEliminate();
}

#endif  //__GYROLIB__
