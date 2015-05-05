/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2015                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     gyroSim.c                                                   */
/*    Author:     James Pearman                                                */
/*    Created:    5 May 2015                                                   */
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
// Simulation of the internal ROBOTC gyro calculations

typedef  long      int32_t;
typedef  short     int16_t;

// final value in deg * 10
static int32_t     GyroValue = 0;

// filter out noise.
const int GyroJitterRange = 4;

// the gyro analog port
static  tSensors   gyroAnalogPin = in1;

/*-----------------------------------------------------------------------------*/
/** @brief task that calculates the gyro value in the same way as ROBOTC       */
/*-----------------------------------------------------------------------------*/

task gyroSim()
{
    int16_t     i;
    int32_t     GyroBiasAcc = 0;
    int32_t     GyroBias;
    int32_t     GyroSmallBias;
    int32_t     GyroRaw;
    int32_t     GyroRawFiltered = 0;
    int32_t     GyroValueTmp;

    int16_t     GyroDelta;
    int32_t     GyroSensorScale = 130;
    int32_t     GyroFullScale   = 3600;
    int32_t     GyroJitterCycles = 0;

    // Delay for 200 milliseconds. This is to allow the gyro to stabilize when it is first powered up.
    // The datasheet indicates that this may take 50 milliseconds so we'll run it a little longer.
    wait1Msec(200);

    // calculate bias
    for(i=0;i<1024;i++)
        {
        GyroBiasAcc = GyroBiasAcc + SensorValue[ gyroAnalogPin ];
        wait1Msec(1);
        }

    GyroBias      = GyroBiasAcc / 1024;
    GyroSmallBias = GyroBiasAcc - (GyroBias * 1024);
    // Ok bias done

    // Run forever
    while(true)
        {
        // Get raw analog value
        GyroRaw   = SensorValue[ gyroAnalogPin ];
        // remove bias
        GyroDelta = GyroRaw - GyroBias;

        // ignore small changes
        if ((GyroDelta < -GyroJitterRange) || (GyroDelta > +GyroJitterRange))
            {
            // integrate angle
            GyroRawFiltered += GyroDelta;

            // compensate for error in bias
            if ((++GyroJitterCycles % 1024) == 0)
                GyroRawFiltered -= GyroSmallBias;
            }

        // calculate angle in deg * 10
        GyroValueTmp = GyroRawFiltered / GyroSensorScale;

        // Clip to +/- full scale
        if( abs(GyroValueTmp) >= GyroFullScale )
            GyroValueTmp = GyroValueTmp - ((GyroValueTmp / GyroFullScale) * GyroFullScale);

        // and store for the user
        GyroValue = GyroValueTmp;

        // sleep
        wait1Msec(1);
        }
}

/*-----------------------------------------------------------------------------*/
/** @brief Initialize the gyro                                                 */
/** @param[in] port the analog input the gyro is connected to                  */
/*-----------------------------------------------------------------------------*/
void
initGyro( tSensors port = in1 )
{
    gyroAnalogPin = port;
    GyroValue     = 0;

    startTask( gyroSim );
}

// Test code
task main()
{
    char  str[32];

    bLCDBacklight = true;

    initGyro( in1 );

    while(1)
        {
        // display current value
        sprintf(str,"Gyro %5.1f   ", GyroValue );
        displayLCDString(1, 0, str);

        wait1Msec(20);
        }
}
