#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>

#include <serial/serial.h>
#include "sabertooth_2x25_driver.h"

#define pi 3.14159265


// Aspects of desired circular path
float radius = 1.5, Period = 20;
float thetadot = 2*pi/Period;

// Suscriber/Publisher
ros::Subscriber vicon_sub;
ros::Publisher ref_pub;
ros::Publisher viconFeedback_pub;

// Set initial position/orientation
double x = 1.5, y = 0, theta = 0;
// Variables for speeds
double dXc,dYc,dthetaC,wheel1,wheel2,wheel3, wheel1speed, wheel2speed, wheel3speed;

// Set radius of wheel and distance from CoM to wheel in meters
float rWheel = 0.04875;    // radius of wheel, m
float D = 0.175;         // distance from CoM to wheel, m

// geometry_msgs
geometry_msgs::Pose2D reftraj;
geometry_msgs::PoseStamped viconFeedback;

// Time
ros::Time timeNow, startTime;
ros::Duration loopTime = ros::Duration(1.75*Period);

// Prepare serial
serial::Serial *ser;




/***********************************************************************************************/
/* SABERTOOTH FUNCTIONS */
/***********************************************************************************************/

// Write data to serial port
void writeToPort(char data){
    bool written=false;
    char* write = &data;
    while(!written){
        try{
            ser->write(write);
            written=true;
        }
        catch(std::exception& e){
            continue;
        }
    }
}


/**********************************************************************************************
 * Function:        static void send_command ( uint8_t command, uint8_t value, uint8_t address )
 *
 * Pre-Condition:   None
 * Input:           Receives the command data from the driver functions
 * Output:          Sends the three commands plus their checksum to the serial port, and through
 *                      that to the Sabertooth Motor Controller
 * Side Effects:    None
 * Overview:        None
 * Notes:           Static helper function, for use only by driver functions in this file
 *********************************************************************************************/

    /* Helper Command, for internal driver use only
     * Defining it here, in the .c file, instead of in the .h file, to prevent
     * compiler warning about it being declared "static", but never defined
     * This is correct, since it is not for the user, only the user's functions */

    

/*********************************************************************************************/
static void send_command ( uint8_t command, uint8_t value, uint8_t address ) {
    //assert  ( command < COMMAND_HIGH_LIMIT);
    // putchar ( address );
    // putchar ( command );
    // putchar ( value );
    // putchar ( ( address + command + value ) & CRC_MASK );

    writeToPort( (char) address );
    writeToPort( (char) command );
    writeToPort( (char) value );
    writeToPort( (char) (( address + command + value ) & CRC_MASK) );
}



/**********************************************************************************************
 * Function:        uint8_t control_motors_sep ( uint8_t command1, uint8_t speed1, \
 *                                               uint8_t command2, uint8_t speed2, \
 *                                               uint8_t address )
 *
 * Pre-Condition:   None
 * Input:           Receives command data from the application program
 * Output:          Sends the commands to the send_command function,
 *                      and then to the serial port
 * Side Effects:    None
 * Overview:        Checks commands for validity, and passes them to the serial port
 * Notes:           This function is valid for Sabertooth Commands 0, 1, 4 - 7
 *                  These commands are for controlling two motors with individual settings, a single motor at a time
 *
 *                      Individual Motor Commands:
 *                          0:  Drive Forward      Motor 1
 *                          1:  Drive Reverse      Motor 1
 *                          4:  Drive Forward      Motor 2
 *                          5:  Drive Reverse      Motor 2
 *                          6:  Drive 7-Bit        Motor 1
 *                          7:  Drive 7-Bit        Motor 2
 *
 *********************************************************************************************/

uint8_t control_motors_sep ( uint8_t command1, uint8_t speed1, \
                             uint8_t command2, uint8_t speed2, \
                             uint8_t address ) {
// = MOTOR_DRIVER_ADDRESS_1 ) { // If your compiler allows overloading, feel free to un-comment the assignment
// and move it immediately after the "address" variable
    if ( ( command1 < COMMAND_LOW_LIMIT || command1 > DRIVE_MOTOR_2_7_BIT ) || \
         ( command2 < COMMAND_LOW_LIMIT || command2 > DRIVE_MOTOR_2_7_BIT ) ) {

   /*     Set error code for invalid command
    *     Call a user error function to do whatever your application requires, such as:
    *         mSTOP_ALL_MOTORS;  */
        return FALSE;
    }

    else {
      
        send_command ( command1, speed1, address );
        send_command ( command2, speed2, address );
//      Set error code for no error
        return TRUE;
    }
}

uint8_t set_baudrate ( uint8_t desired_baudrate, uint8_t address ) {

    static uint8_t new_baudrate = DEFAULT_BAUDRATE;

    if ( desired_baudrate < BAUDRATE_2400 || \
         desired_baudrate > BAUDRATE_38400 ) {

            new_baudrate = DEFAULT_BAUDRATE;
            return FALSE;                                   // Set error code for error
    }

    else {
        new_baudrate = desired_baudrate;
    }

    send_command ( SET_BAUD_RATE, new_baudrate, address );
    return TRUE;                                            // Set error code for no error
}




/***********************************************************************************************/
/* START TARS FUNCTIONS */
/***********************************************************************************************/


double Xt(double time)
{
    float temp; 
    temp = radius*cos(time*thetadot);

    if (time > 1.75*Period)
        temp = 1.5;
    
    return temp;
}

double Yt(double time)
{
    float temp;
    temp = radius*sin(time*thetadot);

    if (time > 1.75*Period)
        temp = 0;

    
    return temp;
}

double Thetat(double time)
{
    float temp;
    temp = thetadot*time;

    if (time > 1.75*Period)
        temp = 0;

    return temp;
}

double dXt(double time)
{
    float temp;
    temp = -radius*thetadot*sin(time*thetadot);

    if (time > 1.75*Period)
        temp = 0;


    return temp;
}

double dYt(double time)
{
    float temp;
    temp = radius*thetadot*cos(time*thetadot);

    if (time > 1.75*Period)
        temp = 0;

    return temp;
}

double dThetat(double time)
{
    float temp;
    temp = thetadot;

    if (time > 1.75*Period)
        temp = 0;
    
    return temp;
}



void viconCallback(geometry_msgs::TransformStamped rover)
{
    x = rover.transform.translation.x;
    y = rover.transform.translation.y;
   
    tf::Quaternion q(rover.transform.rotation.x,rover.transform.rotation.y,rover.transform.rotation.z,rover.transform.rotation.w);

    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    
    theta = yaw;
    theta += pi;

    viconFeedback.header = rover.header;

    viconFeedback.pose.position.x = x;
    viconFeedback.pose.position.y = y;
    viconFeedback.pose.position.z = rover.transform.translation.z;

    viconFeedback.pose.orientation = rover.transform.rotation;

    viconFeedback_pub.publish(viconFeedback);
}


// Function to calculate motor controller wheel speed inputs given input wheel velocity from kinematics model
int speedCalc(float wheel_velocity, int wheelNumber)
{
    float omega;
    float RPM;
    float speed;
    int maxRPM = 85;

    // Calculate angular rate of wheel in rad/s
    omega = wheel_velocity/rWheel;
    // Calculate RPM from angular rate
    RPM = 60*omega/(2*pi);

    // Since wheel3 is input as Motor 1 for Motor Controller 1, speed output must be reversed to account for reversed
    // direction of the motor
    if (wheelNumber == 3)
        RPM = -RPM;

    // Calculate speed based on RPM and motor capability (max RPM of 104)
    if ((RPM > -maxRPM) && (RPM < 0))
    {
        speed = 63*(RPM+maxRPM)/maxRPM + 1;
    }
    if ((RPM > 0) && (RPM < maxRPM))
    {
        speed = 63*RPM/maxRPM + 64;
    }
    if (RPM == 0)
    {
        speed = 64;
    }
    if (RPM >= maxRPM)
    {
        speed = 127;
    }
    if (RPM <= -maxRPM)
    {
        speed = 1;
    }

    // Saturate Wheel Speeds
    if ((speed > 49) && (speed < 61)) { speed = 49; }
    if ((speed < 79) && (speed > 67)) { speed = 79; }


    return speed;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_turtle");
    ros::NodeHandle nh;
    
    int freq = 100;
    ros::Rate loop_rate(freq);

    startTime = ros::Time::now();
    
    vicon_sub = nh.subscribe("/vicon/tars", 100, viconCallback);

    ref_pub = nh.advertise<geometry_msgs::Pose2D>("/ref_traj",100);
    viconFeedback_pub = nh.advertise<geometry_msgs::PoseStamped>("/viconFeedback",10);


    double ref_time;
    double ref_theta;
 
    double J11,J12,J13,J21,J22,J23,J31,J32,J33;

    // Initialize PID Control error variables
    double X_err, Y_err, Theta_err;
    double Pxerr=0.0,Pyerr=0.0,Ptheta_err=0.0;
    double Iyerr=0.0,Ixerr=0.0,Itheta_err=0.0;
    double Dxerr=0.0,Dyerr=0.0,Dtheta_err=0.0;
    double OldErrX=0.0,OldErrY=0.0,OldErrTheta=0.0;

    // Set parameters defining wheel location on body
    double alpha1, alpha2, alpha3, nu;
    // alpha: angle relative to local frame
    // nu:    angle relative to world frame x and y axes 
    alpha1 = 0;
    alpha2 = 2*pi / 3;
    alpha3 = 4*pi / 3;
    nu = 0;


    ser = new serial::Serial();
    ser->setPort("/dev/ttyS0");
    ser->open();
    ser->setBaudrate(9600);

    set_baudrate(2,128);
    set_baudrate(2,129);


    while(ros::ok()) {

        timeNow = ros::Time::now();

        // Calculate reference time for trajectory calculation
        ref_time = timeNow.toSec() - startTime.toSec();

        // Define reference trajectory
        reftraj.x = Xt(ref_time);
        reftraj.y = Yt(ref_time);

        ref_theta = Thetat(ref_time);
        if (ref_theta > 2*pi)
        {
            ref_theta = ref_theta - (floor(ref_theta/(2*pi))*2*pi);
        }
        reftraj.theta = ref_theta;


        //PID controller
        X_err = Xt(ref_time) - x;
        Y_err = Yt(ref_time) - y;
        Theta_err = ref_theta - theta;
        if (Theta_err > pi)
        {
            Theta_err-=2*pi;
            //Theta_err = -Theta_err;
        }
        if (Theta_err < -pi)
        {
            Theta_err+=2*pi;
            //Theta_err = -Theta_err;

        }

        // Proportional control:
        Pxerr = (X_err);
        Pyerr = (Y_err);
        Ptheta_err = (Theta_err);
        if (Pxerr>0.5) {
            Pxerr = 0.5;
        }
        if (Pxerr<-0.5) {
            Pxerr = -0.5;
        }
        if (Pyerr>0.5) {
            Pyerr = 0.5;
        }
        if (Pyerr<-0.5) {
            Pyerr = -0.5;
        }
        if (Ptheta_err > 0.5) {
            Ptheta_err = 0.5;
        }
        if (Ptheta_err < -0.5) {
            Ptheta_err = -0.5;
        }

        // Integral control:
        Ixerr += (X_err)/freq;
        Iyerr += (Y_err)/freq;
        Itheta_err += (Theta_err)/freq;
        if (Ixerr>0.5) {
            Ixerr = 0.5;
        }
        else if (Ixerr<-0.5) { 
            Ixerr = -0.5; 
        }
        if (Iyerr>0.5) {
            Iyerr = 0.5;
        }
        else if (Iyerr<-0.5) {
            Iyerr = -0.5;
        }
        if (Itheta_err>0.5) {
            Itheta_err = 0.5;
        }
        else if (Itheta_err<-0.5) {
            Itheta_err = -0.5;
        }

        // Derivative control:
        Dxerr = (X_err) - OldErrX;
        Dxerr = (Y_err) - OldErrY;
        Dtheta_err = (Theta_err) - OldErrTheta;

        OldErrX = X_err;
        OldErrY = Y_err;
        OldErrTheta = Theta_err;

        // Set P, I, D values
        double Kp, Ki, Kd, Kp_theta, Ki_theta, Kd_theta;
        Kp = 0.75;
        Ki = 0.25;
        Kd = 0;

        Kp_theta = 1.75;
        Ki_theta = 1;
        Kd_theta = 0;

        //Calculate robot speed with PID feedback
        dXc = dXt(ref_time) + Kp*Pxerr + Ki*Ixerr + Kd*Dxerr;
        dYc = dYt(ref_time) + Kp*Pyerr + Ki*Iyerr + Kd*Dyerr;
        dthetaC = dThetat(ref_time) + Kp_theta*Ptheta_err + Ki_theta*Itheta_err + Kd_theta*Dtheta_err;

        // Update rotation of body frame such that wheel speeds calculated appropriately throughout circular path
        // Must be in body frame, thus must use actual theta instead of reference
        nu = theta;

        //Make the Jinv matrix
        J11 = -sin(nu);
        J12 = cos(nu);
        J13 = D;
        J21 = -sin(nu+alpha2);
        J22 = cos(nu+alpha2);
        J23 = D;
        J31 = -sin(nu+alpha3);
        J32 = cos(nu+alpha3);
        J33 = D;

        wheel1 = J11*dXc + J12*dYc + J13*dthetaC;       
        wheel2 = J21*dXc + J22*dYc + J23*dthetaC;       
        wheel3 = J31*dXc + J32*dYc + J33*dthetaC;      

        wheel1speed = speedCalc(wheel1, 1);
        wheel2speed = speedCalc(wheel2, 2);
        wheel3speed = speedCalc(wheel3, 3);
        
        // if (ref_time > Period)
        // {
        //     wheel1speed = 64;
        //     wheel2speed = 64;
        //     wheel3speed = 64;
        // }
        
        ser->flush();
        // Input speeds to motor controllers
        // Motor Controller 1 -- Note: Motor 1 requires negative speed output from kinematics model
        control_motors_sep(6,wheel3speed,7,wheel2speed,128);
        // control_motors_sep(6,84,7,84,128);
        // Motor Controller 2
        control_motors_sep(6,64,7,wheel1speed,129);
        // control_motors_sep(6,64,7,100,129);
        

        //Publish necessary topics
        ref_pub.publish(reftraj);
        viconFeedback_pub.publish(viconFeedback);
        ros::spinOnce();
        loop_rate.sleep();
        ser->flush();   
    }
loop_rate.sleep();
ser->flush();
control_motors_sep(6,64,7,64,128);
control_motors_sep(6,64,7,64,129);
ser->close();
return 0;
}