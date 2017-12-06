#ifndef __BOAT_H_INCL__
#define __BOAT_H_INCL__

#include <Arduino.h>
#include <RBD_Timer.h> // https://github.com/alextaujenis/RBD_Timer
#include <RBD_Motor.h> // https://github.com/alextaujenis/RBD_Motor

/**
 * This class controls the Boat as an entity of dual motors on 
 * its left and right side. 
 * 
 * @author   Nir
 */
class Boat
{
public:
    // Constructor
    Boat(int left_pin, int right_pin, unsigned char throttle_in_pct = 80 )
        :left_motor_(left_pin)
        ,right_motor_(right_pin)
        ,throttle_in_pct_(throttle_in_pct)
    {
        this->left_motor_.off();
        this->right_motor_.off();
    }

    // Destructor
    virtual ~Boat()
    {}

    ///need to call this function in the main loop function for executing the resl-time API
    void update();


    /**
     * go forward at full speed for a specifiec @time
     * 
     * @author Nir (11/30/2017)
     * 
     * @param time 
     */
    void go();

    /**
     * stop the boat completely
     */
    void stop();

    /**
     * turn the boat right or left for a specified time in msec. 
     * in case @time given is 0 continue indefintely. the @angle 
     * provided is translated as a right turn for angles up to 180, 
     * angles larger than that are caluclated as left turns of 
     * 360-@angle 
     * 
     * @author Nir (11/29/2017)
     * 
     * @param time,angle 
     */
    void turn(int angle);

private:

    /**
     * convert from an angle (0-180 degree) to a speed on the motor. 
     * angles larger than angle_range degrees will be constrained to
     * 30 degree. the mapping from angle to speed is in reverse 
     * ratio from 0 to max_throttle. e.g angle 0 will be translated 
     * to max throttle angles larger than angle_range wil be 
     * translated to 0 speed. 
     * 
     * @author Nir (12/5/2017)
     * 
     * @param angle 
     * 
     * @return int 
     */
    int map_angle_to_speed_(int angle);

private:

    //what is the angle which will yield a smoother turn (e.g both engines work)
    static unsigned char angle_range_; 

    RBD::Motor left_motor_;
    RBD::Motor right_motor_;
    unsigned char throttle_in_pct_; //max throotle in pct
};


#endif // __BOAT_H_INCL__
