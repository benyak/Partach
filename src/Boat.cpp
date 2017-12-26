#include "Boat.h"


unsigned char Boat::angle_range_ = 30; 



/**
 * send the boat fwd up to max allowed speed
 * 
 * @author Nir (12/5/2017)
 * 
 * @param time 
 */
void 
Boat::go()
{
    this->left_motor_.setSpeedPercent(this->throttle_in_pct_);
    this->right_motor_.setSpeedPercent(this->throttle_in_pct_);
}

void 
Boat::stop()
{
    this->left_motor_.off();
    this->right_motor_.off();
}

void 
Boat::update()
{
    this->left_motor_.update();
    this->right_motor_.update();
}

void Boat::turn(int angle)
{
    if (angle < 180) { // rigt turn
      Serial.print("Boat::turn right turn of ");Serial.println(angle);
      Serial.print("right motor at=");Serial.print(this->throttle_in_pct_);Serial.println("%");
      this->right_motor_.setSpeedPercent(this->throttle_in_pct_);
      Serial.print("left motor at=");Serial.print(this->map_angle_to_speed_(angle));Serial.println("%");
      this->left_motor_.setSpeedPercent(this->map_angle_to_speed_(angle));
    }
    else //left turn (larger than 180
    {
      int left_angle = 360 - angle;
      Serial.print("Boat::turn left turn of ");Serial.println(left_angle);
      Serial.print("left motor at=");Serial.print(this->throttle_in_pct_);Serial.println("%");
      this->left_motor_.setSpeedPercent(this->throttle_in_pct_);
      Serial.print("right motor at=");Serial.print(this->map_angle_to_speed_(left_angle));Serial.println("%");
      this->right_motor_.setSpeedPercent(this->map_angle_to_speed_(left_angle));
    }

}

int 
Boat::map_angle_to_speed_(int angle)
{
    int speed = constrain(angle,0,Boat::angle_range_);
    speed = map(speed,0,Boat::angle_range_,this->throttle_in_pct_,50);
    return speed;
}

