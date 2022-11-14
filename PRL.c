/*
PRL v0.56
Creator: Jonathan Harris, Jacob Ross
Advisors: Zach Zimmerman, Nathan Povendo, Qbit
the Plainview Robotics Library is the entire collection of commands used by the Plainview Robotics Team.
These functions rely on the internal counters of the wheels to make commands as accurate as possible so 
being as accurate as possible in the setup is important.
*/
#define pi 3.14159265359

int create_in_use = 0;

float wheel_circumference = 21.36283;
float distance_between_wheels = 16.25;
float right_wheel_tpr = 1000;
float left_wheel_tpr = 1000;
float right_wheel_tpc = 0;
float left_wheel_tpc = 0;


float grey_value = 1700;
float black_and_white_diff = 900;
float minimum_line_follow_radius = 30;
float maximum_line_follow_radius = 1000;

float accel_distance = 2;
float accel_deg = 5;

//change the distance that the robot accels and deccels
void set_accel_window_drive(float distance){
    accel_distance = distance;
}

//change the angle that the robot accels and deccels
void set_accel_window_turn(float degrees){
    accel_deg = degrees;
}

//function that takes the error value received from the line follow sensor and gives back a radius for the turn
float line_follow_calculate_radius(int error_value, float max_radius, float min_radius){
    //NOTE: white means positive error and black means negative error FOR LEFT SIDE LINE FOLLOW
    float error_modifier = black_and_white_diff*0.4;
    if(error_value > error_modifier){
        return -min_radius;
    }else if(error_value < -error_modifier){
        return min_radius;
    }else if(error_value < 0){
        return (error_value*((max_radius-min_radius)/error_modifier)) + max_radius;
    }else{
        return (error_value*((max_radius-min_radius)/error_modifier)) - max_radius;
    }
}

//takes the speed of the origin and get the individual wheel speeds
float * calculate_wheel_speed(float radius, float speed){
    static float speeds[2];
    float theta = speed/radius;
    speeds[0] = theta * (radius-(distance_between_wheels*0.5));
    speeds[1] = theta * (radius+(distance_between_wheels*0.5));
    return speeds;
}

//servo function by Jacob of Noble High School (written on August 8,2019)
void servo(int port, int position, int speed){   
    int current = get_servo_position(port);
    if(position > 2047){
        position = 2047;
    } 
    if(position < 0){
        position = 0;
    }   
    while(current <= position-speed || current >= position+speed){ 
        if(current < position){
            current += speed; 
        }
        if(current > position){
            current -= speed;
        }
        set_servo_position(port, current);
        msleep(2);
    }
    set_servo_position(port,position); 
} 



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//DEMO FUNCTIONS

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define right_wheel 0
#define left_wheel 1
//speed to ticks_per_second = (1.08 * abs(speed)) - 2

void set_wheel_ticks(int left, int right){
    left_wheel_tpr = left;
    right_wheel_tpr = right;
    left_wheel_tpc = left/wheel_circumference;
    right_wheel_tpc = right/wheel_circumference;
}

void spin_motor(int port, int ticks, int speed){
    cmpc(port);
    while(gmpc(port) < ticks){
        mav(port, speed);
    }
    mav(port,0);
    msleep(20);
}

void d_drive(float distance, int speed){
    int right_wheel_target_ticks = (right_wheel_tpr/wheel_circumference) * distance;
    int left_wheel_target_ticks = (left_wheel_tpr/wheel_circumference) * distance;
    float accel_window_ticks = (right_wheel_target_ticks/wheel_circumference) * accel_distance;
    if(accel_window_ticks > right_wheel_target_ticks / 2){
        accel_window_ticks = right_wheel_target_ticks / 2 - 1;
    }
    float cps = speed;
    float fastest_speed = 1367.09090909/((right_wheel_tpc+left_wheel_tpc)/2);
    if(cps > fastest_speed){
        cps = fastest_speed;
    }
    float tps = ((right_wheel_tpc+left_wheel_tpc)/2)*cps;
    float spt = 1/tps;
    float seconds_to_completion = right_wheel_target_ticks * spt;
    int right_wheel_target_speed =  ((right_wheel_target_ticks/seconds_to_completion) - 2)/1.08;
    int left_wheel_target_speed = ((left_wheel_target_ticks/seconds_to_completion) - 2)/1.08;
    if(speed < 0 ){
        right_wheel_target_speed = -right_wheel_target_speed;
        left_wheel_target_speed = -left_wheel_target_speed;
    }
    cmpc(right_wheel);
    cmpc(left_wheel);
    float speed_mod = 0.05;
    while(abs(gmpc(right_wheel)) < right_wheel_target_ticks || abs(gmpc(left_wheel)) < left_wheel_target_ticks){
        if(abs(gmpc(right_wheel)) < accel_window_ticks){
            speed_mod = abs(gmpc(right_wheel))/accel_window_ticks;
            if(speed_mod < 0.1){
                speed_mod = 0.1;
            }
            mav(right_wheel, right_wheel_target_speed * speed_mod);
            mav(left_wheel, left_wheel_target_speed * speed_mod);
        }else if (abs(gmpc(right_wheel)) > right_wheel_target_ticks - accel_window_ticks){
            speed_mod = (right_wheel_target_ticks - abs(gmpc(right_wheel)))/accel_window_ticks;
            if(speed_mod < 0.1){
                speed_mod = 0.1;
            }
            mav(right_wheel, right_wheel_target_speed * speed_mod);
            mav(left_wheel, left_wheel_target_speed * speed_mod);
        }else{
            speed_mod = 1;
            mav(right_wheel, right_wheel_target_speed);
            mav(left_wheel, left_wheel_target_speed);
        }
        printf("right: %d\n", right_wheel_target_speed);
        printf("left: %d\n\n", left_wheel_target_speed);
    }
    mav(right_wheel, 0);
    mav(left_wheel, 0);
    msleep(5);
}


void d_line_follow(float distance, float speed, int port){
    cmpc(0);
   	cmpc(1);
    int prev_right_ticks = 0;
    int prev_left_ticks = 0;
    long double local_x = 0;
    long double local_y = 0;
    long double local_theta = 0;
    float speed_modifier = 0;
    
    float cps = speed;
    float fastest_speed = 1367.09090909/((right_wheel_tpc+left_wheel_tpc)/2);
    if(cps > fastest_speed){
        cps = fastest_speed;
    }
    float tps = ((right_wheel_tpc+left_wheel_tpc)/2)*cps;
    float target_speed = (tps + 2)/1.08;
    
    float accel_window_ticks = (right_wheel_tpr/wheel_circumference) * accel_distance;
    if(accel_window_ticks > (distance*(right_wheel_tpr/wheel_circumference)) / 2){
        accel_window_ticks = (distance*(right_wheel_tpr/wheel_circumference) / 2) - 1;
    }
    while(local_y < distance*0.975609756){
        long double radius = line_follow_calculate_radius(grey_value-analog(port), maximum_line_follow_radius, minimum_line_follow_radius);
    	float *speeds = calculate_wheel_speed(radius,target_speed);
        if(gmpc(0) < accel_window_ticks){
            speed_modifier = (gmpc(0)/accel_window_ticks);
            if(speed_modifier < 0.1){
                speed_modifier = 0.1;
            }
        }else if(gmpc(0) > (distance*(right_wheel_tpr/wheel_circumference)) - accel_window_ticks){
            speed_modifier = ((distance*(right_wheel_tpr/wheel_circumference)) - gmpc(right_wheel))/accel_window_ticks;
            if(speed_modifier < 0.1){
                speed_modifier = 0.1;
            }
        }else{
            speed_modifier = 1;
        }
    	mav(0,*(speeds+1)*speed_modifier);//left wheel
    	mav(1,*speeds*speed_modifier);
        msleep(30);
        float rmt = (gmpc(0)-prev_right_ticks)/right_wheel_tpc;
        float lmt = (gmpc(1)-prev_left_ticks)/left_wheel_tpc;
        float arc_length = (rmt+lmt)/2;
        prev_right_ticks = gmpc(0);
        prev_left_ticks = gmpc(1);
        double theta = arc_length/radius;
        long double a = (cos(theta)*radius)-radius;
        long double b = sin(theta)*radius;
    	long double c = sqrt((a*a)+(b*b));
        a = sqrt(a*a);
        long double angle_a = asin(a/c);
        long double angle_y = (pi/2) - local_theta + angle_a;
        local_theta += theta;
        local_y += sin(angle_y)*c;
        local_x += sqrt((c*c)-(local_y*local_y));
        if(isnan(local_y)){
            local_y = 0.01;
        }
    }
    mav(0,0);
    mav(1,0);
    msleep(50);
}

void d_right_turn(float degree, float speed, double radius){
    
    double right_radius = radius-distance_between_wheels/2;
    double left_radius = radius+distance_between_wheels/2;
    printf("%f\n",right_radius);
    double right_wheel_cps = (speed*0.017453) * right_radius;
    double left_wheel_cps = (speed*0.017453) * left_radius;
    double right_wheel_tps = right_wheel_cps * right_wheel_tpc;
    double left_wheel_tps = left_wheel_cps * left_wheel_tpc;
    
    double right_speed = (right_wheel_tps+2)/1.08;
    double left_speed = (left_wheel_tps+2)/1.08;
    float speed_modifier = 0;
    double theta = 0;
    cmpc(left_wheel);
    cmpc(right_wheel);
    double used_accel_deg = accel_deg;
    if(used_accel_deg > degree/2){
        used_accel_deg = (degree/2)-1;
    }
    while(abs(theta) < degree){
        if(theta < used_accel_deg){
            speed_modifier = (theta/used_accel_deg);
            if(speed_modifier < 0.2){
                speed_modifier = 0.2;
            }
        }else if(theta > degree - used_accel_deg){
            speed_modifier = (degree - theta)/used_accel_deg;
            if(speed_modifier < 0.2){
                speed_modifier = 0.2;
            }
        }else{
            speed_modifier = 1;
        }
        mav(right_wheel, right_speed*speed_modifier);
        mav(left_wheel, left_speed*speed_modifier);
        msleep(5);
        if(abs(right_radius) > abs(left_radius)){
          	theta = ((gmpc(right_wheel)/right_wheel_tpc)/(right_radius))*57.29577951;
        }else{
            theta = ((gmpc(left_wheel)/left_wheel_tpc)/(left_radius))*57.29577951;
        }
         
    }
    mav(right_wheel,0);
    mav(left_wheel,0);
    msleep(20);
    
}
void d_left_turn(float degree, float speed, double radius){
    
    double right_radius = radius+distance_between_wheels/2;
    double left_radius = radius-distance_between_wheels/2;
    printf("%f\n",right_radius);
    double right_wheel_cps = (speed*0.017453) * right_radius;
    double left_wheel_cps = (speed*0.017453) * left_radius;
    double right_wheel_tps = right_wheel_cps * right_wheel_tpc;
    double left_wheel_tps = left_wheel_cps * left_wheel_tpc;
    
    double right_speed = (right_wheel_tps+2)/1.08;
    double left_speed = (left_wheel_tps+2)/1.08;
    float speed_modifier = 0;
    double theta = 0;
    cmpc(left_wheel);
    cmpc(right_wheel);
    double used_accel_deg = accel_deg;
    if(used_accel_deg > degree/2){
        used_accel_deg = (degree/2)-1;
    }
    while(abs(theta) < degree){
        if(theta < used_accel_deg){
            speed_modifier = (theta/used_accel_deg);
            if(speed_modifier < 0.2){
                speed_modifier = 0.2;
            }
        }else if(theta > degree - used_accel_deg){
            speed_modifier = (degree - theta)/used_accel_deg;
            if(speed_modifier < 0.2){
                speed_modifier = 0.2;
            }
        }else{
            speed_modifier = 1;
        }
        mav(right_wheel, right_speed*speed_modifier);
        mav(left_wheel, left_speed*speed_modifier);
        msleep(5);
        if(abs(right_radius) > abs(left_radius)){
          	theta = ((gmpc(right_wheel)/right_wheel_tpc)/(right_radius))*57.29577951;
        }else{
            theta = ((gmpc(left_wheel)/left_wheel_tpc)/(left_radius))*57.29577951;
        }
         
    }
    mav(right_wheel,0);
    mav(left_wheel,0);
    msleep(20);
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//CREATE FUNCTIONS

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void create_activate(){
    create_in_use = 1;
    create_connect();
    wheel_circumference = 22.61946711;
    distance_between_wheels = 23.5;
    left_wheel_tpr = 508.8;
    right_wheel_tpr = 508.8;
    left_wheel_tpc = 508.8/wheel_circumference;
    right_wheel_tpc = 508.8/wheel_circumference;

}

int create_gmec(char wheel){
    if(wheel == 'r' || wheel == 'R'){
        create_write_byte(142);
		create_write_byte(101);
		char buffer[28];
		create_read_block(buffer, sizeof (buffer));
		return (buffer[2] << 8) | (buffer[3] << 0);
    }else if(wheel == 'l' || wheel == 'L'){
        create_write_byte(142);
		create_write_byte(101);
		char buffer[28];
		create_read_block(buffer, sizeof (buffer));
		return (buffer[0] << 8) | (buffer[1] << 0);
    }
    return 0;
}

void r_drive(float distance, int speed){
    
}

void r_line_follow(float distance, float speed, int port){
   
}

void r_right_turn(float degree, float speed, double radius){
    
}
void r_left_turn(float degree, float speed, double radius){
   
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//UNIVERSAL FUNCTIONS

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void drive(float distance, int speed){
    if(create_in_use == 0){
        d_drive(distance, speed);
    }else{
        r_drive(distance, speed);
    }
}

void line_follow(float distance, int speed, int port){
    if(create_in_use == 0){
        d_line_follow(distance, speed, port);
    }else{
        r_line_follow(distance, speed, port);
    }
}

void right_turn(float degree, float speed, double radius){
    if(create_in_use == 0){
        d_right_turn(degree, speed, radius);
    }else{
        r_right_turn(degree, speed, radius);
    }
}

void left_turn(float degree, float speed, double radius){
    if(create_in_use == 0){
        d_left_turn(degree, speed, radius);
    }else{
        r_left_turn(degree, speed, radius);
    }
}
