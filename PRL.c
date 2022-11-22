/*
PRL v1.1
Creator: Jonathan Harris, Jacob Ross
Advisors: Zach Zimmerman, Nathan Povendo, Qbit
the Plainview Robotics Library is the entire collection of commands used by the Plainview Robotics Team.
These functions rely on the internal counters of the wheels to make commands as accurate as possible so 
being as accurate as possible in the setup is important.

SETUP:
The setup of this library for use is extremely easy and only requires a couple of steps.
1. The first thing that needs to be done is adding the following two lines ABOVE the include for this library in the main.c file:
    #include <stdlib.h>
    #include <math.h>
2. If you are using the library for a demo bot (anything but the create), then you need to call the function set_wheel_ticks(float l, float r)
	at the beginning of your code even before wait for light or anything else. The parameters should be the ticks per one wheel rotation for both wheels
    respectively. These two numbers should be as close to the actual number of ticks per rotation as physically possible.
3. If you are using a create bot then disreguard step 2 and use the function create_activate() instead of the usual create_connect() function. This will 
	notify the library that a create is in use so that it will use the correct functions
    
    
USE:
The same function call works for both the create and the demo bot assuming that when using a create you followed setup step #3. 
The list of functions is as follows:

*/
#define pi 3.14159265359

int create_in_use = 0;
int chain = 0;
int chain_start = 0;

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

float accel_distance = 5;
float accel_deg = 5;

//change the distance that the robot accels and deccels
void set_accel_window_drive(float distance){
    accel_distance = distance;
}

//change the angle that the robot accels and deccels
void set_accel_window_turn(float degrees){
    accel_deg = degrees;
}
float analog_avg(int port, int loops){
    int mass = 0;
    int i;
    for(i = 0; i < loops; i ++){
        mass += analog(port);
    }
    return mass/loops;
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

void start_chain(int num){
    chain = num;
    chain_start = num;
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
void reckless_drive(float distance, int speed){
    cmpc(right_wheel);
    while(abs(gmpc(right_wheel)) < distance * 83){
        mav(right_wheel, speed);
        mav(left_wheel, speed);
        msleep(5);
    }
    mav(right_wheel, 0);
    mav(left_wheel, 0);
    msleep(20);
}
void reckless_turn(float degree, int speed){
    cmpc(right_wheel);
    float distance = degree*(pi/180)*(distance_between_wheels/2)*83;
    while(abs(gmpc(right_wheel)) < distance){
    	mav(right_wheel, -speed);
        mav(left_wheel, speed);
        msleep(5);
    }
    mav(right_wheel, 0);
    mav(left_wheel, 0);
    msleep(20);
}
void auto_callibrate(int port, float an_to_wheel){
    int exit = 0;
    int white = analog_avg(port, 10);
    int black = 0; 
    int grey = -100;
    while(exit == 0){
        mav(right_wheel, 100);
        mav(left_wheel, 100);
        msleep(5);
        float val = analog_avg(port, 5);
        if(val > black){
            black = val; 
        }
        if(black > white + 100){
            grey = (black+white)/2;
        }
        if(analog(port) < grey){
            exit = 1;
        }
    }
    printf("white: %d\nblack: %d\ngrey: %d\n",white,black,grey);
    mav(right_wheel, 0);
    mav(left_wheel, 0);
    msleep(20);
    grey_value = grey;
    black_and_white_diff = black - white;
    reckless_drive(16,200);
    reckless_turn(87,200);
    exit = 0;
    float speed_mod = 0;
    float right_speed = 0;
    float left_speed = 0;
    cmpc(0);
    while(gmpc(0) < 5320){
        float error = grey_value - analog(0);
        speed_mod = error*0.05;
        right_speed = 300-speed_mod;
        left_speed = 300+speed_mod;
        mav(right_wheel, right_speed);
        mav(left_wheel, left_speed);
        msleep(5);
    }
    printf("right: %f\n", right_speed);
    printf("left: %f\n", left_speed);
    float actual_ratio = right_speed/left_speed;
    float desired_ratio = right_wheel_tpr/left_wheel_tpr;
    if(fabs(actual_ratio - desired_ratio) > 0.01){
        printf("\nWARNING! WE ARE OUT OF CALIBRATION!!!\n");
        printf("Readjusting the motors to drive in a straight line, distances will be off slightly\n");
        set_wheel_ticks((right_wheel_tpr*left_speed)/right_speed, right_wheel_tpr);
        
    }
    mav(right_wheel, 0);
    mav(left_wheel, 0);
    msleep(20);
}

void d_drive(float distance, int speed){
    int right_wheel_target_ticks = (right_wheel_tpr/wheel_circumference) * distance;
    int left_wheel_target_ticks = (left_wheel_tpr/wheel_circumference) * distance;
    float accel_window_ticks = (right_wheel_target_ticks/wheel_circumference) * accel_distance;
    if(accel_window_ticks > right_wheel_target_ticks / 2){
        accel_window_ticks = right_wheel_target_ticks / 2 - 1;
    }
    float cps = abs(speed);
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
            if(chain != chain_start){
                speed_mod = 1;
            }
            if(speed_mod < 0.2){
                speed_mod = 0.2;
            }
            mav(right_wheel, right_wheel_target_speed * speed_mod);
            mav(left_wheel, left_wheel_target_speed * speed_mod);
        }else if (abs(gmpc(right_wheel)) > right_wheel_target_ticks - accel_window_ticks){
            speed_mod = (right_wheel_target_ticks - abs(gmpc(right_wheel)))/accel_window_ticks;
            if(chain != 1){
                speed_mod = 1;
            }
            if(speed_mod < 0.2){
                speed_mod = 0.2;
            }
            mav(right_wheel, right_wheel_target_speed * speed_mod);
            mav(left_wheel, left_wheel_target_speed * speed_mod);
        }else{
            speed_mod = 1;
            mav(right_wheel, right_wheel_target_speed);
            mav(left_wheel, left_wheel_target_speed);
        }
    }
    if(chain == 1){
        chain = 0;
        chain_start = 0;
        mav(right_wheel, 0);
        mav(left_wheel, 0);
        msleep(5);
    }else{
        chain --;
    }
}


void d_line_follow(float distance, float speed, int port){
    cmpc(0);
   	cmpc(1);
    int prev_right_ticks = 0;
    int prev_left_ticks = 0;
    long double local_x = 0;
    long double local_y = 0;
    long double local_theta = 0;
    float speed_mod = 0;
    
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
            speed_mod = (gmpc(0)/accel_window_ticks);
            if(chain != chain_start){
                speed_mod = 1;
            }
            if(speed_mod < 0.1){
                speed_mod = 0.1;
            }
        }else if(gmpc(0) > (distance*(right_wheel_tpr/wheel_circumference)) - accel_window_ticks){
            speed_mod = ((distance*(right_wheel_tpr/wheel_circumference)) - gmpc(right_wheel))/accel_window_ticks;
            if(chain > 1){
                speed_mod = 1;
            }
            if(speed_mod < 0.1){
                speed_mod = 0.1;
            }
        }else{
            speed_mod = 1;
        }
    	mav(0,*(speeds+1)*speed_mod);//left wheel
    	mav(1,*speeds*speed_mod);
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
    if(chain == 1){
        chain = 0;
        chain_start = 0;
        mav(right_wheel, 0);
        mav(left_wheel, 0);
        msleep(5);
    }else{
        chain --;
    }
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
    float speed_mod = 0;
    double theta = 0;
    cmpc(left_wheel);
    cmpc(right_wheel);
    double used_accel_deg = accel_deg;
    if(used_accel_deg > degree/2){
        used_accel_deg = (degree/2)-1;
    }
    while(abs(theta) < degree){
        if(theta < used_accel_deg){
            speed_mod = (theta/used_accel_deg);
            if(chain != chain_start){
                speed_mod = 1;
            }
            if(speed_mod < 0.2){
                speed_mod = 0.2;
            }
        }else if(theta > degree - used_accel_deg){
            speed_mod = (degree - theta)/used_accel_deg;
            if(chain != 1){
                speed_mod = 1;
                printf("I am not done chaining! %d\n", chain);
            }if(chain == 1){
                printf("I am ending the chain %f\n", speed_mod);
            }
            if(speed_mod < 0.2){
                speed_mod = 0.2;
            }
        }else{
            speed_mod = 1;
        }
        mav(right_wheel, right_speed*speed_mod);
        mav(left_wheel, left_speed*speed_mod);
        msleep(5);
        if(abs(right_radius) > abs(left_radius)){
          	theta = ((gmpc(right_wheel)/right_wheel_tpc)/(right_radius))*57.29577951;
        }else{
            theta = ((gmpc(left_wheel)/left_wheel_tpc)/(left_radius))*57.29577951;
        }
         
    }
  	if(chain == 1){
        chain = 0;
        chain_start = 0;
        mav(right_wheel, 0);
        mav(left_wheel, 0);
        msleep(5);
        printf("chain_done\n");
    }else{
        chain --;
        
    }
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
    float speed_mod = 0;
    double theta = 0;
    cmpc(left_wheel);
    cmpc(right_wheel);
    double used_accel_deg = accel_deg;
    if(used_accel_deg > degree/2){
        used_accel_deg = (degree/2)-1;
    }
    while(abs(theta) < degree){
        if(theta < used_accel_deg){
            speed_mod = (theta/used_accel_deg);
            if(chain != chain_start){
                speed_mod = 1;
            }

            if(speed_mod < 0.2){
                speed_mod = 0.2;
            }
        }else if(theta > degree - used_accel_deg){
            speed_mod = (degree - theta)/used_accel_deg;
            if(chain > 1){
                speed_mod = 1;
            }
            if(speed_mod < 0.2){
                speed_mod = 0.2;
            }
        }else{
            speed_mod = 1;
        }
        mav(right_wheel, right_speed*speed_mod);
        mav(left_wheel, left_speed*speed_mod);
        msleep(5);
        if(abs(right_radius) > abs(left_radius)){
          	theta = ((gmpc(right_wheel)/right_wheel_tpc)/(right_radius))*57.29577951;
        }else{
            theta = ((gmpc(left_wheel)/left_wheel_tpc)/(left_radius))*57.29577951;
        }
         
    }
    if(chain == 1){
        chain = 0;
        chain_start = 0;
        mav(right_wheel, 0);
        mav(left_wheel, 0);
        msleep(5);
    }else{
        chain --;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//CREATE FUNCTIONS

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void create_activate(){
    create_in_use = 1;
    create_connect();
    printf("create connected!\n");
    create_full();
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

float create_speed_filter(float num){
    float a = num;
    if(abs(a) < 20){
        a = 20;
        return a*num/abs(num);
    }
    return num;
}

void r_drive(float distance, int speed){
    float right = create_gmec('r');
    float left = create_gmec('l');
    
    float start_right_ticks = right;
    float start_left_ticks = left;
	float dist_travelled = 0;
    float speed_mod = 0.1;
    while(abs(dist_travelled) < distance){
        float speed_adj = -((left-start_left_ticks) - (right-start_right_ticks))*1;
        if(abs(dist_travelled) < accel_distance){
            speed_mod = (abs(dist_travelled)/accel_distance);
            if(chain != chain_start){
                speed_mod = 1;
            }
            if(speed_mod < 0.1){
                speed_mod = 0.1;
            }
        }else if(abs(dist_travelled) > distance - accel_distance){
            speed_mod = (distance - abs(dist_travelled))/accel_distance;
            if(chain != 1){
                speed_mod = 1;
            }
            if(speed_mod < 0.1){
                speed_mod = 0.1;
            }
        }else{
            speed_mod = 1;
        }
        float right_speed = (create_speed_filter(speed)-speed_adj)*speed_mod;
        float left_speed = (create_speed_filter(speed)+speed_adj)*speed_mod;
        create_drive_direct(left_speed,right_speed);
        msleep(5);  	
        right = create_gmec('r');
        left = create_gmec('l');
        float right_movement = (right-start_right_ticks) * (pi * 7.2 / 508.8);
        float left_movement = (left-start_left_ticks) * (pi * 7.2 / 508.8);
        dist_travelled = (right_movement + left_movement)/2;
        
    }
    while(fabs(dist_travelled) > distance){
        printf("we overshot!\n");
        create_drive_direct(create_speed_filter(-speed/100),create_speed_filter(-speed/100));
        right = create_gmec('r');
        left = create_gmec('l');
        float right_movement = (right-start_right_ticks) * (pi * 7.2 / 508.8);
    	float left_movement = (left-start_left_ticks) * (pi * 7.2 / 508.8);
    	dist_travelled = (right_movement + left_movement)/2;
    }
    printf("dist travelled: %f", dist_travelled);
    if(chain == 1){
        chain = 0;
        chain_start = 0;
        mav(right_wheel, 0);
        mav(left_wheel, 0);
        msleep(5);
    }else{
        chain --;
    }
}

void r_line_follow(float distance, float speed, int port){
    float right = create_gmec('r');
    float left = create_gmec('l');
    
    float start_right_ticks = right;
    float start_left_ticks = left;
	float dist_travelled = 0;
    float speed_mod = 0.1;
    while(abs(dist_travelled) < distance){
        float speed_adj = (analog(port)-grey_value) * 0.02;
        if(abs(dist_travelled) < accel_distance){
            speed_mod = (abs(dist_travelled)/accel_distance);
            if(chain != chain_start){
                speed_mod = 1;
            }
            if(speed_mod < 0.1){
                speed_mod = 0.1;
            }
        }else if(abs(dist_travelled) > distance - accel_distance){
            speed_mod = (distance - abs(dist_travelled))/accel_distance;
            if(chain != 1){
                speed_mod = 1;
            }
            if(speed_mod < 0.1){
                speed_mod = 0.1;
            }
        }else{
            speed_mod = 1;
        }
        float right_speed = (create_speed_filter(speed)-speed_adj)*speed_mod;
        float left_speed = (create_speed_filter(speed)+speed_adj)*speed_mod;
        create_drive_direct(left_speed,right_speed);
        msleep(5);  	
        right = create_gmec('r');
        left = create_gmec('l');
        float right_movement = (right-start_right_ticks) * (pi * 7.2 / 508.8);
        float left_movement = (left-start_left_ticks) * (pi * 7.2 / 508.8);
        dist_travelled = (right_movement + left_movement)/2;
        
    }
    while(fabs(dist_travelled) > distance){
        printf("we overshot!\n");
        create_drive_direct(create_speed_filter(-speed/100),create_speed_filter(-speed/100));
        right = create_gmec('r');
        left = create_gmec('l');
        float right_movement = (right-start_right_ticks) * (pi * 7.2 / 508.8);
    	float left_movement = (left-start_left_ticks) * (pi * 7.2 / 508.8);
    	dist_travelled = (right_movement + left_movement)/2;
    }
    printf("dist travelled: %f", dist_travelled);
    if(chain == 1){
        chain = 0;
        chain_start = 0;
        mav(right_wheel, 0);
        mav(left_wheel, 0);
        msleep(5);
    }else{
        chain --;
    }
}
void r_right_turn(float degree, float speed, double radius){
    
    double right_radius = radius-distance_between_wheels/2;
    double left_radius = radius+distance_between_wheels/2;
    double right_wheel_cps = (speed*0.017453) * right_radius;
    double left_wheel_cps = (speed*0.017453) * left_radius;
    float right_speed = right_wheel_cps * 10;
    float left_speed = left_wheel_cps * 10;
    float speed_mod = 0;
    double theta = 0;
    int start_right = create_gmec('r');
    int start_left = create_gmec('l');
    int right = start_right;
    int left = start_left;
    double used_accel_deg = accel_deg;
    if(used_accel_deg > degree/2){
        used_accel_deg = (degree/2)-1;
    }
    while(abs(theta) < degree){
        if(theta < used_accel_deg){
            speed_mod = (theta/used_accel_deg);
            if(chain != chain_start){
                speed_mod = 1;
            }
            if(speed_mod < 0.1){
                speed_mod = 0.1;
            }
        }else if(theta > degree - used_accel_deg){
            speed_mod = (degree - theta)/used_accel_deg;
            if(chain != 1){
                speed_mod = 1;
            }
            if(speed_mod < 0.1){
                speed_mod = 0.1;
            }
        }else{
            speed_mod = 1;
        }
        create_drive_direct(left_speed*speed_mod, right_speed*speed_mod);
        msleep(20);
        if(abs(right_speed) > abs(left_speed)){
            right = create_gmec('r');
            float right_arc = (right-start_right) * (pi * 7.2 / 508.8); 
            theta = ((right_arc)/right_radius)*57.296;
            printf("value: %f\n",theta);
        }else{
            left = create_gmec('l');
            float left_arc = (left-start_left) * (pi * 7.2 / 508.8); 
            theta = ((left_arc)/left_radius)*57.296;
            printf("value: %f\n",theta);
        }
    } 
    if(chain == 1){
        chain = 0;
        chain_start = 0;
        mav(right_wheel, 0);
        mav(left_wheel, 0);
        msleep(5);
    }else{
        chain --;
    }
}
void r_left_turn(float degree, float speed, double radius){
    double right_radius = radius+distance_between_wheels/2;
    double left_radius = radius-distance_between_wheels/2;
    double right_wheel_cps = (speed*0.017453) * right_radius;
    double left_wheel_cps = (speed*0.017453) * left_radius;
    float right_speed = right_wheel_cps * 10;
    float left_speed = left_wheel_cps * 10;
    float speed_mod = 0;
    double theta = 0;
    int start_right = create_gmec('r');
    int start_left = create_gmec('l');
    int right = start_right;
    int left = start_left;
    double used_accel_deg = accel_deg;
    if(used_accel_deg > degree/2){
        used_accel_deg = (degree/2)-1;
    }
    while(abs(theta) < degree){
        
        if(theta < used_accel_deg){
            speed_mod = (theta/used_accel_deg);
            if(chain != chain_start){
                speed_mod = 1;
            }
            if(speed_mod < 0.2){
                speed_mod = 0.2;
            }
        }else if(theta > degree - used_accel_deg){
            speed_mod = (degree - theta)/used_accel_deg;
            if(chain != 1){
                speed_mod = 1;
            }
            if(speed_mod < 0.2){
                speed_mod = 0.2;
            }
        }else{
            speed_mod = 1;
        }
        
        create_drive_direct(left_speed*speed_mod, right_speed*speed_mod);
        msleep(20);
        if(abs(right_speed) > abs(left_speed)){
            right = create_gmec('r');
            float right_arc = (right-start_right) * (pi * 7.2 / 508.8); 
            theta = ((right_arc)/right_radius)*57.296;
            printf("value: %f\n",theta);
        }else{
            left = create_gmec('l');
            float left_arc = (left-start_left) * (pi * 7.2 / 508.8); 
            theta = ((left_arc)/left_radius)*57.296;
            printf("value: %f\n",theta);
        }
    } 
    if(chain == 1){
        chain = 0;
        chain_start = 0;
        mav(right_wheel, 0);
        mav(left_wheel, 0);
        msleep(5);
    }else{
        chain --;
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//UNIVERSAL FUNCTIONS

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void drive(float distance, int speed){
    printf("hello\n");
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