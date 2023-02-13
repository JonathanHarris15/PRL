/*
PRL v1.7
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

//speed to ticks_per_second = (1.08 * abs(speed)) - 2
#define pi 3.14159265359
#define right_wheel 0
#define left_wheel 1

int create_in_use = 0;
int chain = 1;
int chain_start = 1;

float wheel_circumference = 17.45547;
float distance_between_wheels = 11.43;
float right_wheel_tpr = 1000;
float left_wheel_tpr = 1000;
float right_wheel_tpc = 0;
float left_wheel_tpc = 0;

float max_drive_speed = 100;

float grey_value = 1700;
float black_and_white_diff = 500;
float minimum_line_follow_radius = 30;
float maximum_line_follow_radius = 1000;
float used_tape_width = 5.08;
float accel_distance = 10;
float accel_deg = 5;

int servo_desired[4] = {-1,-1,-1,-1};
int servo_current[4] = {-1,-1,-1,-1};
float servo_time[4] = {-1,-1,-1,-1};
double servo_finish_time[4];



////////////////////////////////////////////////////////////////
//HELPER FUNCTIONS
////////////////////////////////////////////////////////////////

void viprint(int val){
    printf("value: %d\n",val);
}
void vfprint(float val){
    printf("value: %f\n",val);
}
float lerp(float start, float finish, float progress){
    return start + (finish-start)*progress;
}
void clear_wheels(){
    cmpc(right_wheel);
    cmpc(left_wheel);
}
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


////////////////////////////////////////////////////////////////
//SERVOS
////////////////////////////////////////////////////////////////

void update_servos(){
    servo_current[0] = get_servo_position(0);
    servo_current[1] = get_servo_position(1);
    servo_current[2] = get_servo_position(2);
    servo_current[3] = get_servo_position(3);
}
void clear_servos(){
    servo_desired[0] = -1;
    servo_desired[1] = -1;
    servo_desired[2] = -1;
    servo_desired[3] = -1;
}
void load_servo(int port, int position, int speed){
    servo_desired[port] = position;
    servo_time[port] = speed;
    update_servos();
}
void servo(int port,int position,int speed){
    mav(right_wheel,0);
    mav(left_wheel,0);
    msleep(10);
    servo_desired[port] = position;
    servo_time[port] = speed;
    update_servos();
    int exit = 0;
    int i = 0;
    float start = seconds();
    for(i = 0; i < 4;i ++){
        servo_finish_time[i] = servo_time[i] + start;
    }
    float multiplier = 0;
    double x = 0;
    while(exit == 0){
        exit = 1;
        for(i = 0; i < 4; i ++){
            if(servo_desired[i] != -1){ 
                if(seconds()-start < servo_time[i] && servo_current[i] != servo_desired[i]){
                    exit = 0;
                    x = (seconds()-start)/(servo_finish_time[i]-start);
                    float total_dist = servo_desired[i]-servo_current[i];
                    float sqt = x*x;
                    multiplier = sqt/(2.0 * (sqt-x)+1.0);
                    set_servo_position(i,servo_current[i] + (total_dist*multiplier));
                }else{
                    set_servo_position(i,servo_desired[i]);
                }
            }
        }
        msleep(5);
    }
    clear_servos();
}



////////////////////////////////////////////////////////////////
//DEMO
////////////////////////////////////////////////////////////////


void set_wheel_ticks(int left, int right){
    left_wheel_tpr = left;
    right_wheel_tpr = right;
    left_wheel_tpc = left/wheel_circumference;
    right_wheel_tpc = right/wheel_circumference;
    if(right_wheel_tpc > left_wheel_tpc){
        max_drive_speed = 1402/right_wheel_tpc;
    }else{
        max_drive_speed = 1402/left_wheel_tpc;
    }
    printf("max drive speed is %f\n",max_drive_speed);
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
//an_to_wheel = 16.8275

double servo_start_time;
void d_drive(float distance, float speed){
    if(speed > max_drive_speed){
        speed = max_drive_speed;
    }
    float r_tps = right_wheel_tpc * speed;
    float l_tps = left_wheel_tpc *speed;
    float r_speed = (r_tps+2)/1.08;
    float l_speed = (l_tps+2)/1.08;
   	clear_wheels();
    float distance_traveled = 0;
    int i = 0;
    float multiplier = 0;
    double x = 0;
    while(abs(distance_traveled) < distance){
        mav(right_wheel, r_speed);
        mav(left_wheel, l_speed);
        msleep(10);
        for(i = 0; i < 4; i ++){
            if(servo_desired[i] != -1){ 
                if(seconds() < servo_finish_time[i] && servo_current[i] != servo_desired[i]){
                    x = (seconds()-servo_start_time)/(servo_finish_time[i]-servo_start_time);  
                    float total_dist = servo_desired[i]-servo_current[i];
                    float sqt = x*x;
                    multiplier = sqt/(2.0 * (sqt-x)+1.0);
                    printf("%f\n",servo_current[i] + (total_dist*multiplier));
                    set_servo_position(i,servo_current[i] + (total_dist*multiplier));
                }
            }
        }
        distance_traveled = (gmpc(right_wheel)/right_wheel_tpc+gmpc(left_wheel)/left_wheel_tpc)/2;
    }
}
void d_drive_accel(float distance, float speed){
    if(speed > max_drive_speed){
        speed = max_drive_speed;
    }
    float r_tps = right_wheel_tpc * speed;
    float l_tps = left_wheel_tpc *speed;
    float r_speed = (r_tps+2)/1.08;
    float l_speed = (l_tps+2)/1.08;
   	clear_wheels();
    float distance_traveled = 0;
    int i = 0;
    servo_start_time = seconds();
    for(i = 0; i < 4;i ++){
        servo_finish_time[i] = servo_time[i] + servo_start_time;
    }
    float multiplier = 0;
    double x = 0;
    while(abs(distance_traveled) < distance){
        float a = fabs(distance_traveled)/distance;
        if(a < 0.1){a = 0.1;}
        mav(right_wheel, r_speed*a);
        mav(left_wheel, l_speed*a);
        msleep(10);
        for(i = 0; i < 4; i ++){
            if(servo_desired[i] != -1){ 
                if(seconds() < servo_finish_time[i] && servo_current[i] != servo_desired[i]){
                    x = (seconds()-servo_start_time)/(servo_finish_time[i]-servo_start_time);
                    float total_dist = servo_desired[i]-servo_current[i];
                    float sqt = x*x;
                    multiplier = sqt/(2.0 * (sqt-x)+1.0);
                    printf("%f\n",servo_current[i] + (total_dist*multiplier));
                    set_servo_position(i,servo_current[i] + (total_dist*multiplier));
                }
            }
        }
        distance_traveled = (gmpc(right_wheel)/right_wheel_tpc+gmpc(left_wheel)/left_wheel_tpc)/2;
    }
}
void d_drive_deccel(float distance, float speed){
    if(speed > max_drive_speed){
        speed = max_drive_speed;
    }
    float r_tps = right_wheel_tpc * speed;
    float l_tps = left_wheel_tpc *speed;
    float r_speed = (r_tps+2)/1.08;
    float l_speed = (l_tps+2)/1.08;
   	clear_wheels();
    float distance_traveled = 0;
    int i = 0;
    float multiplier = 0;
    long double x = 0;
    while(abs(distance_traveled) < distance){
        float a = (distance - fabs(distance_traveled))/distance;
        if(a < 0.1){a = 0.1;}
        if(abs(distance_traveled) > distance){
            a = 0;
        }
        mav(right_wheel, r_speed*a);
        mav(left_wheel, l_speed*a);
        for(i = 0; i < 4; i ++){
            if(servo_desired[i] != -1){ 
                if(seconds() < servo_finish_time[i] && servo_current[i] != servo_desired[i]){
                    float a = seconds()-servo_start_time;
                    float b = servo_finish_time[i]-servo_start_time;
                    x = a/b;
                    float total_dist = servo_desired[i]-servo_current[i];
                    float sqt = x*x;
                    multiplier = sqt/(2.0 * (sqt-x)+1.0);
                    printf("%f\n",servo_current[i] + (total_dist*multiplier));
                    set_servo_position(i,servo_current[i] + (total_dist*multiplier));
                }else{
                    set_servo_position(i,servo_desired[i]);
                }
            }
        }
        msleep(10);
        distance_traveled = (gmpc(right_wheel)/right_wheel_tpc+gmpc(left_wheel)/left_wheel_tpc)/2;
    }
    clear_servos();
}

void d_line_follow(float distance, float speed, int port, char side){
    printf("main\n");
    clear_wheels();
    int prev_right_ticks = 0;
    int prev_left_ticks = 0;
    long double local_x = 0;
    long double local_y = 0;
    long double local_theta = 0;
    if(speed > max_drive_speed){
        speed = max_drive_speed;
    }
    float tps = ((right_wheel_tpc+left_wheel_tpc)/2)*speed;
    float target_speed = (tps + 2)/1.08;
    distance = distance*0.975609756;
    while(local_y < distance){
        long double radius = line_follow_calculate_radius(grey_value-analog(port), maximum_line_follow_radius, minimum_line_follow_radius);
        if(side == 'r'){radius = -radius;}
    	float *speeds = calculate_wheel_speed(radius,target_speed);
    	mav(0,*(speeds+1));//left wheel
    	mav(1,*speeds);//right wheel
        msleep(30);
        float rmt = (gmpc(right_wheel)-prev_right_ticks)/right_wheel_tpc;
        float lmt = (gmpc(left_wheel)-prev_left_ticks)/left_wheel_tpc;
        float arc_length = (rmt+lmt)/2;
        prev_right_ticks = gmpc(right_wheel);
        prev_left_ticks = gmpc(left_wheel);
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
}

void d_line_follow_accel(float distance, float speed, int port, char side){
    printf("up\n");
    clear_wheels();
    int prev_right_ticks = 0;
    int prev_left_ticks = 0;
    long double local_x = 0;
    long double local_y = 0;
    long double local_theta = 0;
    if(speed > max_drive_speed){
        speed = max_drive_speed;
    }
    float tps = ((right_wheel_tpc+left_wheel_tpc)/2)*speed;
    float target_speed = (tps + 2)/1.08;
    distance = distance*0.975609756;
    while(local_y < distance){
        long double radius = line_follow_calculate_radius(grey_value-analog(port), maximum_line_follow_radius, minimum_line_follow_radius);
        if(side == 'r'){radius = -radius;}
    	float *speeds = calculate_wheel_speed(radius,target_speed);
        float mod = local_y/distance;
        if(mod < 0.1){mod = 0.1;}
    	mav(0,*(speeds+1)*mod);//left wheel
    	mav(1,*speeds*mod);//right wheel
        msleep(30);
        float rmt = (gmpc(right_wheel)-prev_right_ticks)/right_wheel_tpc;
        float lmt = (gmpc(left_wheel)-prev_left_ticks)/left_wheel_tpc;
        float arc_length = (rmt+lmt)/2;
        prev_right_ticks = gmpc(right_wheel);
        prev_left_ticks = gmpc(left_wheel);
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
}
void d_line_follow_deccel(float distance, float speed, int port, char side){
    printf("down\n");
    clear_wheels();
    int prev_right_ticks = 0;
    int prev_left_ticks = 0;
    long double local_x = 0;
    long double local_y = 0;
    long double local_theta = 0;
    if(speed > max_drive_speed){
        speed = max_drive_speed;
    }
    float tps = ((right_wheel_tpc+left_wheel_tpc)/2)*speed;
    float target_speed = (tps + 2)/1.08;
    distance = distance*0.975609756;
    while(local_y < distance){
        long double radius = line_follow_calculate_radius(grey_value-analog(port), maximum_line_follow_radius, minimum_line_follow_radius);
        if(side == 'r'){radius = -radius;}
    	float *speeds = calculate_wheel_speed(radius,target_speed);
        float mod = (distance-local_y)/distance;
        if(mod < 0.1){mod = 0.1;}
    	mav(0,*(speeds+1)*mod);//left wheel
    	mav(1,*speeds*mod);//right wheel
        msleep(30);
        float rmt = (gmpc(right_wheel)-prev_right_ticks)/right_wheel_tpc;
        float lmt = (gmpc(left_wheel)-prev_left_ticks)/left_wheel_tpc;
        float arc_length = (rmt+lmt)/2;
        prev_right_ticks = gmpc(right_wheel);
        prev_left_ticks = gmpc(left_wheel);
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
}

void d_right_turn(float degree, float speed, double radius){
    double right_radius = radius-distance_between_wheels/2;
    double left_radius = radius+distance_between_wheels/2;
    double right_wheel_cps = (speed*0.017453) * right_radius;
    double left_wheel_cps = (speed*0.017453) * left_radius;
    double right_wheel_tps = right_wheel_cps * right_wheel_tpc;
    double left_wheel_tps = left_wheel_cps * left_wheel_tpc;
    
    double right_speed = (right_wheel_tps+2)/1.08;
    double left_speed = (left_wheel_tps+2)/1.08;
    double theta = 0;
    clear_wheels();
    while(abs(theta) < degree){
        vfprint(theta);
        mav(right_wheel, right_speed);
        mav(left_wheel, left_speed);
        msleep(5);
        if(abs(right_radius) > abs(left_radius)){
          	theta = ((gmpc(right_wheel)/right_wheel_tpc)/(right_radius))*57.29577951;
        }else{
            theta = ((gmpc(left_wheel)/left_wheel_tpc)/(left_radius))*57.29577951;
        }
         
    }
}
void d_right_turn_accel(float degree, float speed, double radius){
    double right_radius = radius-distance_between_wheels/2;
    double left_radius = radius+distance_between_wheels/2;
    double right_wheel_cps = (speed*0.017453) * right_radius;
    double left_wheel_cps = (speed*0.017453) * left_radius;
    double right_wheel_tps = right_wheel_cps * right_wheel_tpc;
    double left_wheel_tps = left_wheel_cps * left_wheel_tpc;
    
    double right_speed = (right_wheel_tps+2)/1.08;
    double left_speed = (left_wheel_tps+2)/1.08;
    double theta = 0;
    clear_wheels();
    while(fabs(theta) < degree){
        float a = fabs(theta)/degree;
        if(a < 0.1){a = 0.1;}
        mav(right_wheel, right_speed*a);
        mav(left_wheel, left_speed*a);
        msleep(5);
        if(abs(right_radius) > abs(left_radius)){
          	theta = ((gmpc(right_wheel)/right_wheel_tpc)/(right_radius))*57.29577951;
        }else{
            theta = ((gmpc(left_wheel)/left_wheel_tpc)/(left_radius))*57.29577951;
        }
         
    }
}
void d_right_turn_deccel(float degree, float speed, double radius){
    double right_radius = radius-distance_between_wheels/2;
    double left_radius = radius+distance_between_wheels/2;
    double right_wheel_cps = (speed*0.017453) * right_radius;
    double left_wheel_cps = (speed*0.017453) * left_radius;
    double right_wheel_tps = right_wheel_cps * right_wheel_tpc;
    double left_wheel_tps = left_wheel_cps * left_wheel_tpc;
    
    double right_speed = (right_wheel_tps+2)/1.08;
    double left_speed = (left_wheel_tps+2)/1.08;
    double theta = 0;
    clear_wheels();
    while(abs(theta) < degree){
        float a = (degree-fabs(theta))/degree;
        if(a < 0.1){a = 0.1;}
        mav(right_wheel, right_speed*a);
        mav(left_wheel, left_speed*a);
        msleep(5);
        if(abs(right_radius) > abs(left_radius)){
          	theta = ((gmpc(right_wheel)/right_wheel_tpc)/(right_radius))*57.29577951;
        }else{
            theta = ((gmpc(left_wheel)/left_wheel_tpc)/(left_radius))*57.29577951;
        }
         
    }
}
void d_left_turn(float degree, float speed, double radius){   
    double right_radius = radius+distance_between_wheels/2;
    double left_radius = radius-distance_between_wheels/2;
    double right_wheel_cps = (speed*0.017453) * right_radius;
    double left_wheel_cps = (speed*0.017453) * left_radius;
    double right_wheel_tps = right_wheel_cps * right_wheel_tpc;
    double left_wheel_tps = left_wheel_cps * left_wheel_tpc;
    
    double right_speed = (right_wheel_tps+2)/1.08;
    double left_speed = (left_wheel_tps+2)/1.08;
    double theta = 0;
	clear_wheels();
    while(fabs(theta) < degree){
        mav(right_wheel, right_speed);
        mav(left_wheel, left_speed);
        msleep(5);
        if(abs(right_radius) > abs(left_radius)){
          	theta = ((gmpc(right_wheel)/right_wheel_tpc)/(right_radius))*57.29577951;
        }else{
            theta = ((gmpc(left_wheel)/left_wheel_tpc)/(left_radius))*57.29577951;
        }
    }

}
void d_left_turn_accel(float degree, float speed, double radius){   
    double right_radius = radius+distance_between_wheels/2;
    double left_radius = radius-distance_between_wheels/2;
    double right_wheel_cps = (speed*0.017453) * right_radius;
    double left_wheel_cps = (speed*0.017453) * left_radius;
    double right_wheel_tps = right_wheel_cps * right_wheel_tpc;
    double left_wheel_tps = left_wheel_cps * left_wheel_tpc;
    
    double right_speed = (right_wheel_tps+2)/1.08;
    double left_speed = (left_wheel_tps+2)/1.08;
    double theta = 0;
	clear_wheels();
    while(fabs(theta) < degree){
        float a = fabs(theta)/degree;
        if(a < 0.1){a = 0.1;}
        mav(right_wheel, right_speed*a);
        mav(left_wheel, left_speed*a);
        msleep(5);
        if(abs(right_radius) > abs(left_radius)){
          	theta = ((gmpc(right_wheel)/right_wheel_tpc)/(right_radius))*57.29577951;
        }else{
            theta = ((gmpc(left_wheel)/left_wheel_tpc)/(left_radius))*57.29577951;
        }
    }

}
void d_left_turn_deccel(float degree, float speed, double radius){   
    double right_radius = radius+distance_between_wheels/2;
    double left_radius = radius-distance_between_wheels/2;
    double right_wheel_cps = (speed*0.017453) * right_radius;
    double left_wheel_cps = (speed*0.017453) * left_radius;
    double right_wheel_tps = right_wheel_cps * right_wheel_tpc;
    double left_wheel_tps = left_wheel_cps * left_wheel_tpc;
    
    double right_speed = (right_wheel_tps+2)/1.08;
    double left_speed = (left_wheel_tps+2)/1.08;
    double theta = 0;
	clear_wheels();
    while(fabs(theta) < degree){
        float a = (degree-fabs(theta))/degree;
        if(a < 0.1){a = 0.1;}
        mav(right_wheel, right_speed*a);
        mav(left_wheel, left_speed*a);
        msleep(5);
        if(abs(right_radius) > abs(left_radius)){
          	theta = ((gmpc(right_wheel)/right_wheel_tpc)/(right_radius))*57.29577951;
        }else{
            theta = ((gmpc(left_wheel)/left_wheel_tpc)/(left_radius))*57.29577951;
        }
    }

}

/* In progess
void uni_square_up(int speed, int port){
    if(speed > max_drive_speed){
        speed = max_drive_speed;
    }
    float r_tps = right_wheel_tpc * speed;
    float l_tps = left_wheel_tpc *speed;
    float r_speed = (r_tps+2)/1.08;
    float l_speed = (l_tps+2)/1.08;
   	clear_wheels();
    float white_val = analog_avg(port, 10);
    float black_val = white_val;
    float grey_val = 0;
    float distance_traveled;
	int exit = 0;
    float memory[10000] = {0};
    int index = 0;
    float tape_crossed = 0;
    while(exit == 0){
        mav(right_wheel, r_speed);
        mav(left_wheel, l_speed);
        msleep(5);
        distance_traveled = (gmpc(right_wheel)/right_wheel_tpc+gmpc(left_wheel)/left_wheel_tpc)/2;
        float sensor_read = analog_avg(port,5);
        if(sensor_read > black_val){
            black_val = sensor_read;
            memory[index] = sensor_read;
            memory[index+1] = distance_traveled;
            index += 2;
        }
        if(index > 10000){
            printf("overflow!\n");
        }
        grey_val = (white_val+black_val)/2;
        if(black_val > white_val + 70 && sensor_read < grey_val){
            exit = 1;
        }
    }
    int i;
    int closest_index = 0;
    int closest_val = 5000;
    for(i = 0; i < index; i += 2){
        if(abs(memory[i] - grey_val) < closest_val){
            closest_val = abs(memory[i] - grey_val);
            closest_index = i;
        }
    }
    tape_crossed = distance_traveled - memory[closest_index+1];
    vfprint(tape_crossed);
}
*/


////////////////////////////////////////////////////////////////
//CREATE
////////////////////////////////////////////////////////////////

int create_gmpc_r = 0;
int create_gmpc_l = 0;
int gmec_r = -1;
int gmec_l = -1;
int prc = 0;
int plc = 0;

void create_activate(){
    create_in_use = 1;
    create_connect();
    printf("create connected!\n");
    create_full();
    wheel_circumference = 22.61946711;
    distance_between_wheels = 23.5;
    left_wheel_tpr = 505;
    right_wheel_tpr = 505;
    left_wheel_tpc = 505/wheel_circumference;
    right_wheel_tpc = 505/wheel_circumference;
    create_write_byte(142);
    create_write_byte(101);
    char buffer[28];
    create_read_block(buffer, sizeof (buffer));
    gmec_r = (buffer[2] << 8) | (buffer[3] << 0);
    gmec_l = (buffer[0] << 8) | (buffer[1] << 0);
}

void create_gmec_update(){
    create_write_byte(142);
    create_write_byte(101);
    char buffer[28];
    create_read_block(buffer, sizeof (buffer));
    int temp_right = (buffer[2] << 8) | (buffer[3] << 0);
    int temp_left = (buffer[0] << 8) | (buffer[1] << 0);
   	int right_change = temp_right - gmec_r;
    int left_change = temp_left - gmec_l;
    
    //right
    if(abs(right_change) > 500){
        right_change = prc;
        gmec_r += prc;
    }else{
        prc = right_change;
        gmec_r = temp_right;
    }
    create_gmpc_r += right_change;
    
    //left
    if(abs(left_change) > 500){
        left_change = plc;
        gmec_l += plc;
    }else{
        plc = left_change;
        gmec_l = temp_left;
    }
    create_gmpc_l += left_change;
    
    
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


    float x = 0;
    float y = 0;
    float theta = 0;
	float prev_x = 0;
    float d = 0;
    create_gmpc_r = 0;
    create_gmpc_l = 0;
    while(abs(y) < distance){
        float p = x;
        float i = x - prev_x;
        prev_x = x;
        d += x;    
        float speed_mod = (p*200)+(i*100)+(d*80);
        create_drive_direct(speed+speed_mod,speed-speed_mod);
        msleep(15);  	
        create_gmec_update();
        float right_movement = (create_gmpc_r) * (pi * 7.2 / 508.8);
        float left_movement = (create_gmpc_l) * (pi * 7.2 / 508.8);
        float dist_travelled = (right_movement+left_movement)/2;
        create_gmpc_r = 0;
        create_gmpc_l = 0;
        if(right_movement == left_movement){
            if(theta != 0){
                float loc_x = sin(abs(theta))*dist_travelled;
                float loc_y = sin((pi/2)-abs(theta))*dist_travelled;
                y += loc_y;
                if(theta > 0){
                    x += loc_x;
                }else{
                    x -= loc_x;
                }
            }else{
                y += dist_travelled;
            }
        }else{
            double wr = right_movement/left_movement;//wheel ratio
            double radius = ((wr*11.75)+11.75)/(1-wr);
            double alpha = dist_travelled/radius;
            double a = (cos(alpha)*radius)-radius;
            double b = sin(alpha)*radius;
            double c = sqrt((a*a)+(b*b));
            double a_ang = asin(abs(a)/c);
            theta += alpha;
            double y_ang = (pi/2)-alpha+a_ang;
            double local_y = sin(y_ang)*c;
            double local_x = sqrt((c*c)-(local_y*local_y));
            if(right_movement > left_movement){
                x += local_x;
            }else{
                x -= local_x;
            }
            y += local_y;
        }
    }
    printf("y: %f\n", y);
    printf("x: %f\n", x);
    printf("theta: %f\n\n", theta);
}
void r_drive_accel(float distance, int speed){


    float x = 0;
    float y = 0;
    float theta = 0;
	float prev_x = 0;
    float d = 0;
    create_gmpc_r = 0;
    create_gmpc_l = 0;
    float ease_mod = 0;
    while(abs(y) < distance){
        float p = x;
        float i = x - prev_x;
        prev_x = x;
        d += x;
        float speed_mod = (p*200)+(i*100)+(d*80);
        ease_mod = abs(y)/distance;
        if(ease_mod<0.1){ease_mod=0.1;}
        create_drive_direct((speed+speed_mod)*ease_mod,(speed-speed_mod)*ease_mod);
        msleep(15);  	
        create_gmec_update();
        float right_movement = (create_gmpc_r) * (pi * 7.2 / 508.8);
        float left_movement = (create_gmpc_l) * (pi * 7.2 / 508.8);
        float dist_travelled = (right_movement+left_movement)/2;
        create_gmpc_r = 0;
        create_gmpc_l = 0;
        if(right_movement == left_movement){
            if(theta != 0){
                float loc_x = sin(abs(theta))*dist_travelled;
                float loc_y = sin((pi/2)-abs(theta))*dist_travelled;
                y += loc_y;
                if(theta > 0){
                    x += loc_x;
                }else{
                    x -= loc_x;
                }
            }else{
                y += dist_travelled;
            }
        }else{
            double wr = right_movement/left_movement;//wheel ratio
            double radius = ((wr*11.75)+11.75)/(1-wr);
            double alpha = dist_travelled/radius;
            double a = (cos(alpha)*radius)-radius;
            double b = sin(alpha)*radius;
            double c = sqrt((a*a)+(b*b));
            double a_ang = asin(abs(a)/c);
            theta += alpha;
            double y_ang = (pi/2)-alpha+a_ang;
            double local_y = sin(y_ang)*c;
            double local_x = sqrt((c*c)-(local_y*local_y));
            if(right_movement > left_movement){
                x += local_x;
            }else{
                x -= local_x;
            }
            y += local_y;
        }
    }
    printf("y: %f\n", y);
    printf("x: %f\n", x);
    printf("theta: %f\n\n", theta);
}
void r_drive_deccel(float distance, int speed){


    float x = 0;
    float y = 0;
    float theta = 0;
	float prev_x = 0;
    float d = 0;
    create_gmpc_r = 0;
    create_gmpc_l = 0;
    float ease_mod = 0;
    while(abs(y) < distance){
        float p = x;
        float i = x - prev_x;
        prev_x = x;
        d += x;
        float speed_mod = (p*200)+(i*100)+(d*80);
        ease_mod = (distance-abs(y))/distance;
        if(ease_mod<0.1){ease_mod=0.1;}
        create_drive_direct((speed+speed_mod)*ease_mod,(speed-speed_mod)*ease_mod);
        msleep(15);  	
        create_gmec_update();
        float right_movement = (create_gmpc_r) * (pi * 7.2 / 508.8);
        float left_movement = (create_gmpc_l) * (pi * 7.2 / 508.8);
        float dist_travelled = (right_movement+left_movement)/2;
        create_gmpc_r = 0;
        create_gmpc_l = 0;
        if(right_movement == left_movement){
            if(theta != 0){
                float loc_x = sin(abs(theta))*dist_travelled;
                float loc_y = sin((pi/2)-abs(theta))*dist_travelled;
                y += loc_y;
                if(theta > 0){
                    x += loc_x;
                }else{
                    x -= loc_x;
                }
            }else{
                y += dist_travelled;
            }
        }else{
            double wr = right_movement/left_movement;//wheel ratio
            double radius = ((wr*11.75)+11.75)/(1-wr);
            double alpha = dist_travelled/radius;
            double a = (cos(alpha)*radius)-radius;
            double b = sin(alpha)*radius;
            double c = sqrt((a*a)+(b*b));
            double a_ang = asin(abs(a)/c);
            theta += alpha;
            double y_ang = (pi/2)-alpha+a_ang;
            double local_y = sin(y_ang)*c;
            double local_x = sqrt((c*c)-(local_y*local_y));
            if(right_movement > left_movement){
                x += local_x;
            }else{
                x -= local_x;
            }
            y += local_y;
        }
    }
    printf("y: %f\n", y);
    printf("x: %f\n", x);
    printf("theta: %f\n\n", theta);
}

void r_line_follow(float distance, float speed, int port, char side){
    //WARNING NOT YET TESTED
	create_gmpc_r = 0;
    create_gmpc_l = 0;
    
    float dist_travelled = 0;
    while(abs(dist_travelled) < distance){
        float speed_adj = (analog(port)-grey_value) * 0.02;
        float right_speed = (create_speed_filter(speed)-speed_adj);
        float left_speed = (create_speed_filter(speed)+speed_adj);
        create_drive_direct(left_speed,right_speed);
        msleep(15);  	
        create_gmec_update();
        float right_movement = (create_gmpc_r) * (pi * 7.2 / 508.8);
        float left_movement = (create_gmpc_l) * (pi * 7.2 / 508.8);
        create_gmpc_r = 0;
    	create_gmpc_l = 0;
        dist_travelled = (right_movement + left_movement)/2;

    }
}
void r_right_turn(float degree, float speed, double radius){

    double right_radius = radius-distance_between_wheels/2;
    double left_radius = radius+distance_between_wheels/2;
    double right_wheel_cps = (speed*0.017453) * right_radius;
    double left_wheel_cps = (speed*0.017453) * left_radius;
    float right_speed = right_wheel_cps * 10;
    float left_speed = left_wheel_cps * 10;
    float right_arc = 0;
    float left_arc = 0;
    double theta = 0;
    create_gmpc_r = 0;
    create_gmpc_l = 0;
    while(abs(theta) < degree){
        vfprint(theta);
        create_drive_direct(left_speed, right_speed);
        msleep(15);
        create_gmec_update();
        if(abs(right_speed) > abs(left_speed)){
            right_arc += (create_gmpc_r) * (pi * 7.2 / 508.8); 
            theta = ((right_arc)/right_radius)*57.296;
        }else{
            left_arc += (create_gmpc_l) * (pi * 7.2 / 508.8); 
            theta = ((left_arc)/left_radius)*57.296;
        }
        create_gmpc_r = 0;
    	create_gmpc_l = 0;
    } 
}
void r_right_turn_accel(float degree, float speed, double radius){

    double right_radius = radius-distance_between_wheels/2;
    double left_radius = radius+distance_between_wheels/2;
    double right_wheel_cps = (speed*0.017453) * right_radius;
    double left_wheel_cps = (speed*0.017453) * left_radius;
    float right_speed = right_wheel_cps * 10;
    float left_speed = left_wheel_cps * 10;
    float right_arc = 0;
    float left_arc = 0;
    double theta = 0;
    create_gmpc_r = 0;
    create_gmpc_l = 0;
    float speed_mod = 0;
    while(abs(theta) < degree){
        speed_mod = abs(theta)/degree;
        if(speed_mod<0.1){speed_mod=0.1;}
        create_drive_direct(left_speed*speed_mod, right_speed*speed_mod);
        msleep(15);
        create_gmec_update();
        if(abs(right_speed) > abs(left_speed)){
            right_arc += (create_gmpc_r) * (pi * 7.2 / 508.8); 
            theta = ((right_arc)/right_radius)*57.296;
        }else{
            left_arc += (create_gmpc_l) * (pi * 7.2 / 508.8); 
            theta = ((left_arc)/left_radius)*57.296;
        }
        create_gmpc_r = 0;
    	create_gmpc_l = 0;
    } 
}
void r_right_turn_deccel(float degree, float speed, double radius){

    double right_radius = radius-distance_between_wheels/2;
    double left_radius = radius+distance_between_wheels/2;
    double right_wheel_cps = (speed*0.017453) * right_radius;
    double left_wheel_cps = (speed*0.017453) * left_radius;
    float right_speed = right_wheel_cps * 10;
    float left_speed = left_wheel_cps * 10;
    float right_arc = 0;
    float left_arc = 0;
    double theta = 0;
    create_gmpc_r = 0;
    create_gmpc_l = 0;
    float speed_mod = 0;
    while(abs(theta) < degree){
        speed_mod = (degree-abs(theta))/degree;
        if(speed_mod<0.1){speed_mod=0.1;}
        create_drive_direct(left_speed*speed_mod, right_speed*speed_mod);
        msleep(15);
        create_gmec_update();
        if(abs(right_speed) > abs(left_speed)){
            right_arc += (create_gmpc_r) * (pi * 7.2 / 508.8); 
            theta = ((right_arc)/right_radius)*57.296;
        }else{
            left_arc += (create_gmpc_l) * (pi * 7.2 / 508.8); 
            theta = ((left_arc)/left_radius)*57.296;
        }
        create_gmpc_r = 0;
    	create_gmpc_l = 0;
    } 
}
void r_left_turn(float degree, float speed, double radius){
    double right_radius = radius+distance_between_wheels/2;
    double left_radius = radius-distance_between_wheels/2;
    double right_wheel_cps = (speed*0.017453) * right_radius;
    double left_wheel_cps = (speed*0.017453) * left_radius;
    float right_speed = right_wheel_cps * 10;
    float left_speed = left_wheel_cps * 10;
    float right_arc = 0;
    float left_arc = 0;
    double theta = 0;
    create_gmpc_r = 0;
    create_gmpc_l = 0;
    while(abs(theta) < degree){
        create_drive_direct(left_speed, right_speed);
        msleep(15);
        create_gmec_update();
        if(abs(right_speed) > abs(left_speed)){
            right_arc += (create_gmpc_r) * (pi * 7.2 / 508.8); 
            theta = ((right_arc)/right_radius)*57.296;
        }else{
            left_arc += (create_gmpc_l) * (pi * 7.2 / 508.8); 
            theta = ((left_arc)/left_radius)*57.296;
        }
        create_gmpc_r = 0;
    	create_gmpc_l = 0;
    } 
}
void r_left_turn_accel(float degree, float speed, double radius){
    double right_radius = radius+distance_between_wheels/2;
    double left_radius = radius-distance_between_wheels/2;
    double right_wheel_cps = (speed*0.017453) * right_radius;
    double left_wheel_cps = (speed*0.017453) * left_radius;
    float right_speed = right_wheel_cps * 10;
    float left_speed = left_wheel_cps * 10;
    float right_arc = 0;
    float left_arc = 0;
    double theta = 0;
    create_gmpc_r = 0;
    create_gmpc_l = 0;
    float speed_mod = 0;
    while(abs(theta) < degree){
        speed_mod = abs(theta)/degree;
        if(speed_mod<0.1){speed_mod=0.1;}
        create_drive_direct(left_speed*speed_mod, right_speed*speed_mod);
        msleep(15);
        create_gmec_update();
        if(abs(right_speed) > abs(left_speed)){
            right_arc += (create_gmpc_r) * (pi * 7.2 / 508.8); 
            theta = ((right_arc)/right_radius)*57.296;
        }else{
            left_arc += (create_gmpc_l) * (pi * 7.2 / 508.8); 
            theta = ((left_arc)/left_radius)*57.296;
        }
        create_gmpc_r = 0;
    	create_gmpc_l = 0;
    } 
}
void r_left_turn_deccel(float degree, float speed, double radius){
    double right_radius = radius+distance_between_wheels/2;
    double left_radius = radius-distance_between_wheels/2;
    double right_wheel_cps = (speed*0.017453) * right_radius;
    double left_wheel_cps = (speed*0.017453) * left_radius;
    float right_speed = right_wheel_cps * 10;
    float left_speed = left_wheel_cps * 10;
    float right_arc = 0;
    float left_arc = 0;
    double theta = 0;
    create_gmpc_r = 0;
    create_gmpc_l = 0;
    float speed_mod = 0;
    while(abs(theta) < degree){
        speed_mod = (degree-abs(theta))/degree;
        if(speed_mod<0.1){speed_mod=0.1;}
        create_drive_direct(left_speed*speed_mod, right_speed*speed_mod);
        msleep(15);
        create_gmec_update();
        if(abs(right_speed) > abs(left_speed)){
            right_arc += (create_gmpc_r) * (pi * 7.2 / 508.8); 
            theta = ((right_arc)/right_radius)*57.296;
        }else{
            left_arc += (create_gmpc_l) * (pi * 7.2 / 508.8); 
            theta = ((left_arc)/left_radius)*57.296;
        }
        create_gmpc_r = 0;
    	create_gmpc_l = 0;
    } 
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//UNIVERSAL FUNCTIONS

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void drive(float distance, int speed){
    printf("hello\n");
    float ease_window = accel_distance;
    if(ease_window*2 > distance){
        ease_window = distance/2 - 1;
    }
    if(create_in_use == 0){
        d_drive_accel(ease_window,speed);
        d_drive(distance-ease_window*2, speed);
        d_drive_deccel(ease_window,speed);
    }else{
        r_drive_accel(ease_window,speed);
        r_drive(distance-ease_window*2, speed);
        r_drive_deccel(ease_window,speed);
    }
}

void line_follow(float distance, int speed, int port, char side){
    float ease_window = accel_distance;
    if(ease_window*2 > distance){
        ease_window = distance/2 - 1;
    }
    if(create_in_use == 0){
        d_line_follow_accel(ease_window,speed, port, side);
        d_line_follow(distance-ease_window*2, speed, port, side);
        d_line_follow_deccel(ease_window,speed, port, side);
    }else{
        r_line_follow(distance, speed, port, side);
    }
}

void right_turn(float degree, float speed, double radius){
    float ease_window = accel_deg;
    if(ease_window*2 > degree){
        ease_window = degree/2 - 1;
    }
    if(create_in_use == 0){
        d_right_turn_accel(ease_window, speed, radius);
        d_right_turn(degree-ease_window*2, speed, radius);
        d_right_turn_deccel(ease_window, speed, radius);
    }else{
        r_right_turn_accel(ease_window, speed, radius);
        r_right_turn(degree-ease_window*2, speed, radius);
        r_right_turn_deccel(ease_window, speed, radius);
    }
}

void left_turn(float degree, float speed, double radius){
    float ease_window = accel_deg;
    if(ease_window*2 > degree){
        ease_window = degree/2 - 1;
    }
    if(create_in_use == 0){
        d_left_turn_accel(ease_window, speed, radius);
        d_left_turn(degree-ease_window*2, speed, radius);
        d_left_turn_deccel(ease_window, speed, radius);
    }else{
        r_left_turn_accel(ease_window, speed, radius);
        r_left_turn(degree-ease_window*2, speed, radius);
        r_left_turn_deccel(ease_window, speed, radius);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//SEQUENCE SIMPLIFIERS

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
void L_drive(float y, float x, float radius, float d_speed, float t_speed){
    if(radius > distance_between_wheels/2){
    	start_chain(3);
    }
    drive(y-radius, d_speed);
    if(x > 0){
        right_turn(90,t_speed, radius);
    }else if(x < 0){
        left_turn(90,t_speed, radius);
    }
    drive(x-radius, d_speed);
}

void hypo_drive(float y, float x, float speed){
    float hyp = sqrt((y*y)+(x*x));
    float deg = asin(fabs(x)/hyp)*57.29577951;
    if(y < 0){
        deg += 90;
    }
    if(x > 0){
        right_turn(deg,10,0);
    }else{
        left_turn(deg,10,0);
    }
    drive(hyp, speed);
    if(x > 0){
        left_turn(deg,10,0);
    }else{
        right_turn(deg,10,0);
    }
    
}

void drive_skew(float y, float x, float speed){
    float a = x/2;
    float b = y/2;
    float radius = ((b*b)+(a*a))/(fabs(a)*2);
    float deg = asin(fabs(b)/radius)*57.29577951;
    start_chain(2);
    printf("value: %f\n", deg);
    if(x > 0){
        right_turn(deg, speed, radius);
        left_turn(deg,speed,radius);
    }else{
        left_turn(deg, speed,radius);
        right_turn(deg, speed, radius);
    }
}
*/
