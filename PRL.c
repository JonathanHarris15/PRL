/*
PRL v1.8
Creator: Jonathan Harris
Advisors: Zach Zimmerman, Braden McDorman, Nathan Povendo, Qbit
the Plainview Robotics Library is the entire collection of commands used by the Plainview Robotics Team.

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
float accel_distance = 0.4;

int servo_desired[4] = {-1,-1,-1,-1};
int servo_current[4] = {-1,-1,-1,-1};
float servo_time[4] = {-1,-1,-1,-1};
double servo_finish_time[4];

float create_right_speed = 0;
float create_left_speed = 0;

typedef struct{
    double x;
    double y;
    double theta;
}position;

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
void set_accel_window(float distance){
    accel_distance = distance;
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

position calculate_location_change(double right_movement, double left_movement, double theta, float speed){
    position output;
    float dist_travelled = (right_movement+left_movement)/2;
    if(right_movement == left_movement){
        output.theta = 0;
        if(theta != 0){
            float loc_x = sin(abs(theta))*dist_travelled;
            float loc_y = sin((pi/2)-abs(theta))*dist_travelled;
            output.y = loc_y;
            if(theta < 0){output.x = -loc_x;}else{output.x = loc_x;}
        }else{
            output.x = 0;
            if(speed > 0){output.y = dist_travelled;}else{output.y = -dist_travelled;}
        }
    }else{
        double wr = right_movement/left_movement;//wheel ratio
        double radius = ((wr*11.75)+11.75)/(1-wr);
        double alpha = dist_travelled/radius;
        double a = (cos(alpha)*radius)-radius;
        double b = sin(alpha)*radius;
        double c = sqrt((a*a)+(b*b));
        double a_ang = asin(abs(a)/c);
        output.theta = alpha;
        double y_ang = (pi/2)-alpha+a_ang;
        double local_y = sin(y_ang)*c;
        double local_x = sqrt((c*c)-(local_y*local_y));
        if(right_movement > left_movement){output.x = local_x;}else{output.x = -local_x;}
        if(speed > 0){output.y = local_y;}else{output.y = -local_y;}
    }
    return output;
}

float calculate_speed_ramp(float final_dist, float current_dist)
{
    if(current_dist < final_dist*accel_distance){
        float norm_x = current_dist/(final_dist*accel_distance);
        float tween = sin(pi/2*norm_x);
        if(tween < 0.1){
            return 0.1;
        }
        return tween;
    }else if(current_dist > final_dist - (accel_distance*final_dist)){
        float norm_x = (final_dist-current_dist-(accel_distance*final_dist))/(accel_distance*final_dist);
        return cos((pi/2) * norm_x);
    }else{
        return 1;
    }
    
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
typedef struct {
    long left;
    long right;
} encoder_counts_t;

typedef struct {
    double left;
    double right;
}encoder_counts_f;

encoder_counts_t encoder_counts = {0,0};
encoder_counts_t previous_encoder_counts = {0, 0};

const long OVERFLOW_THRESHOLD = 20000L;

void create_activate(){
    create_in_use = 1;
    create_full();
    create_connect();
    printf("create connected!\n");
    wheel_circumference = 22.61946711;
    distance_between_wheels = 23.5;
    left_wheel_tpr = 508;
    right_wheel_tpr = 508;
    left_wheel_tpc = 508/wheel_circumference;
    right_wheel_tpc = 508/wheel_circumference;
}

encoder_counts_t rectify(const encoder_counts_t *const prev, const encoder_counts_t *const next)
{
    encoder_counts_t result = *next;

    if (next->left > OVERFLOW_THRESHOLD && prev->left < -OVERFLOW_THRESHOLD) {
        // Underflow
        result.left += 65536;
    }

    if (next->left < -OVERFLOW_THRESHOLD && prev->left > OVERFLOW_THRESHOLD) {
        // Overflow
        result.left -= 65536;
    }

    if (next->right > OVERFLOW_THRESHOLD && prev->right < -OVERFLOW_THRESHOLD) {
        // Underflow
        result.right += 65536;
    }

    if (next->right < -OVERFLOW_THRESHOLD && prev->right > OVERFLOW_THRESHOLD) {
        // Overflow
        result.right -= 65536;
    }

    return result;
}

encoder_counts_t create_encoder_counts()
{
    create_write_byte(142);
    create_write_byte(101);
    char buffer[28];
    create_read_block(buffer, sizeof (buffer));
    encoder_counts_t retrn = {(buffer[0] << 8) | (buffer[1] << 0), (buffer[2] << 8) | (buffer[3] << 0)};
    return retrn;
}

encoder_counts_f calculate_movement(encoder_counts_t c, float drive_time){
    encoder_counts_t change = c;
    //predicted ticks per second at that speed
    float predicted_right_change = create_right_speed*(784/450);
    float predicted_left_change = create_left_speed*(784/450);

    //moved ticks adjusted for a one second perioud
    float right_ps = (change.right*1000)/drive_time;
    float left_ps = (change.left*1000)/drive_time;

    //vfprint(create_right_speed);
    //vfprint(left_ps - predicted_left_change);

    float rc;
    float lc;

    if(abs(right_ps - predicted_right_change) > 800 || abs(left_ps - predicted_left_change) > 800){
        printf("PACKET WARNING! recovering ------------------------------------------------------\n");
        create_disconnect();
        msleep(5);
        create_connect();
        rc = predicted_right_change * (drive_time/1000);
        lc = predicted_left_change * (drive_time/1000);
    }else{
        rc = change.right;
        lc = change.left;
    }
    encoder_counts_f output = {rc * (pi * 7.2 / 508.8), lc * (pi * 7.2 / 508.8)};
    return output;
}

void create_gmec_update()
{
    const encoder_counts_t next_encoder_counts = create_encoder_counts();
    const encoder_counts_t rectified = rectify(&previous_encoder_counts, &next_encoder_counts);
    encoder_counts.left += rectified.left - previous_encoder_counts.left;
    encoder_counts.right += rectified.right - previous_encoder_counts.right;
    previous_encoder_counts = next_encoder_counts;
}
float create_speed_filter(float num){
    float a = num;
    if(abs(a) < 20){
        a = 20;
        return a*num/abs(num);
    }
    return num;
}
void r_drive(float distance, float speed){
    double x = 0;
    double y = 0;
    double theta = 0;
    double prev_x = 0;
    float d = 0;
    create_gmec_update();
    float speed_mod = 0;
    int drive_time = 15;//<<< this number needs to be changed with the loop msleep
    int loops = 0;
    while(abs(y) < distance*1.05){
        int loop_start_time = seconds();
		
        //DRIVE AND DRIVE_RECORD
        encoder_counts_t start = encoder_counts;
        create_right_speed = speed-speed_mod;
        create_left_speed = speed+speed_mod;

        float speed_ramp = calculate_speed_ramp(distance*1.05, abs(y));
        int rs= create_speed_filter(create_right_speed*speed_ramp);
        int ls = create_speed_filter(create_left_speed*speed_ramp);
        create_drive_direct(ls,rs);
        msleep(5);
        create_gmec_update();
        encoder_counts_t change = {(encoder_counts.right - start.right), (encoder_counts.left - start.left)};
        encoder_counts_f movement = calculate_movement(change, drive_time);
        //CALCULATE POSITIONAL CHANGE
        position locational_change = calculate_location_change(movement.right, movement.left, theta, speed);
        //nan protection
        if(isnan(locational_change.y)){
        	y += y/loops;
        }else{
          	y += locational_change.y;
        }
        if(isnan(locational_change.x)){
        	x += 0;
        }else{
          	x += locational_change.x;
        }
        
        theta += locational_change.theta;
        drive_time = (seconds()-loop_start_time)*1000;
        float p = x;
        float i = x - prev_x;
        prev_x = x;
        d += x;
        speed_mod = (p*200)+(i*10)+(d*50);
        vfprint(speed_mod);
        loops ++;
    }
}

/*
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
*/
void r_right_turn(float degree, float speed, double radius){

    double right_radius = radius-distance_between_wheels/2;
    double left_radius = radius+distance_between_wheels/2;
    double right_wheel_cps = (speed*0.017453) * right_radius;
    double left_wheel_cps = (speed*0.017453) * left_radius;
    float right_speed = right_wheel_cps * 10;//no idea why this is multiplied by 10 but I'm not going to question it
    float left_speed = left_wheel_cps * 10;
    float right_arc = 0;
    float left_arc = 0;
    double theta = 0;
    create_gmec_update();
	int drive_time =15;
    while(abs(theta) < degree){
        int loop_start_time = seconds();
		encoder_counts_t start = encoder_counts;
        
        float speed_ramp = calculate_speed_ramp(degree, abs(theta));
        create_drive_direct(create_speed_filter(left_speed*speed_ramp), create_speed_filter(right_speed*speed_ramp));
        msleep(15);
        
        create_gmec_update();
		encoder_counts_t change = {(encoder_counts.right - start.right), (encoder_counts.left - start.left)};
		encoder_counts_f movement = calculate_movement(change, drive_time);
        
        if(abs(right_speed) > abs(left_speed)){
            right_arc += movement.right; 
            theta = ((right_arc)/right_radius)*57.296;
        }else{
            left_arc += movement.left; 
            theta = ((left_arc)/left_radius)*57.296;
        }
        vfprint(theta);
        drive_time = (seconds()-loop_start_time)*1000;
    } 
}
void r_left_turn(float degree, float speed, double radius){

    double right_radius = radius+distance_between_wheels/2;
    double left_radius = radius-distance_between_wheels/2;
    double right_wheel_cps = (speed*0.017453) * right_radius;
    double left_wheel_cps = (speed*0.017453) * left_radius;
    float right_speed = right_wheel_cps * 10;//no idea why this is multiplied by 10 but I'm not going to question it
    float left_speed = left_wheel_cps * 10;
    float right_arc = 0;
    float left_arc = 0;
    double theta = 0;
    create_gmec_update();
	int drive_time =15;
    while(abs(theta) < degree){
        int loop_start_time = seconds();
		encoder_counts_t start = encoder_counts;
        
        float speed_ramp = calculate_speed_ramp(degree, abs(theta));
        create_drive_direct(create_speed_filter(left_speed*speed_ramp), create_speed_filter(right_speed*speed_ramp));
        msleep(15);
        
        create_gmec_update();
		encoder_counts_t change = {(encoder_counts.right - start.right), (encoder_counts.left - start.left)};
		encoder_counts_f movement = calculate_movement(change, drive_time);
        
        if(abs(right_speed) > abs(left_speed)){
            right_arc += movement.right; 
            theta = ((right_arc)/right_radius)*57.296;
        }else{
            left_arc += movement.left; 
            theta = ((left_arc)/left_radius)*57.296;
        }
        vfprint(theta);
        drive_time = (seconds()-loop_start_time)*1000;
    } 
}

void create_square_up(){
    
    int squarelspeed;
    int squarerspeed;
    //1600
    while(get_create_lcliff_amt() > 2000 || get_create_rcliff_amt() > 2000){
     
        if(get_create_lcliff_amt() > 2000){
            
            squarelspeed = 100;
            
        }else{
         
            squarelspeed = -20;
            
        }
        if(get_create_rcliff_amt() > 2000){
         
            squarerspeed = 100;
            
        }else{
            
            squarerspeed = -20;
            
        }
        create_drive_direct(squarelspeed,squarerspeed);
        
        msleep(15);
       printf("the cliff value for the left wheel is: %d \n",(get_create_rcliff_amt() - get_create_lcliff_amt()));
    }
    create_drive_direct(0,0);
    msleep(10);
}
void create_square_up_back(){
    
    int squarelspeed;
    int squarerspeed;
    
    while(get_create_lcliff_amt() > 1600 || get_create_rcliff_amt() > 1600){
     
        if(get_create_lcliff_amt() > 1600){
            
            squarelspeed = -100;
            
        }else{
         
            squarelspeed = 20;
            
        }
        if(get_create_rcliff_amt() > 1600){
         
            squarerspeed = -100;
            
        }else{
            
            squarerspeed = 20;
            
        }
        create_drive_direct(squarelspeed,squarerspeed);
        
        msleep(15);
       printf("the cliff value for the left wheel is: %d \n",(get_create_rcliff_amt() - get_create_lcliff_amt()));
    }
    create_drive_direct(0,0);
    msleep(10);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//UNIVERSAL FUNCTIONS

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void drive(float distance, int speed){
    if(create_in_use == 0){
        d_drive(distance,speed);
    }else{
        r_drive(distance, speed);
    }
}

void line_follow(float distance, int speed, int port, char side){
    if(create_in_use == 0){
        d_line_follow(distance, speed, port, side);
    }else{
        //r_line_follow(distance, speed, port, side);
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

