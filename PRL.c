/*
PRL v1.8.5
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
float pivot = 23.5/2;

float max_drive_speed = 100;

float grey_value = 3200;
float black_and_white_diff = 100;
float minimum_line_follow_radius = 30;
float maximum_line_follow_radius = 1000;
float used_tape_width = 5.08;

float accel_distance = 20;
float accel_deg = 15/57.296;

int servo_desired[4] = {-1,-1,-1,-1};
int servo_current[4] = {-1,-1,-1,-1};
float servo_time[4] = {-1,-1,-1,-1};
double servo_finish_time[4];

float create_right_speed = 0;
float create_left_speed = 0;

int create_lowest_speed = 40;

typedef struct{
    double x;
    double y;
    double theta;
}position;

int chain_size = -1;
int chain = -1;

////////////////////////////////////////////////////////////////
//HELPER FUNCTIONS
////////////////////////////////////////////////////////////////

void viprint(int val){
    printf("value: %d\n",val);
}
void vfprint(float val){
    printf("%f\n",val);
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

void light(int port){
    printf("press the button when light is ON");
    while(!a_button()){
        msleep(3);
    }
    float on = analog_avg(port,5);
   	vfprint(on);
    msleep(1000);
    printf("press the button when light is OFF");
    while(!a_button()){
        msleep(3);
    }
    float off = analog_avg(port,5);
  	float gry = (on+off)/2;
    printf("Grey Value is %f. now waiting for light\n", gry);
    while(analog(port) > gry){
        msleep(3);
    }
    

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

position calculate_location_change(double right_movement, double left_movement, double theta){
    position output;
    float d = (left_movement+right_movement)/2;
    output.theta = (right_movement - left_movement)/distance_between_wheels;
    output.x = d*cos(theta);
    output.y = d*sin(theta);
    return output;
}
void start_chain(int size){
    chain_size = size;
    chain = size;
}
float calculate_speed_ramp(float final_dist, float current_dist){ 
    if(current_dist < accel_distance && (chain_size == chain || chain < 1)){
        float norm_x = current_dist/(accel_distance);
        float tween = sin(pi/2*norm_x);
        if(tween < 0.2){
            return 0.2;
        }
        return tween;
    }else if(current_dist > final_dist - accel_distance && chain < 2){
        float norm_x = (final_dist-current_dist-(accel_distance))/(accel_distance); 
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
    //printf("max drive speed is %f\n",max_drive_speed);
}
void spin_motor(int port, int ticks, int speed){
    cmpc(port);
    while(abs(gmpc(port)) < ticks){
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
    while(fabs(distance_traveled) < distance){
        mav(right_wheel, calculate_speed_ramp(distance,fabs(distance_traveled))*r_speed);
        mav(left_wheel, calculate_speed_ramp(distance,fabs(distance_traveled))*l_speed);
        msleep(10);
        for(i = 0; i < 4; i ++){
            if(servo_desired[i] != -1){ 
                if(seconds() < servo_finish_time[i] && servo_current[i] != servo_desired[i]){
                    x = (seconds()-servo_start_time)/(servo_finish_time[i]-servo_start_time);  
                    float total_dist = servo_desired[i]-servo_current[i];
                    float sqt = x*x;
                    multiplier = sqt/(2.0 * (sqt-x)+1.0);
                    //printf("%f\n",servo_current[i] + (total_dist*multiplier));
                    set_servo_position(i,servo_current[i] + (total_dist*multiplier));
                }
            }
        }
        distance_traveled = (gmpc(right_wheel)/right_wheel_tpc+gmpc(left_wheel)/left_wheel_tpc)/2;
    }
}

void d_line_follow(float distance, float speed, int port, char side){
    //printf("main\n");
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
        mav(0,*(speeds+1)*calculate_speed_ramp(distance,local_y));//left wheel
        mav(1,*speeds*calculate_speed_ramp(distance,local_y));//right wheel
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
    while(fabs(theta) < degree){
        mav(right_wheel, calculate_speed_ramp(degree,fabs(theta))*right_speed);
        mav(left_wheel, calculate_speed_ramp(degree,fabs(theta))*left_speed);
        msleep(5);
        if(fabs(right_radius) > fabs(left_radius)){
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
        mav(right_wheel, calculate_speed_ramp(degree,fabs(theta))*right_speed);
        mav(left_wheel, calculate_speed_ramp(degree,fabs(theta))*left_speed);
        msleep(5);
        if(fabs(right_radius) > fabs(left_radius)){
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
    create_connect();
    create_full();
    printf("create connected!\n");
    wheel_circumference = 22.61946711;
    distance_between_wheels = 23.5;
    left_wheel_tpr = 508;
    right_wheel_tpr = 508;
    left_wheel_tpc = 508/wheel_circumference;
    right_wheel_tpc = 508/wheel_circumference;
}
void create_adjust_wheel_tpr(float l,float r){
	left_wheel_tpr = l;
	right_wheel_tpr = r;
	left_wheel_tpc = l/wheel_circumference;
    right_wheel_tpc = r/wheel_circumference;
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
    create_clear_serial_buffer();
    create_write_byte(142);
    create_write_byte(101);
    char buffer[28];
    create_read_block(buffer, sizeof (buffer));
    encoder_counts_t retrn = {(buffer[0] << 8) | (buffer[1] << 0), (buffer[2] << 8) | (buffer[3] << 0)};
    return retrn;
}

encoder_counts_f calculate_movement(encoder_counts_t c){
    encoder_counts_t change = c;
    float rc = change.right;
    float lc = change.left;
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
    if(num == 0){
        return 0;
    }
    float a = num;

    if(fabs(a) < create_lowest_speed){
        a = create_lowest_speed;
        return a*num/fabs(num);
    }
    return num;
}

void r_drive(float distance, float speed){
    double x = 0, y = 0, theta = pi/2, prev_x = 0, speed_mod = 0;
    create_gmec_update();
    encoder_counts_t start = encoder_counts;
    set_create_distance(0);
    while(fabs(y) < distance){
        float create_right_speed, create_left_speed;
        //DRIVE AND DRIVE_RECORD
        if(speed > 0){
            create_right_speed = speed+speed_mod;
      		create_left_speed = speed-speed_mod;
        }else{
        	create_right_speed = speed-speed_mod;
      		create_left_speed = speed+speed_mod;
        }
        float speed_ramp = calculate_speed_ramp(distance,fabs(y));
        create_drive_direct(create_speed_filter(create_left_speed*speed_ramp), create_speed_filter(create_right_speed*speed_ramp));
        //create_drive_direct(create_left_speed,create_right_speed);
        msleep(15);
        create_gmec_update();
        encoder_counts_t change = {encoder_counts.right - start.right, encoder_counts.left - start.left};
        start = encoder_counts;
        encoder_counts_f movement = calculate_movement(change);
        position locational_change = calculate_location_change(movement.right, movement.left, theta);
        y += locational_change.y;
        x += locational_change.x;       
        theta += locational_change.theta;
        printf("%lf\n",x);
        
        ////PID CONTROLL////
        float p = x;
        float d = x - prev_x;
        prev_x = x;
        speed_mod = ((p*50)+(d*400))*speed_ramp;
    }
}

void r_line_follow(float distance, float speed, int port, char side){
    float dist_travelled = 0;
    encoder_counts_t start = encoder_counts;
    while(fabs(dist_travelled) < distance){
        float speed_adj = (analog(port)-grey_value) * 0.065;
        if(side == 'r'){
            speed_adj = -speed_adj;
        }
     	  create_right_speed = (-(create_speed_filter(speed)-speed_adj));
        create_left_speed = (-(create_speed_filter(speed)+speed_adj));
        create_drive_direct(create_left_speed,create_right_speed);
        msleep(15);  	
        create_gmec_update();
        encoder_counts_t change = {(encoder_counts.right - start.right), (encoder_counts.left - start.left)};
        start = encoder_counts;
        encoder_counts_f movement = calculate_movement(change);
        dist_travelled += (movement.right + movement.left)/2;
    }
}

void r_right_turn(float degree, float speed, double radius){
    float radians = (degree)/57.296;
    create_gmec_update();
    encoder_counts_t start = encoder_counts;
    float right_radius = radius - (distance_between_wheels/2); 
    float left_radius = radius + (distance_between_wheels/2);
    float right_arc = right_radius*radians;
    float left_arc = left_radius*radians;
    float right_speed, left_speed;
    if(radius == 0){
        right_speed = -speed;
        left_speed = speed;
    }else if(radius > 0){
        left_speed = speed;
        right_speed = (right_arc*speed)/left_arc;
    }else{
        right_speed = -speed;
        left_speed = -(left_arc*speed)/right_arc;
    }
    //float right_speed = (right_arc*10)/speed;
    //float left_speed = (left_arc*10)/speed;
    float dist = fabs(left_arc) + fabs(right_arc);
    float loop_time = 15;
    float moving_speed = 0;
    float turned_dist = 0;
    int exit = 0;
    while(exit == 0){
        float loop_start = seconds();
        float mod = calculate_speed_ramp(dist, fabs(turned_dist));//fabs(right_dist) + fabs(left_dist));
        create_drive_direct(create_speed_filter(mod*left_speed),create_speed_filter(mod*right_speed));
        msleep(15);
        create_gmec_update();
        encoder_counts_t change = {encoder_counts.right - start.right, encoder_counts.left - start.left};
        start = encoder_counts;
        encoder_counts_f movement = calculate_movement(change);
        if(movement.left > 0){
            turned_dist += movement.left + fabs(movement.right);
        }else{
            turned_dist += movement.left - fabs(movement.right);
        }
        vfprint(turned_dist);
        moving_speed = movement.right + movement.left;
        loop_time = (seconds() - loop_start)*1000;
        if(dist - fabs(turned_dist) < 0.1 && moving_speed/loop_time < 0.0002){
            exit = 1;
        }
    }
}
void r_left_turn(float degree, float speed, double radius){
    float radians = (degree)/57.296;
    create_gmec_update();
    encoder_counts_t start = encoder_counts;
    float right_radius = radius + (distance_between_wheels/2); 
    float left_radius = radius - (distance_between_wheels/2);
    float right_arc = right_radius*radians;
    float left_arc = left_radius*radians;
    float right_speed, left_speed;
    if(radius == 0){
        right_speed = speed;
        left_speed = -speed;
    }else if(radius > 0){
        right_speed = speed;
        left_speed = (left_arc*speed)/left_arc;
    }else{
        left_speed = -speed;
        right_speed = -(right_arc*speed)/left_arc;
    }
    //float right_speed = (right_arc*10)/speed;
    //float left_speed = (left_arc*10)/speed;
    float dist = fabs(left_arc) + fabs(right_arc);
    float loop_time = 15;
    float moving_speed = 0;
    float turned_dist = 0;
    int exit = 0;
    while(exit == 0){
        float loop_start = seconds();
        float mod = calculate_speed_ramp(dist, fabs(turned_dist));//fabs(right_dist) + fabs(left_dist));
        vfprint(mod);
        create_drive_direct(create_speed_filter(mod*left_speed),create_speed_filter(mod*right_speed));
        msleep(15);
        create_gmec_update();
        encoder_counts_t change = {encoder_counts.right - start.right, encoder_counts.left - start.left};
        start = encoder_counts;
        encoder_counts_f movement = calculate_movement(change);
        if(movement.right > 0){
            turned_dist += movement.right + fabs(movement.left);
        }else{
            turned_dist += movement.right - fabs(movement.left);
        }
        
        moving_speed = movement.right + movement.left;
        loop_time = (seconds() - loop_start)*1000;
        if(dist - fabs(turned_dist) < 0.1 && moving_speed/loop_time < 0.0002){
            exit = 1;
        }
    }
}


void create_square_up(int speed){
    int squarelspeed, squarerspeed;
    //1600
    while(get_create_lcliff_amt() > 2300 || get_create_rcliff_amt() > 2300){
     
        if(get_create_lcliff_amt() > 2300){
            
            squarelspeed = speed;
            
        }else{
         
            squarelspeed = -20 * speed/fabs(speed);
            
        }
        if(get_create_rcliff_amt() > 2300){
         
            squarerspeed = speed;
            
        }else{
            
            squarerspeed = -20 * speed/fabs(speed);
            
        }
        create_drive_direct(squarelspeed,squarerspeed);
        printf("%d\n",get_create_lcliff_amt());
        msleep(15);

    }
    create_drive_direct(0,0);
    msleep(10);
}
void create_square_up_close(int speed){    
    int squarelspeed, squarerspeed;
    while(get_create_lfcliff_amt() > 2000 || get_create_rfcliff_amt() > 2000){
     
        if(get_create_lfcliff_amt() > 2000){
            
            squarelspeed = speed;
            
        }else{
         
            squarelspeed = -20 * speed/fabs(speed);
            
        }
        if(get_create_rfcliff_amt() > 2000){
         
            squarerspeed = speed;
            
        }else{
            
            squarerspeed = -20 * speed/fabs(speed);
            
        }
        create_drive_direct(squarelspeed,squarerspeed);
        
        msleep(15);
    }
    create_drive_direct(0,0);
    msleep(10);
}
/*
float points[] = {30,30,
                  30,50,
                  100,50,
                  0,0};

waypoint_drive(points,200,5, sizeof(points)/sizeof(points[0]));
*/
void waypoint_drive(float param[],int speed, float precision, int size){
    int target_point = 0;
    double x = 0, y = 0, theta = pi/2, speed_mod = 0, theta_error = 0;
    create_gmec_update();
    encoder_counts_t start = encoder_counts;
    int exit = 0;
    float last_error = 0;
    float i = 0;
    while(exit == 0){
        float tpx = param[target_point*2];
        float tpy = param[(target_point*2)+1];
        float xe = tpx - x;
        float ye = tpy - y;
        //theta error logic
        if(xe == 0){//ON THE Y AXIS

        }else if(ye == 0){//ON THE X AXIS

        }else if(xe > 0 && ye > 0){//QUADRANT ONE
            theta_error = theta - atan(ye/xe);
        }else if(xe < 0){//QUADRANT TWO AND THREE
            theta_error = theta - (pi+atan(ye/xe));
        }else{//QUADRANT FOUR
            theta_error = theta - ((2*pi)+atan(ye/xe));
        } 
        if(fabs(theta_error) > pi){
            if(theta_error < 0){
                theta_error += 2*pi;
            }else{
                theta_error -= 2*pi;
            }

        }
        //PID CONTROLLER
        double p = theta_error*5.5;    
        printf("%f\n",p);
        float d = -(theta_error - last_error)*0.3;
        i += theta_error;
        //printf("%f %f %f\n", p, i, d);
        speed_mod = (p+d+(i*0.05))*speed;
        vfprint(speed_mod);
        if(fabs(speed_mod) > speed/0.5){
            speed_mod = (speed/0.5)*(speed_mod/fabs(speed_mod));
        }
        create_drive_direct(speed+speed_mod,speed-speed_mod);
        msleep(15); 
        create_gmec_update();
        encoder_counts_t change = {encoder_counts.right - start.right, encoder_counts.left - start.left};
        start = encoder_counts;
        encoder_counts_f movement = calculate_movement(change);
        position locational_change = calculate_location_change(movement.right, movement.left, theta);
        y += locational_change.y;
        x += locational_change.x;       
        theta += locational_change.theta;
        if(theta > 2*pi){
            theta -= 2*pi;
        }
        if(theta < 0){
            theta += 2*pi;
        }
        float distance_error = sqrt(pow((tpx-x),2)+pow((tpy-y),2));
        if(distance_error < precision){
            if(target_point == (size/2) - 1){
                exit = 1;
            }
            else{
                target_point ++;
                i = 0;
            }
        }
        //code for displaying to the graphics window
        //float zoom = 2;
        //graphics_circle_fill((798/2)+(x*zoom),(798/4)-(y*zoom),zoom,0,255,0);
        //graphics_update();
    }
    create_drive_direct(0,0);
    msleep(50);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//UNIVERSAL FUNCTIONS

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void drive(float distance, int speed){
    //printf("////////////drive/////////////////\n");
    if(create_in_use == 0){
        d_drive(distance,speed);
    }else{
        r_drive(distance, speed);
    }
    chain --;
}

void line_follow(float distance, int speed, int port, char side){
    if(create_in_use == 0){
        d_line_follow(distance, speed, port, side);
    }else{
        r_line_follow(distance, speed, port, side);
    }
    chain --;
}

void right_turn(float degree, float speed, double radius){
    //printf("////////////right turn/////////////////\n");
    if(create_in_use == 0){
        d_right_turn(degree, speed, radius);
    }else{
        r_right_turn(degree, speed, radius);
    }
    chain --;
}

void left_turn(float degree, float speed, double radius){
    if(create_in_use == 0){
        d_left_turn(degree, speed, radius);
    }else{
        r_left_turn(degree, speed, radius);
    }
    chain --;
}

void square(int speed){
    if(create_in_use == 0){
    }else{
        create_square_up(speed);
    }
    chain --;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//SEQUENCE SIMPLIFIERS

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
        left_turn(deg,200,0);
    }
    drive(hyp, speed);
    if(x > 0){
        left_turn(deg,200,0);
    }else{
        right_turn(deg,200,0);
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
