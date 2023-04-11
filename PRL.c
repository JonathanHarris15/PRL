/*
PRL v1.9
Creator: Jonathan Harris
Advisors: Zach Zimmerman, Braden McDorman, Nathan Povendo, Qbit
the Plainview Robotics Library is the entire collection of commands used by the Plainview Robotics Team.
SETUP:
The setup of this library for use is extremely easy and only requires a couple of steps.
1. The first thing that needs to be done is adding the following two lines ABOVE the include for this library in the main.c file:
    #include <stdlib.h>
    #include <math.h>
2. If you are using the library for a demo bot (anything but the create), then you need to call the function set_wheel_ticks(float l, float r)
	at the beginning of your code even before wait for light or anything else. The parameters should be the ticks it takes for each wheel to turn the bot 90 degrees
    from a pivot turn. use spin_motor(int ticks, int speed, int port) to spin the motor (speed 500 is recomended).
3. If you are using a create bot then disreguard step 2 and use the function create_activate() instead of the usual create_connect() function. This will 
	notify the library that a create is in use so that it will use the correct functions
USE:
	Everything SHOULD be ordered with parameters distance>speed>port>misc

here is an example of a waypoint drive call. At the start it assumes the robot is at location (0,0) and is pointed towards pi/2

float points[] = {30,30,
                  30,50,
                  100,50,
                  0,0};
waypoint_drive(points,200,5, sizeof(points)/sizeof(points[0]));

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
float pivot = 11.43/2;

float max_drive_speed = 100;

float grey_value = 2500;
float black_and_white_diff = 100;
float minimum_line_follow_radius = 30;
float maximum_line_follow_radius = 1000;
float used_tape_width = 5.08;

float accel_distance = 5;

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

typedef struct {
    long left;
    long right;
} encoder_counts_t;

typedef struct {
    double left;
    double right;
}encoder_counts_f;

int chain_size = -1;
int chain = -1;

////////////////////////////////////////////////////////////////
//HELPER FUNCTIONS
////////////////////////////////////////////////////////////////

void viprint(int val){
    printf("%d\n",val);
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

//custom wait for light function in case wombat isnt updated
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

//This takes in wheel movement and the current theta position of the robot and returns the change in pose
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

//Chaining is a little weird right now so I will need to fix that
float calculate_speed_ramp(float final_dist, float current_dist){ 
    if(current_dist < accel_distance && (chain_size == chain || chain < 1)){
        float norm_x = current_dist/(accel_distance);
        vfprint(current_dist);
        float tween = sin(pi/2*norm_x);
        if(tween < 0.2){
            return 0.2;
        }
        return tween;
    }else if(current_dist > final_dist - accel_distance && chain < 2){
        float norm_x = (final_dist-current_dist-(accel_distance))/(accel_distance); 
        float tween = cos((pi/2) * norm_x);
        if(tween < 0.2){
            return 0.2;
        }
        return tween;
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
    left_wheel_tpc = left/17.954202;
    right_wheel_tpc = right/17.954202;
    if(right_wheel_tpc > left_wheel_tpc){
        max_drive_speed = 1402/right_wheel_tpc;
    }else{
        max_drive_speed = 1402/left_wheel_tpc;
    }
    //printf("max drive speed is %f\n",max_drive_speed);
}

encoder_counts_f calculate_movement_demo(encoder_counts_t c){
    encoder_counts_t change = c;
    float rc = change.right;
    float lc = change.left;
    encoder_counts_f output = {rc/right_wheel_tpc, lc/left_wheel_tpc};
    return output;
}
void spin_motor(int ticks, int speed, int port){
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
    //int i = 0;
    //float multiplier = 0;
    //double x = 0;
    while(fabs(distance_traveled) < distance){
        mav(right_wheel, calculate_speed_ramp(distance,fabs(distance_traveled))*r_speed);
        mav(left_wheel, calculate_speed_ramp(distance,fabs(distance_traveled))*l_speed);
        msleep(10);
        encoder_counts_t change = {gmpc(left_wheel),gmpc(right_wheel)};
        encoder_counts_f movement = calculate_movement_demo(change);
        distance_traveled = (movement.right+movement.left)/2;
    }
}

//PARAM TO CHANGE:
//	the speed_mod value that is multiplied into error might change depending on black and white diffirence on the table
//	max_angle prevents overturn in case you are too far away from the line, this can be adjusted for usecases
void d_line_follow(float distance, float speed, int port, char side){
    clear_wheels();
    long double x = 0;
    long double y = 0;
    long double theta = pi/2;
    if(speed > max_drive_speed){
        speed = max_drive_speed;
    }
    float max_angle = 30;
    float tps = ((right_wheel_tpc+left_wheel_tpc)/2)*speed;
    float target_speed = (tps + 2)/1.08;
    while(y < distance){
       	float error = grey_value - analog(port);
        float speed_mod = error*0.7;
       	if(side == 'l'){
            speed_mod *= -1;
        }
        if(fabs(theta-(pi/2))*57.296 > max_angle){
        	speed_mod = 0;
    	}
        float ramp_mod = calculate_speed_ramp(distance, y);
        mav(right_wheel,(target_speed + speed_mod)*ramp_mod);//left wheel
        mav(left_wheel,(target_speed - speed_mod)*ramp_mod);//right wheel
        msleep(30);
        encoder_counts_t change = {gmpc(left_wheel),gmpc(right_wheel)};
        clear_wheels();
        encoder_counts_f movement = calculate_movement_demo(change);
        position locational_change = calculate_location_change(movement.right, movement.left, theta);
        y += locational_change.y;
        x += locational_change.x;       
        theta += locational_change.theta;
        vfprint(y);
    }
}

//degrees and speed should alway be positive but radius can be reversed to get diffirent functionality
void d_right_turn(float degree, float speed, double radius){
    float radians = (degree)/57.296;
	clear_wheels();
    float right_radius = radius - (distance_between_wheels/2); 
    float left_radius = radius + (distance_between_wheels/2);
    float right_arc = right_radius*radians*right_wheel_tpc;
    float left_arc = left_radius*radians*left_wheel_tpc;
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
	float mod = 0;
    while(abs(gmpc(right_wheel)) < fabs(right_arc) || abs(gmpc(left_wheel)) < fabs(left_arc)){
        mod = calculate_speed_ramp(fabs(left_arc)/left_wheel_tpc, abs(gmpc(left_wheel))/left_wheel_tpc);
        if(fabs(right_arc) > fabs(left_arc)){
            mod = calculate_speed_ramp(fabs(right_arc)/right_wheel_tpc, fabs(gmpc(right_wheel))/right_wheel_tpc);
        }
        mav(right_wheel, mod*right_speed);
        mav(left_wheel, mod*left_speed);
        msleep(15);
    }
}

void d_left_turn(float degree, float speed, double radius){   
    float radians = (degree)/57.296;
	clear_wheels();
    float right_radius = radius + (distance_between_wheels/2); 
    float left_radius = radius - (distance_between_wheels/2);
    float right_arc = right_radius*radians*right_wheel_tpc;
    float left_arc = left_radius*radians*left_wheel_tpc;
    float right_speed, left_speed;
    if(radius == 0){
        right_speed = speed;
        left_speed = -speed;
    }else if(radius > 0){
        right_speed = speed;
        left_speed = (left_arc*speed)/right_arc;
    }else{
        left_speed = -speed;
        right_speed = -(right_arc*speed)/left_arc;
    }
	float mod = 0;
    while(abs(gmpc(right_wheel)) < fabs(right_arc) || abs(gmpc(left_wheel)) < fabs(left_arc)){
        mod = calculate_speed_ramp(fabs(left_arc)/left_wheel_tpc, abs(gmpc(left_wheel))/left_wheel_tpc);
        if(fabs(right_arc) > fabs(left_arc)){
            mod = calculate_speed_ramp(fabs(right_arc)/right_wheel_tpc, fabs(gmpc(right_wheel))/right_wheel_tpc);
        }
        mav(right_wheel, mod*right_speed);
        mav(left_wheel, mod*left_speed);
        msleep(15);
    }

}


void d_waypoint_drive(float param[],int speed, float precision, int size){
    int target_point = 0;
    double x = 0, y = 0, theta = pi/2, speed_mod = 0, theta_error = 0;
    clear_wheels();
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
        double p = theta_error*10;    
        float d = -(theta_error - last_error)*0;
        i += theta_error;
        printf("%f %f %f\n", p, i*0, d);
        speed_mod = (p+d+(i*0))*speed;
        if(fabs(speed_mod) > speed/0.5){
            speed_mod = (speed/0.5)*(speed_mod/fabs(speed_mod));
        }
        mav(right_wheel, speed-speed_mod);
        mav(left_wheel, speed+speed_mod);
        msleep(15); 
        encoder_counts_t change = {gmpc(right_wheel), gmpc(left_wheel)};
        clear_wheels();
        encoder_counts_f movement = calculate_movement_demo(change);
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
        float zoom = 2;
        graphics_circle_fill((798/2)+(x*zoom),(798/4)-(y*zoom),zoom,0,255,0);
        graphics_update();
    }
    mav(right_wheel, 0);
    mav(left_wheel, 0);
    msleep(50);
}
////////////////////////////////////////////////////////////////
//CREATE
////////////////////////////////////////////////////////////////

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
    pivot = distance_between_wheels/2;
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

//The PID should be properly tuned for all creates but further testing will need to be done
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

//This line follow does not take in locational data to protect from infinite looping against a wall
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

//uses the close sensors instead of the wide ones
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

void create_waypoint_drive(float param[],int speed, float precision, int size){
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
void waypoint_drive(float param[],int speed, float precision, int size){
    if(create_in_use == 0){
        d_waypoint_drive(param,speed,precision,size);
    }else{
        create_waypoint_drive(param,speed,precision,size);
    }
    chain --;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//SEQUENCE SIMPLIFIERS

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//these should work I think
//speeds might be a little wonky
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
