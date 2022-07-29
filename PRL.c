#define right_wheel 0
#define left_wheel 1
#define wheel_circumference 21.36283
#define distance_between_wheels 16.25
#define pi 3.14159265359

int right_wheel_tpr = 1000;
int left_wheel_tpr = 1000;
float accel_distance = 3;
float accel_angle = 3;

void set_wheel_ticks(int left, int right){
    left_wheel_tpr = left;
    right_wheel_tpr = right;
}

void set_accel_window_drive(float distance){
    accel_distance = distance;
}

void set_accel_window_turn(float degrees){
    accel_angle = degrees;
}

void drive(float distance, int speed){
    if(speed > 1400){
        printf("RUNTIME ERROR: drive speed must not exceed 1400\nNow stopping the program\n");
        exit(0);
    }
    int right_wheel_target_ticks = (right_wheel_tpr/wheel_circumference) * distance;
    int left_wheel_target_ticks = (left_wheel_tpr/wheel_circumference) * distance;
    float accel_window_ticks = (right_wheel_target_ticks/wheel_circumference) * accel_distance;
    if(accel_window_ticks > right_wheel_target_ticks / 2){
        accel_window_ticks = right_wheel_target_ticks / 2 - 1;
    }
    float tps = ((1.0981818181818 * abs(speed)) - 5.6363636363636);
    float spt = 1/tps;
    float seconds_to_completion = right_wheel_target_ticks * spt;
    int right_wheel_target_speed =  ((right_wheel_target_ticks/seconds_to_completion) - 5.6363636363636)/1.0981818181818;
    int left_wheel_target_speed = ((left_wheel_target_ticks/seconds_to_completion) - 5.6363636363636)/1.0981818181818;
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
        
    }
    mav(right_wheel, 0);
    mav(left_wheel, 0);
    msleep(5);
}
void right(float deg, int speed, float pivot_point){
    if(speed > 1400){
        printf("RUNTIME ERROR: turn speed must not exceed 1400\nNow stopping the program\n");
        exit(0);
    }
    if(pivot_point < 0){
        printf("RUNTIME ERROR: pivot_point cannot be negative!\nNow stopping the program\n");
        exit(0);
    }
    float left_wheel_arc_radius = abs(-distance_between_wheels/2 - pivot_point);
    float right_wheel_arc_radius = abs(distance_between_wheels/2 - pivot_point);
    float left_wheel_arc_length = deg * (pi/180) * left_wheel_arc_radius;
    float right_wheel_arc_length = deg * (pi/180) * right_wheel_arc_radius;
    float right_wheel_target_ticks = (right_wheel_tpr/wheel_circumference) * right_wheel_arc_length;
    float left_wheel_target_ticks = (left_wheel_tpr/wheel_circumference) * left_wheel_arc_length;
    float tps = ((1.0981818181818 * abs(speed)) - 5.6363636363636);
    float spt = 1/tps;
    float seconds_to_completion = left_wheel_target_ticks * spt;
    float right_wheel_target_speed =  ((right_wheel_target_ticks/seconds_to_completion) - 5.6363636363636)/1.0981818181818;
    float left_wheel_target_speed = ((left_wheel_target_ticks/seconds_to_completion) - 5.6363636363636)/1.0981818181818;
    if(pivot_point < distance_between_wheels/2){
        right_wheel_target_speed = -right_wheel_target_speed;
    }
    cmpc(right_wheel);
    cmpc(left_wheel);
    float speed_mod = 0.05;
    while(abs(gmpc(right_wheel)) < right_wheel_target_ticks || abs(gmpc(left_wheel)) < left_wheel_target_ticks){
        float deg_traveled = ((wheel_circumference/left_wheel_tpr)*gmpc(left_wheel))/((pi/180)*left_wheel_arc_radius);
        if(deg_traveled < accel_angle){
            speed_mod = deg_traveled/accel_angle;
            
        }else if(deg_traveled > deg - accel_angle){
            speed_mod = (deg - deg_traveled)/accel_angle;
        }else{
            speed_mod = 1;
        }
        printf("%f\n",deg_traveled);
        if(speed_mod < 0.1){
        	speed_mod = 0.1;
        }
        mav(right_wheel, right_wheel_target_speed*speed_mod);
        mav(left_wheel, left_wheel_target_speed*speed_mod);
    }
    mav(right_wheel,0);
    mav(left_wheel,0);
    msleep(50);
}
void left(){
    
}