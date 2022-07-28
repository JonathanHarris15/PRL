#define right_wheel 0
#define left_wheel 1
#define wheel_circumference 21.36283
#define distance_between_wheels 16.3
#define pi 3.14159

int right_wheel_tpr = 1000;
int left_wheel_tpr = 1000;
float accel_distance = 3;

void set_wheel_ticks(int left, int right){
    left_wheel_tpr = left;
    right_wheel_tpr = right;
}

void set_accel_window(float distance){
    accel_distance = distance;
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
    float tps = ((1.0981818181818 * speed) - 5.6363636363636);
    float spt = 1/tps;
    float seconds_to_completion = right_wheel_target_ticks * spt;
    int right_wheel_target_speed =  ((right_wheel_target_ticks/seconds_to_completion) - 5.6363636363636)/1.0981818181818;
    int left_wheel_target_speed = ((left_wheel_target_ticks/seconds_to_completion) - 5.6363636363636)/1.0981818181818;
    cmpc(right_wheel);
    cmpc(left_wheel);
    float speed_mod = 0.05;
    while(gmpc(right_wheel) < right_wheel_target_ticks || gmpc(left_wheel) < left_wheel_target_ticks){
        if(gmpc(right_wheel) < accel_window_ticks){
            speed_mod = gmpc(right_wheel)/accel_window_ticks;
            if(speed_mod < 0.1){
                speed_mod = 0.1;
            }
            mav(right_wheel, right_wheel_target_speed * speed_mod);
            mav(left_wheel, left_wheel_target_speed * speed_mod);
        }else if (gmpc(right_wheel) > right_wheel_target_ticks - accel_window_ticks){
            speed_mod = (right_wheel_target_ticks - gmpc(right_wheel))/accel_window_ticks;
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
    msleep(50);
}
void right(){

}
void left(){
    
}