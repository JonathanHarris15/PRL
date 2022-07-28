#define right_wheel 0
#define left_wheel 1
#define wheel_circumference 21.99115
#define distance_between_wheels 16.3
#define pi 3.14159

int right_wheel_tpr = 1000;
int left_wheel_tpr = 1000;

void set_wheel_ticks(int left, int right){
    left_wheel_tpr = left;
    right_wheel_tpr = right;
}

void drive(float distance, int speed){
    if(speed > 1400){
        printf("RUNTIME ERROR: drive speed must not exceed 1400\n Now stopping the program");
        shut_down_in(1);
        return;
    }
    int right_wheel_target_ticks = (right_wheel_tpr/wheel_circumference) * distance;
    int left_wheel_target_ticks = (left_wheel_tpr/wheel_circumference) * distance;
    float tps = ((1.0981818181818 * speed) - 5.6363636363636);
    float spt = 1/tps;
    float seconds_to_completion = right_wheel_target_ticks * spt;
    int right_wheel_target_speed =  ((right_wheel_target_ticks/seconds_to_completion) - 5.6363636363636)/1.0981818181818;
    int left_wheel_target_speed = ((left_wheel_target_ticks/seconds_to_completion) - 5.6363636363636)/1.0981818181818;
    printf("right: %d\n",right_wheel_target_speed);
    printf("left: %d\n",left_wheel_target_speed);
    cmpc(right_wheel);
    cmpc(left_wheel);
    while(gmpc(right_wheel) < right_wheel_target_ticks || gmpc(left_wheel) < left_wheel_target_ticks){
        mav(right_wheel, right_wheel_target_speed);
        mav(left_wheel, left_wheel_target_speed);
    }
    mav(right_wheel, 0);
    mav(left_wheel, 0);
    msleep(50);
}
void right(){

}
void left(){
    
}