float distance_between_wheels = 16.25;
float grey_value = 1700;
float minimum_line_follow_radius = 50;
float maximum_line_follow_radius = 1000;

//NOTE: white means positive error and black means negative error FOR LEFT SIDE LINE FOLLOW
float line_follow_calculate_radius(int error_value, float max_radius, float min_radius){
    float error_modifier = 400;
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

float * calculate_wheel_speed(float radius, float speed){
    static float speeds[2];
    float theta = speed/radius;
    speeds[0] = theta * (radius-(distance_between_wheels*0.5));
    speeds[1] = theta * (radius+(distance_between_wheels*0.5));
    return speeds;
}

void line_follow(float distance, float speed, int port){
    float tape_distance;
    while(tape_distance < distance){
        float radius = line_follow_calculate_radius(grey_value-analog(port), maximum_line_follow_radius, minimum_line_follow_radius);
    	float *speeds = calculate_wheel_speed(radius,speed);
    	mav(0,*(speeds+1));//left wheel
    	mav(1,*speeds);
    	tape_distance += 1;
    }
    mav(0,0);
    mav(1,0);
    msleep(50);
}
