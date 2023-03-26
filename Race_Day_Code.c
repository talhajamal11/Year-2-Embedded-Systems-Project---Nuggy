#include "mbed.h"
#include "QEI.h"
#include "C12832.h"
#include "PID.h"
#define WHEEL_RADIUS 0.041
#define GEAR_RATIO 18.75
#define CPR 256

class encoder : public QEI{
    protected:
        int start_time_us, end_time_us;
        int time_us;
        int start_pulse, end_pulse;
        int pulse;
        int pulse_counter;
        float wheel_velocity;
        float pps;
        Timer t;
        Timeout to;
    public:
        encoder(PinName p1, PinName p2, PinName p3, int count): QEI (p1,p2,p3,count)
        {time_us = 0; pulse = 0; pulse_counter=0; wheel_velocity=0;}
        
        //function that returns number of rotations per second
        float RPS (float time_sec, int pulse){                      
            //returns Number of Rotations per second
            return (pulse/time_sec*512);
        }

        void start_counter(void)
        {   start_time_us = t.read_us();
            start_pulse = getPulses();
            t.start();
            to.attach(callback(this, &encoder::stop_counter),0.01);
        }
        
        void stop_counter(void)
        {   
            end_time_us = t.read_us();
            end_pulse = getPulses();
            speed_calculator();
        }
        
        float pps_calculation (float time_sec, int pulse){                      
            //returns Number of Rotations per second
            return (pulse/time_sec);
        }
            
        void speed_calculator (void) {

            time_us = end_time_us - start_time_us;
            pulse = end_pulse - start_pulse;
            pulse_counter = pulse_counter + pulse; 
            wheel_velocity = (RPS(time_us/1000000.0, pulse) * (2*3.142*WHEEL_RADIUS));
            pps = pps_calculation(time_us/1000000.0, pulse);
            reset();
            
            start_counter();
        }
        
        float distance (void)
        {
            return (pulse_counter/512)*(2*3.142*WHEEL_RADIUS);
        } 
        
        float return_speed(void)
        {    
            return wheel_velocity;
        }

        float return_pps(void)
        {    
            return pps;
        }
        
        int total_pulse_count(){
            return pulse_counter;
        }
        
        void reset_count(void)
        {   
            pulse_counter=0;
        }
};
class Wheel_Control{
    private:
    PwmOut left_wheel_pwm;
    PwmOut right_wheel_pwm;
    
    Timeout angular_change;                     // Timer to perform 90 degrees and 180 turns
    
    float prop_speed;                           // Ratio of the speed given to each wheel: from -100 to 100 (with 0 <=> same speed to both wheels) 
    
    float stop_speed;
    float buggy_speed;                          // Speed of the buggy in m/s
    float buggy_speed_pwm;                      // Corresponding pwm
    float offset;
    
    float left_wheel_pwm_calc;
    float right_wheel_pwm_calc;
    
    DigitalOut polarity_left;                   // Mode of wheels
    DigitalOut polarity_right;
    
    DigitalOut direction_left;                  // Direction of wheels
    DigitalOut direction_right;
    
    DigitalOut enable;                          // Enable motor drive board
    
    public:
    Wheel_Control(PinName left_pin, PinName right_pin, PinName mode_left,PinName mode_right,PinName direction_l,PinName direction_r, PinName en): 
    
    left_wheel_pwm (left_pin), right_wheel_pwm (right_pin), polarity_left (mode_left),polarity_right (mode_right), direction_left(direction_l), direction_right(direction_r), enable (en)
    
    {   prop_speed= 0; buggy_speed = 0.5; stop_speed = 1.0;
        
        polarity_left = 0;                      // Set to unipolar
        polarity_right = 0;
        enable =1;
        offset= 0.005;
        
        left_wheel_pwm.period(0.00001f);               
        right_wheel_pwm.period(0.00001f);
    }
    
    void write_speed (float speed)
    {
           buggy_speed = speed;
    }
    
    void read_pwm_left (float pwm)
    {
        left_wheel_pwm.write(pwm);
    }
    
    void read_pwm_right (float pwm)
    {
        right_wheel_pwm.write(pwm);
    }
    float return_buggy_speed()
    {    
        return buggy_speed;
    }   
    
    void direction(int left, int right){
        direction_left = left ;
        direction_right = right;
    }
    
    void forward(float pwm_l, float pwm_r)
    {
            direction_left = 1;
            direction_right = 1;
            left_wheel_pwm.write(pwm_l);
            right_wheel_pwm.write(pwm_r);   
    }
    
    void stop(void)
    {
            direction_left = 1;
            direction_right = 1;
            left_wheel_pwm.write(stop_speed);
            right_wheel_pwm.write(stop_speed);
    }
    
    void turn_right(float pwm_l, float pwm_r)
    {   direction_left = 1;
        direction_right = 1;
        left_wheel_pwm.write(pwm_l);
        right_wheel_pwm.write(pwm_r);
    }
    
    void turn_left(float pwm_l, float pwm_r)
    {   direction_left = 1;
        direction_right = 1;
        left_wheel_pwm.write(pwm_l);
        right_wheel_pwm.write(pwm_r);
    }
    
};
class Analogue_sensor {
  private:
  AnalogIn sensor;
  DigitalOut emitter;
  float value;
  Timeout turn_off;
  
  public:
  Analogue_sensor(PinName p1, PinName p2): sensor(p1), emitter(p2){
      emitter = 0 ;
      value = 0.0;}

    void emitter_on(void)
    {   
        emitter = 1;
        //turn_off.attach(callback(this, &Analogue_sensor::reading_value), 0.05);
    }

    
    void emitter_off(){
        emitter = 0;
    }
    
    void reading_value()
    {
        value = sensor.read();
        emitter = 0;
        emitter_on();
    }

    float reading()
    {
        return sensor.read();
    }
    

};

float find_max_in_array (float list[], int size){
    float max = 0;
    for (int n = 0; n < size; n++){
        if (list[n] > max){
            max = list[n];
        }
    }
    return max;
}

float max_sensor_readings[6];

float * calibration(){
    int calibration_points = 1000;
    Analogue_sensor sensor_1 (PA_4, PA_13);
    Analogue_sensor sensor_2 (PC_3, PB_8);
    Analogue_sensor sensor_3 (PC_2, PC_6);
    Analogue_sensor sensor_4 (PB_0, PB_9);
    Analogue_sensor sensor_5 (PC_1, PC_8);
    Analogue_sensor sensor_6 (PC_0, PC_9);
    sensor_1.emitter_on();
    sensor_2.emitter_on();
    sensor_3.emitter_on();
    sensor_4.emitter_on();
    sensor_5.emitter_on();
    sensor_6.emitter_on();
    float S1_list[calibration_points], S2_list[calibration_points], S3_list[calibration_points], S4_list[calibration_points], S5_list[calibration_points], S6_list[calibration_points];
    Timer calibration_time;
    calibration_time.start();
    for (int n = 0;calibration_time.read() < 3;n++){
        S1_list[n] = sensor_1.reading();
        S4_list[n] = sensor_4.reading();
        S2_list[n] = sensor_2.reading();
        S5_list[n] = sensor_5.reading();
        S3_list[n] = sensor_3.reading();
        S6_list[n] = sensor_6.reading();
        printf("%f %f %f %f %f %f\n", S1_list[n], S2_list[n], S3_list[n], S4_list[n], S5_list[n], S6_list[n]);
    }
    float calibration_factors[6] = {find_max_in_array (S1_list, calibration_points), find_max_in_array (S2_list, calibration_points), find_max_in_array (S3_list, calibration_points), find_max_in_array (S4_list, calibration_points), find_max_in_array (S5_list, calibration_points), find_max_in_array (S6_list, calibration_points)};
    printf("%f %f %f %f %f %f\n", calibration_factors[0],calibration_factors[1],calibration_factors[2],calibration_factors[3],calibration_factors[4],calibration_factors[5]);
    return calibration_factors;
};


int main() {
    // Bluetooth initialising
    Serial hm10(PA_11, PA_12); //UART6 TX,RX
    char c = 'S'; //the character we want to receive
    hm10.baud(9600);
    
    Timer NoTrack, StartUp;

    // LCD inititalising
    C12832 lcd(D11, D13, D12, D7, D10); // object named lcd 
    Wheel_Control buggy(PB_7,PA_15,PC_11,PC_10,PD_2,PC_12,PB_2);
    buggy.stop();
    // encoder intialising
    encoder left(PB_14, PB_13, NC, 256);
    encoder right(PB_15, PB_12, NC, 256);
    
    //Kc, Ti, Td, interval (PID initialising)
    PID controllerRight(1.2 ,1.2 ,0.00012 ,0.01);             //
    PID controllerLeft(1.2 ,1.2 ,0.00012 ,0.01);              //0.7, 0.7, 0.007 // 1.2 ,1.2 ,0.00012 , 0.1 <- values worked
    
    PID xPID(1.6, 0.00001, 0.0000001, 0.01);                        // works at low battery at 1.2, 0.5, 0.0001, 0.1       PID xPID(0.8, 0.99999, 0.00001, 0.1); PID xPID(0.6, 1, 0.00001, 0.1); works at TD3_Tuege Speed(0.6, 1, 0.00001, 0.01); PERFECT_ONE TD3_Tuege Speed(0.8, 1, 0.00001, 0.01);
    
    //1st Round P -> 2.45 -> critical oscillations
    float coRight = 1.0, coLeft = 1.0; //--> output pwm dyty cyle
    float coxPID = 600;
    
    // Pulse per second
    int target = 1000;

    // Timer for encoder
    left.start_counter();
    right.start_counter();
    
    //Analog input from 0.0 to 3.3V
    controllerRight.setInputLimits(0.0, 4169.0); //--> max pps    4169
    controllerLeft.setInputLimits(0.0, 4159); //--> max pps       4159
    xPID.setInputLimits(0.0, 160.0);
  
    //Pwm output from 0.0 to 1.0
    controllerRight.setOutputLimits(0.0, 1.0); // pwm is fine
    controllerLeft.setOutputLimits(0.0, 1.0); // pwm is fine
    xPID.setOutputLimits(0.0, 2*target);
  
    //If there's a bias.
    //controller.setBias(0.3);
    //xPID.setBias(-100);
    
    controllerRight.setMode(1);
    controllerLeft.setMode(1);
    xPID.setMode(1);
    
    //We want the process variable to be 1.7V
    controllerRight.setSetPoint(target); // needs ot be accessible for the rest 
    controllerLeft.setSetPoint(target); // needs ot be accessible for the rest 
    xPID.setSetPoint(600);
    
    // Initialising sensors
    Analogue_sensor sensor_1 (PA_4, PA_13);
    Analogue_sensor sensor_2 (PC_3, PB_8);
    Analogue_sensor sensor_3 (PC_2, PC_6);
    Analogue_sensor sensor_4 (PB_0, PB_9);
    Analogue_sensor sensor_5 (PC_1, PC_8);
    Analogue_sensor sensor_6 (PC_0, PC_9);
    
    //float * sensor_scaling_factors = calibration();
    
    sensor_1.emitter_on();
    sensor_2.emitter_on();
    sensor_3.emitter_on();
    sensor_4.emitter_on();
    sensor_5.emitter_on();
    sensor_6.emitter_on();
    
    StartUp.start();
    
    int current_position = 0;
    float S1, S2, S3, S4, S5, S6;
    float threshold = 0.2;
    bool start = true;
    float offTrackTimeLimit = 0.15;
    printf("test\n");
    while(1) {
        
        S1 = sensor_1.reading();
        S2 = sensor_2.reading();
        S3 = sensor_3.reading();
        S4 = sensor_4.reading();
        S5 = sensor_5.reading();
        S6 = sensor_6.reading();
        float average = (S1 + S2 + S3 + S4 + S5 + S6)/6;
        if (S1 > threshold || S2 > threshold || S3 > threshold ||  S4 > threshold || S5 > threshold || S6 > threshold)
//        if (S1 > 2*average || S2 > 2*average || S3 > 1.5*average ||  S4 > 1.4*average || S5 > 1.7*average || S6 > 3*average)
            {
        current_position =(-(S1*241.36992)-(S2*61.25)-(S3*26.325)+(S4*28)+(S5*74.97)+(S6*137.142)); //current_position = (-(S1*200)-(S2*70)-(S3*20)+(S4*20)+(S5*70)+(S6*200));
        //NoTrack.stop();
//        NoTrack.reset();
        }
//        else if (NoTrack.read() == 0){
//            NoTrack.start();
//        }
//        else if (NoTrack.read()> offTrackTimeLimit){
//            buggy.stop();
//            wait(1);
//        }
//        printf("%f %f %f %f %f %f \n", S1*241.36992, S2*61.25, S3*26.325, S4*28, S6*74.97, S5*137.142);    // current_position  %d
        printf("%d \n", current_position);
        xPID.setSetPoint(80);
        xPID.setProcessValue(current_position+80);
        coxPID = xPID.compute()-target;
        controllerRight.setSetPoint(target - coxPID); // needs ot be accessible for the rest 
        controllerLeft.setSetPoint(target + coxPID);
        //printf("%f\n", coxPID);
        
        controllerRight.setProcessValue(right.return_pps());
        controllerLeft.setProcessValue(left.return_pps());
        
        //printf("%f %f\n", left.return_pps(), right.return_pps());
        
        coRight = controllerRight.compute();
        coLeft = controllerLeft.compute();
        
        //printf("%f %f\n", coLeft, coRight);
               
        if (coRight < 0){coRight=0;}
        if (coLeft < 0){coLeft=0;}
        //Wait for another loop calculation.
        if((start == true)&&(NoTrack.read()<= offTrackTimeLimit) && (S1 > threshold || S2 > threshold || S3 > threshold ||  S4 > threshold || S5 > threshold || S6 > threshold)){
            buggy.forward(1.0-coLeft, 1.0-coRight);
        }
        else if(start == false){
            buggy.forward(1.0,1.0);
        }
        wait(0.02);
        
        
    
        if(hm10.readable())
        {
            c = hm10.getc(); //read a single character
            if(c == 'r'){
                c = 'A';
                buggy.stop();
                wait(0.5);
                left.reset_count();
                right.reset_count();
                buggy.forward(0.6,1.0);
                buggy.direction(0,1);
                while (left.total_pulse_count() > -548)
                    {

                    }
                buggy.forward(1.0,0.6);
                wait(0.1);
                while (not(sensor_1.reading() > threshold || sensor_2.reading() > threshold || sensor_3.reading() > threshold ||  sensor_4.reading() > threshold || sensor_5.reading() > threshold || sensor_6.reading() > threshold))
                    {
                    }
                right.reset_count();
                /*while (right.total_pulse_count() < 100)
                    {

                    }*/
                wait(0.1);
                buggy.stop();
                buggy.direction(1,1);

            }
            else if(c == 'l'){
                c = 'A';
                target=1000;
                xPID.setTunings(1.2, 0.001, 0.0000001);
                xPID.setOutputLimits(0.0, 2*target);
            }
            else if(c == 'm'){
                c = 'A';
                target=2000;
                xPID.setTunings(0.8, 1, 0.00001);
                xPID.setOutputLimits(0.0, 2*target);
            }
            else if(c == 'h'){
                c = 'A';
                target=3000;
                xPID.setTunings(1, 1, 0.00001);
                xPID.setOutputLimits(0.0, 2*target);
            }
            else if(c== 's'){
                c = 'A';
                start = not start;
            }
        }
    }
}



