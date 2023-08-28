// ARDUINO NANO PINOUT: https://i0.wp.com/www.teachmemicro.com/wp-content/uploads/2019/06/Arduino-Nano-pinout-4.jpg
// PINOUTS
// outputs
#define led1 7 // small red LED
#define led2 9
#define led3 10
#define relay_general 2 // emergency stop
// inputs
#define ther1 A4       // saw_esc // thermal sensitive resister
#define ther2 A5       // saw_motor
#define ther3 A6       // middle_motor
#define ther4 A7       // paperContainer_motor
#define saw_spdMeter 6 // photo interrupt sensor, used to read the rpm of the saw
#define swh1 11
#define swh2 8
#define swh3 12
// ERROR TYPES
// #define error_rpm mainMotor_rpm() < 1000 && digitalRead(swh2)

unsigned long prev_time = 0;
bool prev_state, present_state;
float rpm = 0;
int8_t counter = 0;
float mainMotor_rpm()
{
    present_state = digitalRead(6);
    if (present_state != prev_state)
    { // changed
        if (++counter >= 2)
        {
            // time_elapsed = micros() - prev_time;
            rpm = (float)60000000 / (micros() - prev_time);
            prev_time = micros();
            counter = 0;
        }
        prev_state = present_state;
        // Serial.println("hi");
    }
    else if (micros() - prev_time > 500000)
    { // over 0.5 sec no change
        rpm = 0;
        prev_time = micros();
    }

    if (millis() % 100 <= 5)
    {
        // Serial.println(rpm);
        return rpm;
    }
}

int temp(uint8_t num)
{
    // int8_t pins[4] = {19U, 18U, 17U, 16U};
    // Serial.println(digitalRead(6));
    return map(analogRead(num), 576, 606, 30, 37);
}

int8_t error_swh()
{
    int8_t swh_state = ((!digitalRead(swh1)) << 2) + (digitalRead(swh2) << 1) + digitalRead(swh3);
    if (swh_state == 0B011)
        return 1;
    if (swh_state == 0B001)
        return 2;
    if (swh_state == 0B010)
        return 3;
    if (swh_state == 0B111)
        return 4;
    if (swh_state == 0B101)
        return 5;
    if (swh_state == 0B110)
        return 6;
    if (swh_state == 0B000)
        return 10;
    return 0;
}

#define d 0.95
int prev_temp[4] = {0, 0, 0, 0};
int present_temp[4];
int8_t error_temp()
{
    present_temp[0] = (1 - d) * temp(ther1) + d * prev_temp[0];
    present_temp[1] = (1 - d) * temp(ther2) + d * prev_temp[1];
    present_temp[2] = (1 - d) * temp(ther3) + d * prev_temp[2];
    present_temp[3] = (1 - d) * temp(ther4) + d * prev_temp[3];
    prev_temp[0] = present_temp[0];
    prev_temp[1] = present_temp[1];
    prev_temp[2] = present_temp[2];
    prev_temp[3] = present_temp[3];

    // return (((temp(ther1) > 40) || (temp(ther2) > 40) || (temp(ther3) > 40)) << 1) + (temp(ther4) > 60);
    return (((present_temp[0] > 40) || (present_temp[1] > 40) || (present_temp[2] > 40)) << 1) + (present_temp[3] > 60);
}

bool prev_a = 0;
unsigned long a_startTime = millis();
bool error_rpm()
{
    bool a = mainMotor_rpm() < 1000 && digitalRead(swh2) && !digitalRead(swh1);
    if (!prev_a && a)
    { // a: ok->error
        a_startTime = millis();
    }
    if ((millis() - a_startTime) > 500 && a)
    { // time out and error
        return 1;
    }
    prev_a = a;
    return 0;
}

void setup()
{
    // Serial.begin(115200);
    // pinMode(led1, OUTPUT);
    // pinMode(led2, OUTPUT);
    // pinMode(led3, OUTPUT);
    // pinMode(relay_general, OUTPUT); // emergency stop, high if detect problem
    // pinMode(saw_spdMeter, INPUT);
    // pinMode(swh1, INPUT); // reads the state of the saw relay, high if turned on
    // pinMode(swh2, INPUT);
    // pinMode(swh3, INPUT);
    DDRD |= 0B10000100;
    DDRB |= 0B000110;                       // set PD2, PD7, PB1, PB2 as output
    prev_state = digitalRead(saw_spdMeter); // mainMotor_rpm()
}

bool swh_reset = 0;
void loop()
{
    // no need to initialize
    // digitalWrite(led1, 0);
    // digitalWrite(led2, 0);
    // digitalWrite(led3, 0);
    bool relay_general_out = 0, led1_out = 0, led2_out = 0, led3_out = 0;
    bool blink_fast = (millis() % 300 > 150) ? 0 : 1;
    bool blink_slow = (millis() % 1000 > 500) ? 0 : 1;

    //  errors feedback
    if (error_rpm())
    {
        // digitalWrite(relay_general, 1);
        relay_general_out = 1;
        // digitalWrite(led2, 1);
        led2_out = 1;
    }

    int8_t error_swhB = error_swh();
    if (((error_swhB == 1) || (error_swhB == 2) || (error_swhB == 3)) || !swh_reset)
    {
        swh_reset = 0;
        relay_general_out = 1;
        switch (error_swhB)
        {
        case 1:
        case 4:
            // digitalWrite(led2, (millis() % 500 > 250) ? 0 : 1);
            // digitalWrite(led3, (millis() % 500 > 250) ? 0 : 1);
            led2_out = blink_fast;
            led3_out = blink_fast;
            break;
        case 2:
        case 5:
            // digitalWrite(led3, (millis() % 500 > 250) ? 0 : 1);
            led3_out = blink_fast;
            break;
        case 3:
        case 6:
            // digitalWrite(led2, (millis() % 500 > 250) ? 0 : 1);
            led2_out = blink_fast;
            break;
        }
    }
    if (error_swhB == 10)
        swh_reset = 1;

    int8_t error_tempB = error_temp();
    if (error_tempB)
    {
        relay_general_out = 1;
        if ((error_tempB & 0B10) == 0B10)
        {
            // digitalWrite(led2, (millis() % 1000 > 500) ? 0 : 1);
            led2_out = blink_slow;
        }
        if ((error_tempB & 0B01) == 0B01)
        {
            // digitalWrite(led3, (millis() % 1000 > 500) ? 0 : 1);
            led3_out = blink_slow;
        }
    }
    // digitalWrite(relay_general, relay_general_out);
    // output the result
    PORTD &= 0B01111011;
    PORTD |= (led1_out << 7) + (relay_general_out << 2);
    PORTB &= 0B111001;
    PORTB |= (led3_out << 2) + (led2_out << 1);
}
