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

enum class State{
    OK,
    Err,
};
enum class From{
    rpm,
    swh,
    temp
};
class Rusult{
    public:
    Rusult(){
        state=State::OK;
        value=0;
    }
    State state;
    From from ;
    int8_t value;
};
enum class LED_STATE{
    ON,
    OFF,
    SLOW_FLASHING,
    FAST_FLASHING
};
class LED{
    public:
    LED_STATE state;
    int pin;
    void init(int p){
        this->pin=p;
        this->state=LED_STATE::OFF;
    }    
};
LED led_list[3];

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
    return map(analogRead(num), 576, 606, 30, 37);
}

Rusult error_swh()
{
    int8_t swh_state = (!digitalRead(swh1) << 2) + (digitalRead(swh2) << 1) + digitalRead(swh3);
    Rusult r;
    r.from=From::swh;
    switch (swh_state)
    {
    case 0B011:
        r.state=State::Err;
        r.value=1;
        break;
    case 0B001:
        r.state=State::Err;
        r.value=2;
        break;
    case 0B010:
        r.state=State::Err;
        r.value=3;
        break;
    }
    return r;
}

Rusult error_temp()
{   
    Rusult r;
    r.from=From::temp;
    r.value=(temp(ther1) > 40 || temp(ther2) > 40 || temp(ther3) > 40) << 1 + (temp(ther4) > 40);
    if (r.value!=0) r.state=State::Err;
    return r;
}
bool prev_a = 0;
unsigned long a_startTime = millis();
Rusult error_rpm()
{
    bool a = mainMotor_rpm() < 1000 && digitalRead(swh2) && !digitalRead(swh1);
    Rusult r;
    r.from=From::rpm;
    if (!prev_a && a)
    { // a: ok->error
        a_startTime = millis();
    }
    if ((millis() - a_startTime) > 500 && a)
    { // time out and error
        r.state=State::Err;
        return r;
    }
    prev_a = a;
    return r;
}

Rusult f_error(){
    auto f=error_temp();
    if (f.state==State::Err) return f;
    f= error_rpm();
    if (f.state==State::Err) return f;
    f=error_swh();
    if (f.state==State::Err) return f;
    Rusult r;
    return r;
}

void temp_error_handle(Rusult e){
    if (e.value & 0B10)
    {
        led_list[1].state=LED_STATE::SLOW_FLASHING;
    }
    if (e.value & 0B01)
    {
        led_list[2].state=LED_STATE::SLOW_FLASHING;
    }
}
void swh_error_handle(Rusult e){
    switch (e.value)
    {
    case 1:
        led_list[1].state=LED_STATE::FAST_FLASHING;
        led_list[2].state=LED_STATE::FAST_FLASHING;
        break;
    case 2:
        led_list[2].state=LED_STATE::FAST_FLASHING;
        break;
    case 3:
        led_list[1].state=LED_STATE::FAST_FLASHING;
        break;
    }
}
void rpm_error_handle(Rusult e){
    led_list[1].state=LED_STATE::ON;
}
void setup()
{
    Serial.begin(115200);
    pinMode(led1, OUTPUT);
    pinMode(led2, OUTPUT);
    pinMode(led3, OUTPUT);
    pinMode(relay_general, OUTPUT); // emergency stop, high if detect problem
    pinMode(saw_spdMeter, INPUT);
    pinMode(swh1, INPUT); // reads the state of the saw relay, high if turned on
    pinMode(swh2, INPUT);
    pinMode(swh3, INPUT);
    prev_state = digitalRead(saw_spdMeter); // mainMotor_rpm()
    led_list[0].init(led1);
    led_list[1].init(led2);
    led_list[2].init(led3);
}

void loop()
{
    bool relay_general_out = 0;
    Rusult ff=f_error();
    switch (ff.state)
    {
    case State::Err:
        switch (ff.from){
            case From::rpm:
                rpm_error_handle(ff);
                break;
            case From::swh:
                swh_error_handle(ff);
                break;
            case From::temp:
                temp_error_handle(ff);
                break;
        }
        relay_general_out = 1;
        break;
    case State::OK:
        led_list[0].state=LED_STATE::OFF;
        led_list[1].state=LED_STATE::OFF;
        led_list[2].state=LED_STATE::OFF;
        break;
    }
    for (int i=0;i<3;i++){
        switch (led_list[i].state)
        {
        case LED_STATE::ON:
            digitalWrite(led_list[i].pin, 1);
            break;
        case LED_STATE::OFF:
            digitalWrite(led_list[i].pin, 0);
            break;
        case LED_STATE::SLOW_FLASHING:
            digitalWrite(led_list[i].pin, (millis() % 1000 > 500) ? 0 : 1);
            break;
        case LED_STATE::FAST_FLASHING:
            digitalWrite(led_list[i].pin, (millis() % 500 > 250) ? 0 : 1);
            break;
        }
    }
}
