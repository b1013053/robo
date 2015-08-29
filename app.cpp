/**
 ******************************************************************************
 ** ƒtƒ@ƒCƒ‹–¼ : app.cpp
 **
 ** ŠT—v : 2—Ö“|—§UŽqƒ‰ƒCƒ“ƒgƒŒ[ƒXƒƒ{ƒbƒg‚ÌTOPPERS/HRP2—pC++ƒTƒ“ƒvƒ‹ƒvƒƒOƒ‰ƒ€
 **
 ** ’‹L : sample_cpp (ƒ‰ƒCƒ“ƒgƒŒ[ƒX/K”öƒ‚[ƒ^/’´‰¹”gƒZƒ“ƒT/ƒŠƒ‚[ƒgƒXƒ^[ƒg)
 ******************************************************************************
 **/

#include "ev3api.h"
#include "app.h"
#include "balancer.h"

#include "Motor.h"
#include "GyroSensor.h"
#include "TouchSensor.h"
#include "SonarSensor.h"
#include "Clock.h"

#include "PID.h"

using namespace ev3api;

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

/* Bluetooth */
static int32_t   bt_cmd = 0;      /* BluetoothƒRƒ}ƒ“ƒh 1:ƒŠƒ‚[ƒgƒXƒ^[ƒg */
static FILE     *bt = NULL;      /* Bluetoothƒtƒ@ƒCƒ‹ƒnƒ“ƒhƒ‹ */

/* ‰º‹L‚Ìƒ}ƒNƒ‚ÍŒÂ‘Ì/ŠÂ‹«‚É‡‚í‚¹‚Ä•ÏX‚·‚é•K—v‚ª‚ ‚è‚Ü‚· */
#define GYRO_OFFSET           0  /* ƒWƒƒƒCƒƒZƒ“ƒTƒIƒtƒZƒbƒg’l(Šp‘¬“x0[deg/sec]Žž) */

#define SONAR_ALERT_DISTANCE 30  /* ’´‰¹”gƒZƒ“ƒT‚É‚æ‚éáŠQ•¨ŒŸ’m‹——£[cm] */
#define TAIL_ANGLE_STAND_UP  92  /* Š®‘S’âŽ~Žž‚ÌŠp“x[“x] */
#define TAIL_ANGLE_DRIVE      3  /* ƒoƒ‰ƒ“ƒX‘–sŽž‚ÌŠp“x[“x] */
#define P_GAIN             2.5F  /* Š®‘S’âŽ~—pƒ‚[ƒ^§Œä”ä—áŒW” */
#define PWM_ABS_MAX          60  /* Š®‘S’âŽ~—pƒ‚[ƒ^§ŒäPWMâ‘ÎÅ‘å’l */
//#define DEVICE_NAME     "ET0"  /* Bluetooth–¼ hrp2/target/ev3.h BLUETOOTH_LOCAL_NAME‚ÅÝ’è */
//#define PASS_KEY        "1234" /* ƒpƒXƒL[    hrp2/target/ev3.h BLUETOOTH_PIN_CODE‚ÅÝ’è */
#define CMD_START         '1'    /* ƒŠƒ‚[ƒgƒXƒ^[ƒgƒRƒ}ƒ“ƒh */

/* LCDƒtƒHƒ“ƒgƒTƒCƒY */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)


/* Color　Detection*/
uint8_t LIGHT_WHITE;           
uint8_t LIGHT_BLACK;

char lcd_white[4]={};
char lcd_black[4]={};
char lcd_gyro[4]={};

static int touch_counter = 0;
static int prev_touch = 0;

/* ŠÖ”ƒvƒƒgƒ^ƒCƒvéŒ¾ */
static int32_t sonar_alert(void);
static void tail_control(int32_t angle);

/* ƒIƒuƒWƒFƒNƒg‚Ö‚Ìƒ|ƒCƒ“ƒ^’è‹` */
TouchSensor*    touchSensor;
SonarSensor*    sonarSensor;
ColorSensor*    colorSensor;
GyroSensor*     gyroSensor;
Motor*          leftMotor;
Motor*          rightMotor;
Motor*          tailMotor;
Clock*          clock;

static PID *gPID;

/* ƒƒCƒ“ƒ^ƒXƒN */
void main_task(intptr_t unused)
{
    int8_t forward;      /* ‘OŒãi–½—ß */
    int8_t turn;         /* ù‰ñ–½—ß */
    int8_t pwm_L, pwm_R; /* ¶‰Eƒ‚[ƒ^PWMo—Í */

    /* ŠeƒIƒuƒWƒFƒNƒg‚ð¶¬E‰Šú‰»‚·‚é */
    touchSensor = new TouchSensor(PORT_1);
    colorSensor = new ColorSensor(PORT_3);
    sonarSensor = new SonarSensor(PORT_2);
    gyroSensor  = new GyroSensor(PORT_4);
    leftMotor   = new Motor(PORT_C);
    rightMotor  = new Motor(PORT_B);
    tailMotor   = new Motor(PORT_A);
    clock       = new Clock();
    gPID = new PID(0.65,2.0,0.045);
    //gPID = new PID(0.65,0.0,0.0);


    /* LCD‰æ–Ê•\Ž¦ */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("EV3way-ET sample_cpp", 0, CALIB_FONT_HEIGHT*1);

    /* K”öƒ‚[ƒ^[‚ÌƒŠƒZƒbƒg */
    tailMotor->reset();
    
    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    /* Bluetooth’ÊMƒ^ƒXƒN‚Ì‹N“® */
    act_tsk(BT_TASK);

    ev3_led_set_color(LED_ORANGE); /* ‰Šú‰»Š®—¹’Ê’m */

    /* ƒXƒ^[ƒg‘Ò‹@ */
    while(1)
    {
        tail_control(TAIL_ANGLE_STAND_UP); /* Š®‘S’âŽ~—pŠp“x‚É§Œä */

        if (bt_cmd == 1)
        {
            break; /* ƒŠƒ‚[ƒgƒXƒ^[ƒg */
        }

        if (touchSensor->isPressed() && prev_touch == 0)
        {
        if(touch_counter==0){
         LIGHT_WHITE = colorSensor->getBrightness();  
         sprintf(lcd_white,"WHITE = %d",LIGHT_WHITE);
         ev3_lcd_draw_string(lcd_white, 0, CALIB_FONT_HEIGHT*3);    
         ev3_speaker_play_tone (880,100);
         touch_counter = 1;
       }else if(touch_counter ==  1){
         LIGHT_BLACK =  colorSensor->getBrightness();
         sprintf(lcd_black,"BLACK = %d",LIGHT_BLACK);
         ev3_lcd_draw_string(lcd_black, 0, CALIB_FONT_HEIGHT*4);       
         ev3_speaker_play_tone (880,100);
         touch_counter = 2;
       }else if(touch_counter == 2)
         break; 
 }
      prev_touch = touchSensor->isPressed();

        clock->sleep(10);
    }

    /* ‘–sƒ‚[ƒ^[ƒGƒ“ƒR[ƒ_[ƒŠƒZƒbƒg */
    leftMotor->reset();
    rightMotor->reset();
    //tailMotor->reset();
    
    /* ƒWƒƒƒCƒƒZƒ“ƒT[ƒŠƒZƒbƒg */
    gyroSensor->reset();
    balance_init();

    ev3_led_set_color(LED_GREEN); /* ƒXƒ^[ƒg’Ê’m */

    /**
    * Main loop for the self-balance control algorithm
    */
    while(1)
    {
    	int32_t tail;
        int32_t motor_ang_l, motor_ang_r;
        int32_t gyro, volt;

        if (ev3_button_is_pressed(BACK_BUTTON)) break;

  		//tail = 3;
        //tail_control(TAIL_ANGLE_DRIVE); /* ƒoƒ‰ƒ“ƒX‘–s—pŠp“x‚É§Œä */

        if (sonar_alert() == 1) /* áŠQ•¨ŒŸ’m */
        {
            forward = turn = 0; /* áŠQ•¨‚ðŒŸ’m‚µ‚½‚ç’âŽ~ */
            tail = 70;
            pwm_L = 0;
            pwm_R = 0;
            gyro = 0;
        }
        else
        {
            forward = 50; /* ‘Oi–½—ß */
            tail = 3;
        	
           
        turn = gPID->calcPID(colorSensor->getBrightness(),(LIGHT_WHITE + LIGHT_BLACK) / 2); /* ‰Eù‰ñ–½—ß */
            gyro =gyroSensor->getAnglerVelocity();
        }
        tail_control(tail); /* ƒoƒ‰ƒ“ƒX‘–s—pŠp“x‚É§Œä */

        /* “|—§UŽq§ŒäAPI ‚É“n‚·ƒpƒ‰ƒ[ƒ^‚ðŽæ“¾‚·‚é */
        motor_ang_l = leftMotor->getCount();
        motor_ang_r = rightMotor->getCount();
        //gyro =gyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        /* “|—§UŽq§ŒäAPI‚ðŒÄ‚Ño‚µA“|—§‘–s‚·‚é‚½‚ß‚Ì */
        /* ¶‰Eƒ‚[ƒ^o—Í’l‚ð“¾‚é */
        balance_control(
            (float)forward,
            (float)turn,
            (float)gyro,
            (float)GYRO_OFFSET,
            (float)motor_ang_l,
            (float)motor_ang_r,
            (float)volt,
            (int8_t *)&pwm_L,
            (int8_t *)&pwm_R);

        leftMotor->setPWM(pwm_L);
        rightMotor->setPWM(pwm_R);

        clock->sleep(4); /* 4msecŽüŠú‹N“® */
    }
    leftMotor->reset();
    rightMotor->reset();

    ter_tsk(BT_TASK);
    fclose(bt);

    ext_tsk();
}

//*****************************************************************************
// ŠÖ”–¼ : sonar_alert
// ˆø” : –³‚µ
// •Ô‚è’l : 1(áŠQ•¨‚ ‚è)/0(áŠQ•¨–³‚µ)
// ŠT—v : ’´‰¹”gƒZƒ“ƒT‚É‚æ‚éáŠQ•¨ŒŸ’m
//*****************************************************************************
static int32_t sonar_alert(void)
{
    static uint32_t counter = 0;
    static int32_t alert = 0;

    int32_t distance;

    if (++counter == 40/4) /* –ñ40msecŽüŠú–ˆ‚ÉáŠQ•¨ŒŸ’m  */
    {
        /*
         * ’´‰¹”gƒZƒ“ƒT‚É‚æ‚é‹——£‘ª’èŽüŠú‚ÍA’´‰¹”g‚ÌŒ¸Š“Á«‚ÉˆË‘¶‚µ‚Ü‚·B
         * NXT‚Ìê‡‚ÍA40msecŽüŠú’ö“x‚ªŒoŒ±ã‚ÌÅ’Z‘ª’èŽüŠú‚Å‚·B
         * EV3‚Ìê‡‚ÍA—vŠm”F
         */
        distance = sonarSensor->getDistance();
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
        {
            alert = 1; /* áŠQ•¨‚ðŒŸ’m */

        }
        else
        {
            alert = 0; /* áŠQ•¨–³‚µ */
        }
        counter = 0;
    }

    return alert;
}

//*****************************************************************************
// ŠÖ”–¼ : tail_control
// ˆø” : angle (ƒ‚[ƒ^–Ú•WŠp“x[“x])
// •Ô‚è’l : –³‚µ
// ŠT—v : ‘–s‘ÌŠ®‘S’âŽ~—pƒ‚[ƒ^‚ÌŠp“x§Œä
//*****************************************************************************
static void tail_control(int32_t angle)
{
    float pwm = (float)(angle - tailMotor->getCount()) * P_GAIN; /* ”ä—á§Œä */
    /* PWMo—Í–O˜aˆ— */
    if (pwm > PWM_ABS_MAX)
    {
        pwm = PWM_ABS_MAX;
    }
    else if (pwm < -PWM_ABS_MAX)
    {
        pwm = -PWM_ABS_MAX;
    }

    tailMotor->setPWM(pwm);
}

//*****************************************************************************
// ŠÖ”–¼ : bt_task
// ˆø” : unused
// •Ô‚è’l : ‚È‚µ
// ŠT—v : Bluetooth’ÊM‚É‚æ‚éƒŠƒ‚[ƒgƒXƒ^[ƒgB Tera Term‚È‚Ç‚Ìƒ^[ƒ~ƒiƒ‹ƒ\ƒtƒg‚©‚çA
//       ASCIIƒR[ƒh‚Å1‚ð‘—M‚·‚é‚ÆAƒŠƒ‚[ƒgƒXƒ^[ƒg‚·‚éB
//*****************************************************************************
void bt_task(intptr_t unused)
{
    while(1)
    {
        uint8_t c = fgetc(bt); /* ŽóM */
        switch(c)
        {
        case '1':
            bt_cmd = 1;
            break;
        default:
            break;
        }
        fputc(c, bt); /* ƒGƒR[ƒoƒbƒN */
    }
}
