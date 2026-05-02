/*
 * @file        main.c 
 * @author      Vishal Shukla
 * @module      Embedded Systems
 * @project     Vehicle Tilt Detection System
 * @date        22 March 2026
 * 
 * @brief       Tilt detection with Rotary Encoder + Watchdog
 */

#include <eeng1030_lib.h>
#include <stdio.h>
#include <errno.h>
#include <sys/unistd.h>
#include <string.h>

#include "display.h"
#include "i2c.h"

/*---------- PIN DEFINITIONS ----------*/
#define ENC_CLK     3   // PB3 - Rotary Encoder CLK
#define ENC_DT      1   // PB1 - Rotary Encoder DT
#define LED_PIN     5   // PB5 - Alarm LED
#define BTN_PIN     4   // PB4 - Reset Button

/*---------- FUNCTION DECLARATIONS ----------*/
void delay(uint32_t dly);
void delay_ms(uint32_t dly);
void delay_us(uint32_t dly);
void initSerial(uint32_t baudrate);
void eputc(char c);
void initADC(void);
int readADC(int chan);
void initTimer2(void);
void buzzer_on(int volume);
void buzzer_off(void);
void initEncoder(void);
int readEncoder(void);
void initWatchdog(void);
void feedWatchdog(void);

/*---------- COLORS ----------*/
#define COLOR_RED     RGBToWord(255,0,0)
#define COLOR_GREEN   RGBToWord(0,255,0)
#define COLOR_WHITE   RGBToWord(255,255,255)
#define COLOR_BLACK   RGBToWord(0,0,0)
#define COLOR_GREY    RGBToWord(25,25,25)
#define COLOR_YELLOW  RGBToWord(255,255,0)
#define COLOR_CYAN    RGBToWord(0,255,255)

/*---------- GLOBALS ----------*/
int16_t x_accel, y_accel, z_accel;
int32_t X_g, Y_g, Z_g;
uint16_t response;
char debug_str[30];

int pot_value = 0;
int duty_cycle = 0;
int tilt_threshold = 500;
int alarm_active = 0;
int last_threshold = 500;

volatile uint32_t systick_ms = 0;
int watchdog_feeds = 0;

/*---------- DELAY FUNCTIONS ----------*/
void delay(uint32_t dly)
{
    while(dly--);
}

void delay_us(uint32_t dly)
{
    // Approximate delay for 1us at 80MHz
    for(uint32_t i = 0; i < dly * 80; i++) {
        __asm("nop");
    }
}

void delay_ms(uint32_t dly)
{
    uint32_t start = systick_ms;
    while((systick_ms - start) < dly);
}

/*---------- ROTARY ENCODER ----------*/
void initEncoder(void)
{
    RCC->AHB2ENR |= (1 << 1);  // GPIOB clock
    
    pinMode(GPIOB, ENC_CLK, 0);
    pinMode(GPIOB, ENC_DT, 0);
    
    // Enable pull-ups
    GPIOB->PUPDR |= (1 << (ENC_CLK * 2));
    GPIOB->PUPDR |= (1 << (ENC_DT * 2));
}

int readEncoder(void)
{
    static int last_clk = 0;
    int clk = (GPIOB->IDR >> ENC_CLK) & 1;
    int dt = (GPIOB->IDR >> ENC_DT) & 1;
    static int result = 0;
    
    if(last_clk == 1 && clk == 0) {
        if(dt == 1) {
            result = 1;   // Clockwise
        } else {
            result = -1;  // Counter-clockwise
        }
    } else {
        result = 0;
    }
    last_clk = clk;
    return result;
}

/*---------- BUZZER ----------*/
void buzzer_on(int volume)
{
    int pwm_value = (volume * TIM2->ARR) / 100;
    if(pwm_value > TIM2->ARR) pwm_value = TIM2->ARR;
    if(pwm_value < 0) pwm_value = 0;
    TIM2->CCR4 = pwm_value;
}

void buzzer_off(void)
{
    TIM2->CCR4 = 0;
}

void initTimer2(void)
{
    RCC->APB1ENR1 |= (1 << 0);
    RCC->AHB2ENR |= (1 << 0);
    pinMode(GPIOA, 3, 2);
    selectAlternateFunction(GPIOA, 3, 1);
    TIM2->CR1 = 0;
    TIM2->CCMR2 = (6 << 12) | (1 << 11);
    TIM2->CCER |= (1 << 12);
    TIM2->PSC = 79;
    TIM2->ARR = 1000 - 1;
    TIM2->CCR4 = 0;
    TIM2->EGR |= (1 << 0);
    TIM2->CR1 |= (1 << 7) | (1 << 0);
}

/*---------- WATCHDOG ----------*/
void initWatchdog(void)
{
    IWDG->KR = 0x5555;
    while(IWDG->SR & (1 << 0));
    IWDG->PR = 0b100;
    while(IWDG->SR & (1 << 1));
    IWDG->RLR = 625;
    IWDG->KR = 0xAAAA;
    IWDG->KR = 0xCCCC;
    printf("Watchdog Active (1s)\r\n");
}

void feedWatchdog(void)
{
    IWDG->KR = 0xAAAA;
    watchdog_feeds++;
}

/*---------- ADC ----------*/
void initADC(void)
{
    RCC->AHB2ENR |= (1 << 13);
    RCC->CCIPR |= (1 << 29) | (1 << 28);
    ADC1_COMMON->CCR = ((0b01) << 16) + (1 << 22);
    
    ADC1->CR = (1 << 28);
    delay(100);
    ADC1->CR |= (1 << 31);
    while(ADC1->CR & (1 << 31));
    
    ADC1->CFGR = (1 << 31);
    ADC1_COMMON->CCR |= (0x0f << 18);
}

int readADC(int chan)
{
    ADC1->SQR1 = 0;
    ADC1->SQR1 |= (chan << 6);
    ADC1->ISR = (1 << 3);
    ADC1->CR |= (1 << 0);
    while ((ADC1->ISR & (1 << 0)) == 0);
    ADC1->CR |= (1 << 2);
    while ((ADC1->ISR & (1 << 3)) == 0);
    return ADC1->DR;
}

/*---------- SERIAL ----------*/
void initSerial(uint32_t baudrate)
{
    RCC->AHB2ENR |= (1 << 0);
    pinMode(GPIOA, 2, 2);
    selectAlternateFunction(GPIOA, 2, 7);
    RCC->APB1ENR1 |= (1 << 17);
    USART2->BRR = 80000000 / baudrate;
    USART2->CR1 = (1 << 3) | (1 << 2) | (1 << 0);
}

void eputc(char c)
{
    while(!(USART2->ISR & (1 << 6)));
    USART2->TDR = c;
}

int _write(int file, char *data, int len)
{
    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
    {
        errno = EBADF;
        return -1;
    }
    while(len--) eputc(*data++);
    return 0;
}

/*---------- SYSTICK ----------*/
void SysTick_Handler(void)
{
    systick_ms++;
}

/*---------- MAIN ----------*/ 
int main()
{
    // Basic clocks
    initClocks();
    RCC->AHB2ENR |= (1 << 0) | (1 << 1) | (1 << 2);
    
    // Setup SysTick for delays
    SysTick->LOAD = 80000 - 1;
    SysTick->CTRL = 7;
    SysTick->VAL = 0;
    
    // Initialize serial
    initSerial(9600);
    
    printf("\r\n========================================\r\n");
    printf("Tilt Detection + Encoder + Watchdog\r\n");
    printf("========================================\r\n");
    delay_ms(500);
    
    // Initialize peripherals
    initI2C();
    init_display();
    initTimer2();
    initADC();
    initEncoder();
    
    printf("All peripherals OK\r\n");
    
    // Test buzzer
    buzzer_on(50);
    delay_ms(200);
    buzzer_off();
    
    // Setup LED
    pinMode(GPIOB, LED_PIN, 1);
    GPIOB->ODR &= ~(1 << LED_PIN);
    
    // Setup button (reset)
    pinMode(GPIOB, BTN_PIN, 0);
    GPIOB->PUPDR |= (1 << (BTN_PIN * 2));
    
    // Wake BMI160
    ResetI2C();
    I2CStart(0x69, WRITE, 2);
    I2CWrite(0x7E);
    I2CWrite(0x11);
    I2CStop();
    delay_ms(1000);
    
    // Initialize Watchdog
    initWatchdog();
    
    printf("\r\n=== SYSTEM READY ===\r\n");
    printf("Threshold: %d\r\n", tilt_threshold);
    printf("Turn encoder CW = Increase threshold & reset alarm\r\n");
    printf("Turn encoder CCW = Decrease threshold\r\n");
    printf("Press button = Reset alarm\r\n");
    printf("========================================\r\n\r\n");
    
    // Main display
    drawRectangle(0, 0, 159, 79, COLOR_GREY);
    printText("Tilt Monitor", 10, 5, COLOR_WHITE, 0);
    printText("Encoder=Thr", 80, 5, COLOR_CYAN, 0);
    
    last_threshold = tilt_threshold;
    
    while(1)
    {
        feedWatchdog();  // Feed the watchdog
        
        /*---------- READ ENCODER & RESET ALARM IF THRESHOLD CHANGES ----------*/
        int change = readEncoder();
        
        if(change > 0) {
            // Increase threshold
            tilt_threshold += 25;
            if(tilt_threshold > 1000) tilt_threshold = 1000;
            printf("Encoder CW: Threshold = %d\r\n", tilt_threshold);
            
            // CRITICAL: Reset alarm when threshold changes
            if(alarm_active) {
                alarm_active = 0;
                GPIOB->ODR &= ~(1 << LED_PIN);
                buzzer_off();
                printf(">>> ALARM RESET by encoder <<<\r\n");
            }
        }
        else if(change < 0) {
            // Decrease threshold
            tilt_threshold -= 25;
            if(tilt_threshold < 50) tilt_threshold = 50;
            printf("Encoder CCW: Threshold = %d\r\n", tilt_threshold);
            
            // Reset alarm when threshold changes
            if(alarm_active) {
                alarm_active = 0;
                GPIOB->ODR &= ~(1 << LED_PIN);
                buzzer_off();
                printf(">>> ALARM RESET by encoder <<<\r\n");
            }
        }
        
        /*---------- READ POTENTIOMETER (Volume control) ----------*/
        pot_value = readADC(5);
        duty_cycle = (pot_value * 100) / 4095;
        
        /*---------- READ BMI160 SENSORS ----------*/
        // Read X acceleration
        I2CStart(0x69, WRITE, 1);
        I2CWrite(0x12);
        I2CReStart(0x69, READ, 2);
        response = I2CRead();
        x_accel = response;
        response = I2CRead();
        x_accel += (response << 8);
        I2CStop();
        X_g = (x_accel * 981) / 16384;

        // Read Y acceleration
        I2CStart(0x69, WRITE, 1);
        I2CWrite(0x14);
        I2CReStart(0x69, READ, 2);
        response = I2CRead();
        y_accel = response;
        response = I2CRead();
        y_accel += (response << 8);
        I2CStop();
        Y_g = (y_accel * 981) / 16384;

        // Read Z acceleration
        I2CStart(0x69, WRITE, 1);
        I2CWrite(0x16);
        I2CReStart(0x69, READ, 2);
        response = I2CRead();
        z_accel = response;
        response = I2CRead();
        z_accel += (response << 8);
        I2CStop();
        Z_g = (z_accel * 981) / 16384;

        /*---------- TILT DETECTION ----------*/
        int tilt_detected = (X_g > tilt_threshold) || (X_g < -tilt_threshold) ||
                            (Y_g > tilt_threshold) || (Y_g < -tilt_threshold);
        
        /*---------- BUTTON RESET ----------*/
        int button = (GPIOB->IDR >> BTN_PIN) & 1;
        if(button == 0) {
            alarm_active = 0;
            GPIOB->ODR &= ~(1 << LED_PIN);
            buzzer_off();
            printf(">>> ALARM RESET by button <<<\r\n");
            delay_ms(200);
        }
        
        /*---------- TRIGGER ALARM IF TILT DETECTED ----------*/
        if(tilt_detected && !alarm_active) {
            alarm_active = 1;
            printf("\r\n");
            printf("\r\n");
            printf("     !!! TILT DETECTED !!!      \r\n");
            printf("\r\n");
            printf("X: %5ld    Y: %5ld    \r\n", X_g, Y_g);
            printf("Threshold: %d              \r\n", tilt_threshold);
            printf("\r\n");
            printf("Turn encoder to reset         \r\n");
            printf("\r\n\r\n");
        }
        
        /*---------- CONTROL LED AND BUZZER ----------*/
        if(alarm_active) {
            GPIOB->ODR |= (1 << LED_PIN);     // LED ON
            buzzer_on(duty_cycle);             // Buzzer ON
        } else {
            GPIOB->ODR &= ~(1 << LED_PIN);    // LED OFF
            buzzer_off();                      // Buzzer OFF
        }
        
        /*---------- UPDATE DISPLAY ----------*/
        drawRectangle(0, 20, 159, 79, COLOR_BLACK);
        
        sprintf(debug_str, "X:%5ld", X_g);
        printText(debug_str, 5, 25, COLOR_WHITE, 0);
        sprintf(debug_str, "Y:%5ld", Y_g);
        printText(debug_str, 5, 35, COLOR_WHITE, 0);
        sprintf(debug_str, "Z:%5ld", Z_g);
        printText(debug_str, 5, 45, COLOR_WHITE, 0);
        
        sprintf(debug_str, "Thr:%4d", tilt_threshold);
        printText(debug_str, 90, 25, COLOR_YELLOW, 0);
        sprintf(debug_str, "Vol:%3d%%", duty_cycle);
        printText(debug_str, 90, 35, COLOR_CYAN, 0);
        
        printText("CW=+Thr", 5, 60, COLOR_GREEN, 0);
        printText("CCW=-Thr", 5, 70, COLOR_GREEN, 0);
        
        if(alarm_active) {
            printText("!!!ALARM!!!", 85, 60, COLOR_RED, 1);
            static int blink = 0;
            blink++;
            if(blink % 20 < 10) {
                printText("TILT!", 95, 70, COLOR_RED, 0);
            } else {
                printText("    ", 95, 70, COLOR_BLACK, 0);
            }
        } else {
            printText("  OK   ", 85, 60, COLOR_GREEN, 1);
            printText("SAFE", 95, 70, COLOR_GREEN, 0);
        }
        
        if(change > 0) {
            printText("CW>", 68, 70, COLOR_GREEN, 0);
        } else if(change < 0) {
            printText("CCW", 68, 70, COLOR_RED, 0);
        } else {
            printText("   ", 68, 70, COLOR_BLACK, 0);
        }
        
        delay_ms(100);
    }
}