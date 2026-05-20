/* ============================================================
   main.c — STM32F401RE  PID Speed Control  (THANH GHI THUẦN)
   ============================================================
   Phần cứng:
     Encoder : TIM2 Quadrature  PA0(A) / PA1(B)
               11 PPR * 4x = 44 xung/vòng trục MOTOR
               Tỉ số truyền 1/45

     PWM     : TIM3 CH1  PA6  → ENA L298N   (1 kHz)
     IN1     : PA7              IN2 : PB6
     UART2   : PA2(TX) PA3(RX)  9600-8N1
     PID     : TIM4 ngắt định kỳ
     LED     : PA5 (LD2 xanh trên Nucleo F401RE)
               Nháy khi |error| > ERROR_THRESHOLD RPM
               Tắt khi motor bám setpoint ổn định

   Khởi động: tự đo chu kỳ TIM4 thực tế bằng SysTick
   → tính pid_period_s chính xác bất kể clock thực tế
   → in ra UART để kiểm tra
============================================================ */

#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ──────────────────────────────────────────────────────────
   HẰNG SỐ
────────────────────────────────────────────────────────── */
#define PPR_MOTOR        44u      /* 11ppr * 4 quadrature           */
#define GEAR_RATIO       45u      /* tỉ số truyền                   */
#define RPM_MAX          130.0f   /* giới hạn setpoint              */
#define PWM_ARR          999u     /* TIM3 ARR                       */
#define ERROR_THRESHOLD  10.0f   /* RPM — ngưỡng cảnh báo LED      */

/* ──────────────────────────────────────────────────────────
   PID GAINS
   Kp = 1.2  → error 100 RPM → output 120%
   Ki = 0.5  → i_lim = 200, không bão hòa sớm
   Kd = 0.1  → giảm dao động
────────────────────────────────────────────────────────── */
float Kp = 1.2f;
float Ki = 0.5f;
float Kd = 0.01f;

/* ──────────────────────────────────────────────────────────
   BIẾN TOÀN CỤC
────────────────────────────────────────────────────────── */
float   pid_period_s = 0.1f;   /* đo thực tế lúc khởi động */
float   setpoint_rpm = 0.0f;
float   integral     = 0.0f;
float   prev_error   = 0.0f;
float   current_rpm  = 0.0f;
int32_t last_count   = 0;

volatile uint8_t pid_flag  = 0;
uint8_t          led_state = 0;   /* trạng thái LED hiện tại */

/* UART buffer */
char    rx_buf[16];
uint8_t rx_idx = 0;

/* ══════════════════════════════════════════════════════════
   KHAI BÁO HÀM
══════════════════════════════════════════════════════════ */
void  GPIO_Init(void);
void  TIM2_Encoder_Init(void);
void  TIM3_PWM_Init(void);
void  TIM4_Timer_Init(void);
void  UART2_Init(void);
void  UART2_SendChar(char c);
void  UART2_SendString(const char *s);
void  Motor_SetOutput(float pct);
void  LED_Update(float error);
float Measure_PID_Period(void);

/* ══════════════════════════════════════════════════════════
   1. GPIO
   PA0  AF1  TIM2_CH1  Encoder A
   PA1  AF1  TIM2_CH2  Encoder B
   PA2  AF7  USART2_TX
   PA3  AF7  USART2_RX
   PA5  OUT  LD2 LED xanh (cảnh báo sai số)
   PA6  AF2  TIM3_CH1  PWM / ENA
   PA7  OUT  IN1
   PB6  OUT  IN2
══════════════════════════════════════════════════════════ */
void GPIO_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    __DSB();

    /* MODER: PA0,1,2,3,6=AF(10)  PA5,7=Output(01) */
    GPIOA->MODER &= ~( GPIO_MODER_MODER0 | GPIO_MODER_MODER1 |
                       GPIO_MODER_MODER2 | GPIO_MODER_MODER3 |
                       GPIO_MODER_MODER5 | GPIO_MODER_MODER6 |
                       GPIO_MODER_MODER7 );

    GPIOA->MODER |=  ( (2u << GPIO_MODER_MODER0_Pos)   /* AF  */
                     | (2u << GPIO_MODER_MODER1_Pos)   /* AF  */
                     | (2u << GPIO_MODER_MODER2_Pos)   /* AF  */
                     | (2u << GPIO_MODER_MODER3_Pos)   /* AF  */
                     | (1u << GPIO_MODER_MODER5_Pos)   /* OUT LED */
                     | (2u << GPIO_MODER_MODER6_Pos)   /* AF  */
                     | (1u << GPIO_MODER_MODER7_Pos)   /* OUT IN1 */
                     );

    /* Speed */
    GPIOA->OSPEEDR |= (3u << GPIO_OSPEEDR_OSPEED5_Pos)
                    | (3u << GPIO_OSPEEDR_OSPEED6_Pos)
                    | (3u << GPIO_OSPEEDR_OSPEED7_Pos);

    /* Pull-up encoder */
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1);
    GPIOA->PUPDR |=  (1u << GPIO_PUPDR_PUPD0_Pos)
                   | (1u << GPIO_PUPDR_PUPD1_Pos);

    /* AF: PA0,PA1=AF1(TIM2)  PA2,PA3=AF7(USART2)  PA6=AF2(TIM3) */
    GPIOA->AFR[0] &= ~( GPIO_AFRL_AFSEL0 | GPIO_AFRL_AFSEL1 |
                        GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3 |
                        GPIO_AFRL_AFSEL6 );

    GPIOA->AFR[0] |=  ( (1u << GPIO_AFRL_AFSEL0_Pos)
                      | (1u << GPIO_AFRL_AFSEL1_Pos)
                      | (7u << GPIO_AFRL_AFSEL2_Pos)
                      | (7u << GPIO_AFRL_AFSEL3_Pos)
                      | (2u << GPIO_AFRL_AFSEL6_Pos)
                      );

    GPIOA->BSRR = GPIO_BSRR_BR5;   /* PA5 LED tắt ban đầu */
    GPIOA->BSRR = GPIO_BSRR_BR7;   /* PA7 IN1 = 0         */

    /* PB6 Output (IN2) */
    GPIOB->MODER &= ~GPIO_MODER_MODER6;
    GPIOB->MODER |=  (1u << GPIO_MODER_MODER6_Pos);
    GPIOB->OSPEEDR |= (3u << GPIO_OSPEEDR_OSPEED6_Pos);
    GPIOB->BSRR = GPIO_BSRR_BR6;
}

/* ══════════════════════════════════════════════════════════
   2. TIM2 — ENCODER 32-bit
      ARR = 0xFFFFFFFF, CNT khởi tại 0x80000000
      int32_t tự xử lý wrap-around đúng
══════════════════════════════════════════════════════════ */
void TIM2_Encoder_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    __DSB();

    TIM2->CR1   = 0;
    TIM2->SMCR  = 0;
    TIM2->CCMR1 = 0;
    TIM2->CCER  = 0;

    TIM2->CCMR1 |= (1u << TIM_CCMR1_CC1S_Pos)
                 | (1u << TIM_CCMR1_CC2S_Pos);

    TIM2->SMCR  |= (3u << TIM_SMCR_SMS_Pos);   /* Encoder mode 3 */

    TIM2->ARR    = 0xFFFFFFFF;
    TIM2->CNT    = 0x80000000;
    TIM2->CR1   |= TIM_CR1_CEN;
}

/* ══════════════════════════════════════════════════════════
   3. TIM3 CH1 — PWM 1 kHz trên PA6
      16MHz / 16 / 1000 = 1 kHz
══════════════════════════════════════════════════════════ */
void TIM3_PWM_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    __DSB();

    TIM3->CR1   = 0;
    TIM3->PSC   = 15u;
    TIM3->ARR   = PWM_ARR;
    TIM3->CCR1  = 0;

    TIM3->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM3->CCER  = TIM_CCER_CC1E;
    TIM3->EGR   = TIM_EGR_UG;
    TIM3->CR1  |= TIM_CR1_CEN;
}

/* ══════════════════════════════════════════════════════════
   4. TIM4 — Ngắt định kỳ (nhắm 100ms)
      PSC=15, ARR=99999
      Chu kỳ thực tế đo bởi Measure_PID_Period()
══════════════════════════════════════════════════════════ */
void TIM4_Timer_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    __DSB();

    TIM4->CR1  = 0;
    TIM4->PSC  = 15u;
    TIM4->ARR  = 99999u;
    TIM4->EGR  = TIM_EGR_UG;
    TIM4->SR   = 0;
    TIM4->DIER = TIM_DIER_UIE;

    NVIC_SetPriority(TIM4_IRQn, 1);
    NVIC_EnableIRQ(TIM4_IRQn);

    TIM4->CR1 |= TIM_CR1_CEN;
}

/* ══════════════════════════════════════════════════════════
   5. USART2 — 9600-8N1  16 MHz
══════════════════════════════════════════════════════════ */
void UART2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    __DSB();

    USART2->CR1 = 0;
    USART2->BRR = (uint32_t)(16000000u / 9600u);
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE
                | USART_CR1_RXNEIE | USART_CR1_UE;

    NVIC_SetPriority(USART2_IRQn, 2);
    NVIC_EnableIRQ(USART2_IRQn);
}

void UART2_SendChar(char c)
{
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = (uint8_t)c;
}

void UART2_SendString(const char *s)
{
    while (*s) UART2_SendChar(*s++);
}

/* ══════════════════════════════════════════════════════════
   6. ĐO CHU KỲ TIM4 THỰC TẾ bằng SysTick

   SysTick dùng AHB clock (SystemCoreClock)
   Đếm lùi → t1 > t2
   period_s = (t1 - t2) / SystemCoreClock
══════════════════════════════════════════════════════════ */
float Measure_PID_Period(void)
{
    SysTick->CTRL = 0;
    SysTick->LOAD = 0x00FFFFFF;   /* ~1 giây ở 16MHz */
    SysTick->VAL  = 0;
    SysTick->CTRL = 0x5;          /* enable, AHB clock, no IRQ */

    /* Flush chu kỳ đang dở */
    pid_flag = 0;
    while (!pid_flag);

    /* Đo chu kỳ tiếp theo */
    pid_flag = 0;
    uint32_t t1 = SysTick->VAL;
    while (!pid_flag);
    uint32_t t2 = SysTick->VAL;

    SysTick->CTRL = 0;

    uint32_t ticks = t1 - t2;
    if (ticks == 0u) return 0.1f;

    return (float)ticks / (float)SystemCoreClock;
}

/* ══════════════════════════════════════════════════════════
   7. MOTOR OUTPUT
   pct > 0 → thuận  (IN1=1, IN2=0)
   pct < 0 → ngược  (IN1=0, IN2=1)
   setpoint = 0 → brake
══════════════════════════════════════════════════════════ */
void Motor_SetOutput(float pct)
{
    if (setpoint_rpm == 0.0f)
    {
        GPIOA->BSRR = GPIO_BSRR_BR7;
        GPIOB->BSRR = GPIO_BSRR_BR6;
        TIM3->CCR1  = 0;
        return;
    }

    if (pct >  100.0f) pct =  100.0f;
    if (pct < -100.0f) pct = -100.0f;

    float    abs_pct = (pct < 0.0f) ? -pct : pct;
    uint32_t pwm     = (uint32_t)(abs_pct / 100.0f * (float)PWM_ARR);

    if (pct >= 0.0f)
    {
        GPIOA->BSRR = GPIO_BSRR_BS7;   /* IN1=1 */
        GPIOB->BSRR = GPIO_BSRR_BR6;   /* IN2=0 */
    }
    else
    {
        GPIOA->BSRR = GPIO_BSRR_BR7;   /* IN1=0 */
        GPIOB->BSRR = GPIO_BSRR_BS6;   /* IN2=1 */
    }

    TIM3->CCR1 = pwm;
}

/* ══════════════════════════════════════════════════════════
   8. LED_Update — gọi mỗi chu kỳ PID

   |error| > ERROR_THRESHOLD  → LED nháy (toggle mỗi chu kỳ)
   |error| <= ERROR_THRESHOLD → LED tắt (ổn định)
   setpoint = 0               → LED tắt
══════════════════════════════════════════════════════════ */
void LED_Update(float error)
{
    /* Tắt LED khi không có setpoint */
    if (setpoint_rpm == 0.0f)
    {
        GPIOA->BSRR = GPIO_BSRR_BR5;
        led_state = 0;
        return;
    }

    float abs_err = (error < 0.0f) ? -error : error;

    if (abs_err > ERROR_THRESHOLD)
    {
        /* Toggle LED mỗi chu kỳ PID → nháy */
        if (led_state)
        {
            GPIOA->BSRR = GPIO_BSRR_BR5;   /* tắt */
            led_state = 0;
        }
        else
        {
            GPIOA->BSRR = GPIO_BSRR_BS5;   /* bật */
            led_state = 1;
        }
    }
    else
    {
        /* Sai số nhỏ → LED tắt hoàn toàn */
        GPIOA->BSRR = GPIO_BSRR_BR5;
        led_state = 0;
    }
}

/* ══════════════════════════════════════════════════════════
   9. TIM4 ISR — set flag mỗi chu kỳ
══════════════════════════════════════════════════════════ */
void TIM4_IRQHandler(void)
{
    if (TIM4->SR & TIM_SR_UIF)
    {
        TIM4->SR &= ~TIM_SR_UIF;
        pid_flag = 1;
    }
}

/* ══════════════════════════════════════════════════════════
   10. USART2 ISR
   Nhập RPM rồi Enter:
     "100"  → +100 RPM thuận
     "-80"  → -80  RPM ngược
     "0"    → dừng
══════════════════════════════════════════════════════════ */
void USART2_IRQHandler(void)
{
    if (USART2->SR & USART_SR_RXNE)
    {
        char c = (char)(USART2->DR & 0xFF);
        UART2_SendChar(c);

        if (c == '\r' || c == '\n')
        {
            if (rx_idx > 0)
            {
                rx_buf[rx_idx] = '\0';
                rx_idx = 0;

                float new_sp = (float)atoi(rx_buf);

                if (new_sp >  RPM_MAX) new_sp =  RPM_MAX;
                if (new_sp < -RPM_MAX) new_sp = -RPM_MAX;

                setpoint_rpm = new_sp;
                integral     = 0.0f;
                prev_error   = 0.0f;

                char msg[48];
                snprintf(msg, sizeof(msg),
                         "\r\n>> Setpoint: %d RPM\r\n",
                         (int)setpoint_rpm);
                UART2_SendString(msg);
            }
        }
        else if (rx_idx < (uint8_t)(sizeof(rx_buf) - 1u))
        {
            rx_buf[rx_idx++] = c;
        }
    }
}

/* ══════════════════════════════════════════════════════════
   11. MAIN
══════════════════════════════════════════════════════════ */
int main(void)
{
    GPIO_Init();
    TIM2_Encoder_Init();
    TIM3_PWM_Init();
    TIM4_Timer_Init();
    UART2_Init();

    UART2_SendString("\r\n=== STM32F401RE PID Motor Control ===\r\n");
    UART2_SendString("Dang do chu ky TIM4...\r\n");

    /* Đo chu kỳ TIM4 thực tế */
    pid_period_s = Measure_PID_Period();

    /* In chu kỳ thực tế ra UART */
    {
        int period_ms   = (int)(pid_period_s * 1000.0f);
        int period_frac = (int)(pid_period_s * 10000.0f) % 10;
        char msg[64];
        snprintf(msg, sizeof(msg),
                 "Chu ky TIM4 thuc te: %d.%d ms\r\n",
                 period_ms, period_frac);
        UART2_SendString(msg);
    }

    /* Đồng bộ last_count sau khi đo xong */
    last_count = (int32_t)(TIM2->CNT);
    integral   = 0.0f;
    prev_error = 0.0f;

    UART2_SendString("Nguong canh bao LED: ±10 RPM\r\n");
    UART2_SendString("Pham vi: -130 den +130 RPM\r\n");
    UART2_SendString("Nhap RPM roi nhan Enter:\r\n");

    while (1)
    {
        if (!pid_flag) continue;
        pid_flag = 0;

        /* ── Đọc encoder (32-bit) ─────────────────────── */
        int32_t now   = (int32_t)(TIM2->CNT);
        int32_t delta = now - last_count;
        last_count    = now;

        /*
         * RPM trục tải dùng pid_period_s đã đo thực tế
         * Dấu âm: encoder nối ngược chiều quay
         * Nếu CUR ngược dấu SP → xóa dấu âm
         */
        current_rpm = -( (float)delta * 60.0f
                       / ( (float)PPR_MOTOR
                         * (float)GEAR_RATIO
                         * pid_period_s ) );

        /* ── PID (output đơn vị %) ────────────────────── */
        float error = setpoint_rpm - current_rpm;

        integral += error * pid_period_s;

        float i_lim = (Ki > 0.0f) ? (100.0f / Ki) : 100.0f;
        if (integral >  i_lim) integral =  i_lim;
        if (integral < -i_lim) integral = -i_lim;

        float derivative = (error - prev_error) / pid_period_s;
        prev_error = error;

        float output = Kp * error
                     + Ki * integral
                     + Kd * derivative;

        /* ── Ra motor ────────────────────────────────── */
        Motor_SetOutput(output);

        /* ── Cảnh báo LED ────────────────────────────── */
        LED_Update(error);

        /* ── Log ra UART ─────────────────────────────── */
        int sp_i  = (int)setpoint_rpm;
        int cur_i = (int)current_rpm;
        int out_i = (int)output;

        float frac = current_rpm - (float)cur_i;
        if (frac < 0.0f) frac = -frac;
        int cur_d = (int)(frac * 10.0f);

        /* Cột ERR để dễ theo dõi */
        int err_i = (int)error;
        float err_frac = error - (float)err_i;
        if (err_frac < 0.0f) err_frac = -err_frac;
        int err_d = (int)(err_frac * 10.0f);

        char buf[80];
        snprintf(buf, sizeof(buf),
                 "SP:%4d | CUR:%4d.%1d | ERR:%4d.%1d | OUT:%4d%% | PWM:%4lu\r\n",
                 sp_i, cur_i, cur_d, err_i, err_d, out_i,
                 (unsigned long)TIM3->CCR1);
        UART2_SendString(buf);
    }
}
