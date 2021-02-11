/*
 * Copyright (c) 2019-2021 Kim Jørgensen, Marko Edzes
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

/* 
 * Marko Edzes: The code below is chagned to allow compile-time choice of GPIO banks and pin positions
 * for the KungFuFlash rev.2 PCB and the DEVEBOX prototype from the Makefile.
 * Furthermore, the Timer 1 interrupt time position is split into Kernal mode and non-Kernal mode for easier modification.
 * 
 * The DEVEBOX comes with a LED on PA1. That's on a different GPIO bank than the other control signals on PD. 
 * For DEVEBOX the led is only toggled in non-critical code sections.
 * 
 * DEVEBOX uses PD8-PD15 for the databus. This is just as fast for access, as long as it is accessed with ldrb/strb.
 * To support this the databus access is 8-bit throughout the code also when compiled for rev 2 PCB.
 * There may be minor timing differences due to this where the compiler inserts utxb's (8 to 32 bit unsigned extractions)
 * even when they are not needed, in *different* places than the original code.
 * In the original code there are also unneccessary utxb's. (gcc compiler quirk)
 * 
 * DEVEBOX has no C64-reset output inverter nor a driver transistor. This is not an issue as all pins connected to the C64 
 * are inputs during and after ARM reset. (The C64 is likely to crash though but there is no bus conflict)
 * 
 * DEVEBOX connects the DMA line.
 */

// The following definition forces reliable gcc code inlining when compiling with -Os.
#define INLINE inline __attribute__((always_inline))

#define MENU_RAM_SIGNATURE  "KungFu:Menu"

#ifdef DEVEBOX
#define GPIO_ADDRESSBUS GPIOE
#define GPIO_DATABUS GPIOD
#define GPIO_DATABUS_LANE 1
#else
#define GPIO_ADDRESSBUS GPIOB
#define GPIO_DATABUS GPIOC
#define GPIO_DATABUS_LANE 0
#endif

static inline void set_menu_signature(void)
{
    memcpy(scratch_buf, MENU_RAM_SIGNATURE, sizeof(MENU_RAM_SIGNATURE));
}

static inline bool menu_signature(void)
{
    return memcmp(scratch_buf, MENU_RAM_SIGNATURE, sizeof(MENU_RAM_SIGNATURE)) == 0;
}

static inline void invalidate_menu_signature(void)
{
    *((uint32_t *)scratch_buf) = 0;
}

/*************************************************
* C64 address bus on PB0-PB15 (or PE0-PE15)
* Returned as 32 bit value for performance
*************************************************/
static INLINE uint32_t c64_addr_read()
{
    return GPIO_ADDRESSBUS->IDR;
}

static void c64_address_config(void)
{
    // Make GPIOB input
    GPIO_ADDRESSBUS->MODER = 0;

    // Set output low
    GPIO_ADDRESSBUS->ODR = 0;

    // Set GPIOB to low speed
    GPIO_ADDRESSBUS->OSPEEDR = 0;

    // No pull-up, pull-down
    GPIO_ADDRESSBUS->PUPDR = 0;
}

/*************************************************
* C64 data bus on PC0-PC7 (or PD8-PD15)
* Returned as 32 bit value for performance
*************************************************/
static INLINE uint32_t c64_data_read()
{
    return (uint32_t) (*((volatile uint8_t *)&GPIO_DATABUS->IDR + GPIO_DATABUS_LANE));
}

static INLINE void c64_data_write(uint8_t data)
{
    // Make PC0-PC7 (or PD8-15) output
    *((volatile uint16_t *)&GPIO_DATABUS->MODER + GPIO_DATABUS_LANE) = 0x5555;

    *((volatile uint8_t *)&GPIO_DATABUS->ODR + GPIO_DATABUS_LANE) = data;
    __DMB();
}

static INLINE void c64_data_input(void)
{
    // Make PC0-PC7 (or PD8-15) input
    *((volatile uint16_t *)&GPIO_DATABUS->MODER + GPIO_DATABUS_LANE) = 0x0000;
    __DMB();
}

static void c64_data_config(void)
{
    // Make PC0-PC7 (or PD8-15) input
    c64_data_input();
}

/*************************************************
* C64 control bus on PA0-PA3 and PA6-PA7 (or PC0-4,PC6-PC7,PC13)
* Menu button & special button on PA4 & PA5 (or PC4, PC5)
* Returned as 32 bit value for performance
*************************************************/
#ifdef DEVEBOX
#define GPIO_CONTROL GPIOC
#define C64_WRITE_POS   0    // R/W on PC0
#define C64_IO1_POS     1    // IO1 on PC1
#define C64_IO2_POS     2    // IO2 on PC2
#define C64_BA_POS      3    // BA on PC3
#define MENU_BTN_POS    4    // Menu button on PC4, to be compatible with the EXTI4 code.
#define SPECIAL_BTN_POS 13   // Special button on PC13
#define C64_ROML_POS    6    // ROML on PC6
#define C64_ROMH_POS    7    // ROMH on PC7
#define MENU_BTN_INT_PORT SYSCFG_EXTICR2_EXTI4_PC
#else
#define GPIO_CONTROL GPIOA
#define C64_WRITE_POS   0    // R/W on PA0
#define C64_IO1_POS     1    // IO1 on PA1
#define C64_IO2_POS     2    // IO2 on PA2
#define C64_BA_POS      3    // BA on PA3
#define MENU_BTN_POS    4    // Menu button on PA4
#define SPECIAL_BTN_POS 5    // Special button on PA5
#define C64_ROML_POS    6    // ROML on PA6
#define C64_ROMH_POS    7    // ROMH on PA7
#define MENU_BTN_INT_PORT SYSCFG_EXTICR2_EXTI4_PA
#endif
#define C64_WRITE   (1UL<<C64_WRITE_POS)
#define C64_IO1     (1UL<<C64_IO1_POS)
#define C64_IO2     (1UL<<C64_IO2_POS)
#define C64_BA      (1UL<<C64_BA_POS)
#define MENU_BTN    (1UL<<MENU_BTN_POS)
#define SPECIAL_BTN (1UL<<SPECIAL_BTN_POS)
#define C64_ROML    (1UL<<C64_ROML_POS)
#define C64_ROMH    (1UL<<C64_ROMH_POS)

static inline uint32_t c64_control_read()
{
    return GPIO_CONTROL->IDR;
}

static void c64_control_config(void)
{
    // Make PA0-PA3 and PA6-PA7 input (or PC0-4, PC6-7, PC13)
    GPIO_CONTROL->MODER &= ~(
      (3UL<<C64_WRITE_POS*2) |
      (3UL<<C64_IO1_POS*2) |
      (3UL<<C64_IO2_POS*2) |
      (3UL<<C64_BA_POS*2) |
      (3UL<<C64_ROML_POS*2) |
      (3UL<<C64_ROMH_POS*2)
    ); 		
}

// Wait until the raster beam is in the upper or lower border (if VIC-II is enabled)
static void c64_sync_with_vic(void)
{
    timer_start_us(1000);
    while (!timer_elapsed())
    {
        if (!(c64_control_read() & C64_BA))
        {
            timer_reset();
        }
    }
}

/*************************************************
* C64 GAME and EXROM on PC14 & PC15 (or PD0-1,PD2)
* Status LED on PC13 (or PA1)
*************************************************/

#ifdef DEVEBOX
#define GPIO_OTHER GPIOD
#define C64_GAME_HIGH   GPIO_BSRR_BS0
#define C64_GAME_LOW    GPIO_BSRR_BR0
#define C64_EXROM_HIGH  GPIO_BSRR_BS1
#define C64_EXROM_LOW   GPIO_BSRR_BR1
#define C64_DMA_HIGH    GPIO_BSRR_BS3
#define C64_DMA_LOW     GPIO_BSRR_BR3
#define C64_GAME_POS    0
#define C64_EXROM_POS   1
#define C64_DMA_POS     3

#define GPIO_STATUS_LED    GPIOA
// Status led on PA1 can't be written at the same time as GAME/EXROM on PD. 
// So for DEVEBOX the STATUS_LED_ON/STATUS_LED_OFF macros do nothing.
#define STATUS_LED_ON      (0)
#define STATUS_LED_OFF     (0)
#define OPT_STATUS_LED_ON  GPIO_BSRR_BR1
#define OPT_STATUS_LED_OFF GPIO_BSRR_BS1
#define STATUS_LED_POS     1

#else

#define GPIO_OTHER GPIOC
#define C64_GAME_HIGH   GPIO_BSRR_BS14
#define C64_GAME_LOW    GPIO_BSRR_BR14
#define C64_EXROM_HIGH  GPIO_BSRR_BS15
#define C64_EXROM_LOW   GPIO_BSRR_BR15
#define C64_GAME_POS    14
#define C64_EXROM_POS   15
// DMA pin is not supported on pcb rev2.
#define C64_DMA_HIGH    (0)
#define C64_DMA_LOW     (0)
#define C64_DMA_POS     -1

#define GPIO_STATUS_LED GPIOC
#define STATUS_LED_ON   GPIO_BSRR_BR13
#define STATUS_LED_OFF  GPIO_BSRR_BS13
#define OPT_STATUS_LED_ON   GPIO_BSRR_BR13
#define OPT_STATUS_LED_OFF  GPIO_BSRR_BS13
#define STATUS_LED_POS  13
#endif

static INLINE void c64_crt_control(uint32_t state)
{
    GPIO_OTHER->BSRR = state;
}

static INLINE void led_off(void)
{
    GPIO_STATUS_LED->BSRR = OPT_STATUS_LED_OFF;
}

static INLINE void led_on(void)
{
    GPIO_STATUS_LED->BSRR = OPT_STATUS_LED_ON;
}

static INLINE void led_set(uint32_t state)
{
    GPIO_STATUS_LED->BSRR = (state == 0 ? OPT_STATUS_LED_OFF : OPT_STATUS_LED_ON);
}

static INLINE void led_toggle(void)
{
    GPIO_STATUS_LED->ODR ^= (1UL<<STATUS_LED_POS);
}

static void c64_crt_config(void)
{
    // Cartridge inactive
    led_off();
    c64_crt_control(C64_GAME_HIGH | C64_EXROM_HIGH | C64_DMA_HIGH);

    // Set PC14 & PC15 (or PD0,PD1,PD3) as open-drain
    GPIO_OTHER->OTYPER |= 
     (1UL<<C64_GAME_POS) |
     (1UL<<C64_EXROM_POS) |
     ((C64_DMA_POS>=0) ? (1UL<<C64_DMA_POS) : 0);

    // Set PC13-PC15 (or PD0,PD1,PD3, PA1) as output 
    MODIFY_REG(GPIO_STATUS_LED->MODER, (3UL<<STATUS_LED_POS*2), (1UL<<STATUS_LED_POS*2));
    MODIFY_REG(GPIO_OTHER->MODER,
        (3UL<<C64_GAME_POS*2) |
        (3UL<<C64_EXROM_POS*2) |
        ((C64_DMA_POS>=0) ? (3UL<<C64_DMA_POS*2) : 0),
        (1UL<<C64_GAME_POS*2) |
        (1UL<<C64_EXROM_POS*2) |
        ((C64_DMA_POS>=0) ? (1UL<<C64_DMA_POS*2) : 0));
        
    // Default game, exrom, dma outputs slow.
    GPIO_OTHER->OSPEEDR &= ~(
        (3UL<<C64_GAME_POS*2) |
        (3UL<<C64_EXROM_POS*2) |
        ((C64_DMA_POS>=0) ? (3UL<<C64_DMA_POS*2) : 0)
	);
}

/*************************************************
* C64 IRQ and NMI on PA9 & PA10 (or PD4,PD5)
*************************************************/
#ifdef DEVEBOX
#define GPIO_INTERRUPT GPIOD
#define C64_IRQ_HIGH        GPIO_BSRR_BS4
#define C64_IRQ_LOW         GPIO_BSRR_BR4
#define C64_IRQ_POS         4
#define C64_NMI_HIGH        GPIO_BSRR_BS5
#define C64_NMI_LOW         GPIO_BSRR_BR5
#define C64_NMI_POS         5
#define C64_IRQ_NMI_HIGH    (C64_IRQ_HIGH|C64_NMI_HIGH)
#define C64_IRQ_NMI_LOW     (C64_IRQ_LOW|C64_NMI_LOW)
#else
#define GPIO_INTERRUPT GPIOA
#define C64_IRQ_HIGH        GPIO_BSRR_BS9
#define C64_IRQ_LOW         GPIO_BSRR_BR9
#define C64_IRQ_POS         9
#define C64_NMI_HIGH        GPIO_BSRR_BS10
#define C64_NMI_LOW         GPIO_BSRR_BR10
#define C64_NMI_POS         10
#define C64_IRQ_NMI_HIGH    (C64_IRQ_HIGH|C64_NMI_HIGH)
#define C64_IRQ_NMI_LOW     (C64_IRQ_LOW|C64_NMI_LOW)
#endif

static INLINE void c64_irq_nmi(uint32_t state)
{
    GPIO_INTERRUPT->BSRR = state;
}

static void c64_irq_config(void)
{
    c64_irq_nmi(C64_IRQ_NMI_HIGH);

    // Set PA9 & PA10 (or PD4, PD5) as open-drain
    GPIO_INTERRUPT->OTYPER |= (
      (1UL<<C64_IRQ_POS) |
      (1UL<<C64_NMI_POS)
    );

    // Set PA9 & PA10 (or PD4, PD5) as output
    MODIFY_REG(GPIO_INTERRUPT->MODER, 
      (3UL<<C64_IRQ_POS*2) | 
      (3UL<<C64_NMI_POS*2),
      (1UL<<C64_IRQ_POS*2) | 
      (1UL<<C64_NMI_POS*2)
    );
}

/*************************************************
* C64 clock input on PA8 (timer 1)
* C64 clock is 0.985 MHz (PAL) / 1.023 MHz (NTSC)
*************************************************/
static void c64_clock_config()
{
     // Enable TIM1 clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    __DSB();

    // Set PA8 as alternate mode 1 (TIM1_CH1)
    MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL8, GPIO_AFRH_AFSEL8_0);
    MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER8, GPIO_MODER_MODER8_1);

    // No prescaler, timer runs at ABP2 timer clock speed (168 MHz)
    TIM1->PSC = 0;

    /**** Setup timer 1 to measure clock speed in CCR1 and duty cycle in CCR2 ****/

    // CC1 and CC2 channel is input, IC1 and IC2 is mapped on TI1
    MODIFY_REG(TIM1->CCMR1,
               TIM_CCMR1_CC1S|TIM_CCMR1_CC2S,
               TIM_CCMR1_CC1S_0|TIM_CCMR1_CC2S_1);

    // TI1FP1 active on falling edge. TI1FP2 active on rising edge
    MODIFY_REG(TIM1->CCER,
               TIM_CCER_CC1P|TIM_CCER_CC1NP|TIM_CCER_CC2P|TIM_CCER_CC2NP,
               TIM_CCER_CC1P);

    // Select TI1FP1 as trigger
    MODIFY_REG(TIM1->SMCR, TIM_SMCR_TS|TIM_SMCR_SMS, TIM_SMCR_TS_2|TIM_SMCR_TS_0);

    // Set reset mode
    TIM1->SMCR |= TIM_SMCR_SMS_2;

    // Enable capture 1 and 2
    TIM1->CCER |= TIM_CCER_CC1E|TIM_CCER_CC2E;

    // Disable UEV events and enable counter
    TIM1->CR1 |= TIM_CR1_UDIS|TIM_CR1_CEN;

    /**** Setup phi2 interrupt ****/

    // Generate OC3 interrupt before 0.5 of the C64 clock cycle
    // Value set in c64_interface()
    TIM1->CCR3 = 0;

    // Enable compare mode 1. OC3 is high when TIM1_CNT == TIM1_CCR3
    MODIFY_REG(TIM1->CCMR2, TIM_CCMR2_OC3M, TIM_CCMR2_OC3M_0);

    // Disable all TIM1 (and TIM8) interrupts
    TIM1->DIER = 0;

    // Enable TIM1_CC_IRQn, highest priority
    NVIC_SetPriority(TIM1_CC_IRQn, 0);
    NVIC_EnableIRQ(TIM1_CC_IRQn);
}

// C64_BUS_HANDLER timing
#define NTSC_PHI2_HIGH      93
#define NTSC_PHI2_INT       (NTSC_PHI2_HIGH - 41)
#define NTSC_PHI2_LOW       142

#define PAL_PHI2_HIGH       96
#define PAL_PHI2_INT        (PAL_PHI2_HIGH - 43)
#define PAL_PHI2_LOW        149

// Kernal timing is separate for easier modification
#define PAL_PHI2_KERNAL_INT (PAL_PHI2_HIGH - 33)

// C64_VIC_BUS_HANDLER timing
#ifdef NTSC

#define PHI2_CPU_START      94
#define PHI2_WRITE_DELAY    126
#define PHI2_CPU_END        NTSC_PHI2_LOW

#define PHI2_VIC_START      17
#define PHI2_VIC_DELAY      32
#define PHI2_VIC_END        58

#define C64_CPU_VIC_DELAY() \
    __NOP();                \
    __NOP();                \
    __NOP();                \
    __NOP();                \
    __NOP();                \
    __NOP();

#else // PAL

#define PHI2_CPU_START      102
#define PHI2_WRITE_DELAY    130
#define PHI2_CPU_END        PAL_PHI2_LOW

#define PHI2_VIC_START      18
#define PHI2_VIC_DELAY      33
#define PHI2_VIC_END        61

#define C64_CPU_VIC_DELAY() \
    __NOP();                \
    __NOP();                \
    __NOP();                \
    __NOP();                \
    __NOP();                \
    __NOP();                \
    __NOP();                \
    __NOP();

#endif

#define C64_BUS_HANDLER(name)                                                       \
        C64_BUS_HANDLER_(name##_handler, name##_read_handler, name##_write_handler)

#define C64_BUS_HANDLER_READ(name, read_handler)                                    \
        C64_BUS_HANDLER_(name##_handler, read_handler, name##_write_handler)

#define C64_BUS_HANDLER_(name, read_handler, write_handler)                     \
static void name(void)                                                          \
{                                                                               \
    /* We need to clear the interrupt flag early otherwise the next */          \
    /* interrupt may be delayed */                                              \
    TIM1->SR = ~TIM_SR_CC3IF;                                                   \
    __DSB();                                                                    \
    /* Use debug cycle counter which is faster to access than timer */          \
    DWT->CYCCNT = TIM1->CNT;                                                    \
    __DMB();                                                                    \
    uint32_t phi2_high = DWT->COMP0;                                            \
    while (DWT->CYCCNT < phi2_high);                                            \
    uint32_t addr = c64_addr_read();                                            \
    uint32_t control = c64_control_read();                                      \
    if (control & C64_WRITE)                                                    \
    {                                                                           \
        COMPILER_BARRIER();                                                     \
        if (read_handler(control, addr))                                        \
        {                                                                       \
            /* Wait for phi2 to go low */                                       \
            uint32_t phi2_low = DWT->COMP1;                                     \
            while (DWT->CYCCNT < phi2_low);                                     \
            /* We releases the bus as fast as possible when phi2 is low */      \
            c64_data_input();                                                   \
        }                                                                       \
    }                                                                           \
    else                                                                        \
    {                                                                           \
        COMPILER_BARRIER();                                                     \
        uint32_t data = c64_data_read();                                        \
        write_handler(control, addr, data);                                     \
    }                                                                           \
}

#define C64_VIC_BUS_HANDLER_EX(name)                                                \
        C64_VIC_BUS_HANDLER_EX_(name##_handler, name##_vic_read_handler,            \
                                name##_read_handler, name##_early_write_handler,    \
                                name##_write_handler, C64_VIC_DELAY)

#define C64_VIC_BUS_HANDLER(name)                                                   \
        C64_VIC_BUS_HANDLER_EX_(name##_handler, name##_read_handler,                \
                                name##_read_handler, C64_WRITE_DELAY,               \
                                name##_write_handler, C64_VIC_DELAY)

#define C64_C128_BUS_HANDLER(name)                                                  \
        C64_VIC_BUS_HANDLER_EX_(name##_handler, name##_read_handler,                \
                                name##_read_handler, C64_WRITE_DELAY,               \
                                name##_write_handler, C64_NO_DELAY)

#define C64_NO_DELAY()
#define C64_WRITE_DELAY()                                                           \
    /* Wait for data to become ready on the data bus */                             \
    while (DWT->CYCCNT < PHI2_WRITE_DELAY);

#define C64_VIC_DELAY()                                                             \
    /* Wait for the control bus to become stable */                                 \
    while (DWT->CYCCNT < PHI2_VIC_DELAY);

// This supports VIC-II reads from the cartridge (i.e. character and sprite data)
// but uses 100% CPU - other interrupts are not served due to the interrupt priority
#define C64_VIC_BUS_HANDLER_EX_(name, vic_read_handler, read_handler,           \
                                early_write_handler, write_handler, vic_delay)  \
void name(void)                                                                 \
{                                                                               \
    /* As we don't return from this handler, we need to do this here */         \
    c64_reset(false);                                                           \
    /* Use debug cycle counter which is faster to access than timer */          \
    DWT->CYCCNT = TIM1->CNT;                                                    \
    __DMB();                                                                    \
    while (true)                                                                \
    {                                                                           \
        /* Wait for CPU cycle */                                                \
        while (DWT->CYCCNT < PHI2_CPU_START);                                   \
        uint32_t addr = c64_addr_read();                                        \
        COMPILER_BARRIER();                                                     \
        uint32_t control = c64_control_read();                                  \
        /* Check if CPU has the bus (no bad line) */                            \
        if ((control & (C64_BA|C64_WRITE)) == (C64_BA|C64_WRITE))               \
        {                                                                       \
            if (read_handler(control, addr))                                    \
            {                                                                   \
                /* Release bus when phi2 is going low */                        \
                while (DWT->CYCCNT < PHI2_CPU_END);                             \
                c64_data_input();                                               \
            }                                                                   \
        }                                                                       \
        else if (!(control & C64_WRITE))                                        \
        {                                                                       \
            early_write_handler();                                              \
            uint32_t data = c64_data_read();                                    \
            write_handler(control, addr, data);                                 \
        }                                                                       \
        /* VIC-II has the bus */                                                \
        else                                                                    \
        {                                                                       \
            /* Wait for the control bus to become stable */                     \
            C64_CPU_VIC_DELAY()                                                 \
            control = c64_control_read();                                       \
            if (vic_read_handler(control, addr))                                \
            {                                                                   \
                /* Release bus when phi2 is going low */                        \
                while (DWT->CYCCNT < PHI2_CPU_END);                             \
                c64_data_input();                                               \
            }                                                                   \
        }                                                                       \
        if (control & MENU_BTN)                                                 \
        {                                                                       \
            /* Allow the menu button interrupt handler to run */                \
            c64_interface(false);                                               \
            break;                                                              \
        }                                                                       \
        /* Wait for VIC-II cycle */                                             \
        while (TIM1->CNT >= 80);                                                \
        DWT->CYCCNT = TIM1->CNT;                                                \
        __DMB();                                                                \
        while (DWT->CYCCNT < PHI2_VIC_START);                                   \
        addr = c64_addr_read();                                                 \
        COMPILER_BARRIER();                                                     \
        /* Ideally, we would always wait until PHI2_VIC_DELAY here which is */  \
        /* required when the VIC-II has the bus, but we need more cycles */     \
        /* in C128 2 MHz mode where data is read from flash */                  \
        vic_delay();                                                            \
        control = c64_control_read();                                           \
        if (vic_read_handler(control, addr))                                    \
        {                                                                       \
            /* Release bus when phi2 is going high */                           \
            while (DWT->CYCCNT < PHI2_VIC_END);                                 \
            c64_data_input();                                                   \
        }                                                                       \
    }                                                                           \
    TIM1->SR = ~TIM_SR_CC3IF;                                                   \
    __DMB();                                                                    \
}

#define C64_INSTALL_HANDLER(handler)                    \
    /* Set TIM1_CC_IRQHandler vector */                 \
    ((uint32_t *)0x00000000)[43] = (uint32_t)handler

static INLINE bool c64_is_ntsc(void)
{
    return TIM1->CCR1 < 167;
}

static inline bool c64_fw_supports_crt(void)
{
#ifdef NTSC
    return c64_is_ntsc();
#else
    return !c64_is_ntsc();
#endif
}

/*************************************************
* C64 interface status
*************************************************/
// Returns if the C64 interface is up and running
static inline bool c64_interface_active(void)
{
    return (TIM1->DIER & TIM_DIER_CC3IE) != 0;
}

// Enables and disables the ARM interrupt on PHI2. 
// When disabled, the C64 can keep running normally, but no cartridge,
// soft-kernal or easyflash register reads/writes can be done.

typedef enum {INTERFACE_OFF =0, INTERFACE_NORMAL, INTERFACE_KERNAL} interface_t;

static interface_t current_interface_timing;
static void c64_interface(interface_t state)
{
    if (state==INTERFACE_OFF)
    {
        // Capture/Compare 3 interrupt disable
        TIM1->DIER &= ~TIM_DIER_CC3IE;
        TIM1->SR = ~TIM_SR_CC3IF;
        current_interface_timing = INTERFACE_OFF;
        return;
    }

    if (current_interface_timing == state)
    {
        return;
    }

    uint8_t valid_clock_count = 0;
    uint32_t led_activity = 0;

    // Wait for a valid C64 clock signal
    while (valid_clock_count < 3)
    {
        // NTSC: 161-164, PAL: 168-169
        if(TIM1->CCR1 < 161 || TIM1->CCR1 > 169)
        {
            valid_clock_count = 0;

            // Fast blink if no valid clock
            if (led_activity++ > 15000)
            {
                led_activity = 0;
                led_toggle();
            }
        }
        else
        {
            valid_clock_count++;
            led_on();
        }

        delay_us(2); // Wait more than a clock cycle
    }

    if (c64_is_ntsc())
    {
        // NTSC timing
        TIM1->CCR3 = NTSC_PHI2_INT;     // generate interrupt before phi2 is high

        // Abuse COMPx registers for better performance
        DWT->COMP0 = NTSC_PHI2_HIGH;    // after phi2 is high
        DWT->COMP1 = NTSC_PHI2_LOW;     // before phi2 is low
    }
    else
    {
		// PAL timing: interrupt timing before phi2 is high
		TIM1->CCR3 = (state == INTERFACE_KERNAL) ? PAL_PHI2_KERNAL_INT : PAL_PHI2_INT;
		// Abuse COMPx registers for better performance
		DWT->COMP0 = PAL_PHI2_HIGH;    // after phi2 is high
		DWT->COMP1 = PAL_PHI2_LOW;     // before phi2 is low
    }

    // Capture/Compare 3 interrupt enable
    TIM1->SR = ~TIM_SR_CC3IF;
    TIM1->DIER |= TIM_DIER_CC3IE;
}

/*************************************************
* C64 reset on PA15 (or PD6)
* Reset is inverted externally with a drive transistor in PCB rev2. Reset is not inverted in DEVEBOX.
*************************************************/
#ifdef DEVEBOX
#define GPIO_RESET     GPIOD
#define C64_RESET_ON   GPIO_BSRR_BR6
#define C64_RESET_OFF  GPIO_BSRR_BS6
#define C64_RESET_POS  6
#define C64_RESET_INVERT 0
#else
#define GPIO_RESET     GPIOA
#define C64_RESET_ON   GPIO_BSRR_BS15
#define C64_RESET_OFF  GPIO_BSRR_BR15
#define C64_RESET_POS  15
#define C64_RESET_INVERT 1
#endif

// Enable or disable the reset line towards the C64
static inline void c64_reset(bool state)
{
    if (state)
    {
        GPIO_RESET->BSRR = C64_RESET_ON;
        delay_us(200); // Make sure that the C64 is reset
    }
    else
    {
        GPIO_RESET->BSRR = C64_RESET_OFF;
    }
}

static inline bool c64_is_reset(void)
{
    return (GPIO_RESET->ODR & (1UL << C64_RESET_POS)) == (C64_RESET_INVERT ? (1UL << C64_RESET_POS) : 0);
}

// Turn the C64 cartridge interface on, and then let the C64 start.
// Does not change EXROM/GAME state.
static void c64_enable(void)
{
    c64_interface(true);
    c64_reset(false);
}

// Turn the C64 cartridge interface off, and then force the C64 in reset state.
// Does not change EXROM/GAME state.
static void c64_disable(void)
{
    c64_interface(false);
    c64_reset(true);
}

static void c64_reset_config(void)
{
    c64_reset(true);

    // Set PD6 as open-drain if non-inverting
    if(!C64_RESET_INVERT) {
		GPIO_RESET->OTYPER |= (
		  (1UL<<C64_RESET_POS)
		);
	}

    // Set PA15 (or PD6) as output
    MODIFY_REG(GPIO_RESET->MODER, 
       (3UL << C64_RESET_POS*2),
       (1UL << C64_RESET_POS*2)
    );
}

/*************************************************
* Menu button and special button on PA4 & PA5
*************************************************/

static INLINE bool menu_button(void)
{
    return (GPIO_CONTROL->IDR & (1UL<<MENU_BTN_POS)) != 0;
}

static void menu_button_wait_release(void)
{
    while (menu_button());
}

static INLINE bool special_button(void)
{
    return (GPIO_CONTROL->IDR & (1UL<<SPECIAL_BTN_POS)) != 0;
}

static void special_button_wait_release(void)
{
    while (special_button());
}

void EXTI4_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR4)
    {
        c64_disable();
        restart_to_menu();
    }
}

static void button_config(void)
{
    // Make PA4 and PA5 (or PC4,PC13) input
    GPIO_CONTROL->MODER &= ~(
      (3UL<<MENU_BTN_POS*2)|
      (3UL<<SPECIAL_BTN_POS*2)
    );

    // Enable pull-down
    MODIFY_REG(GPIO_CONTROL->PUPDR, 
      (3UL<<MENU_BTN_POS*2)|
      (3UL<<SPECIAL_BTN_POS*2),
      (2UL<<MENU_BTN_POS*2)|
      (2UL<<SPECIAL_BTN_POS*2)
    );
    
    // Enable EXTI4 interrupt on PA4 (or PC4)
    MODIFY_REG(SYSCFG->EXTICR[1], SYSCFG_EXTICR2_EXTI4, MENU_BTN_INT_PORT);
    EXTI->IMR |= EXTI_IMR_MR4;

    // Rising edge trigger on PA4 (or PC4)
    EXTI->RTSR |= EXTI_RTSR_TR4;
    EXTI->FTSR &= ~EXTI_FTSR_TR4;

    // Enable EXTI4_IRQHandler, lowest priority
    EXTI->PR = EXTI_PR_PR4;
    NVIC_SetPriority(EXTI4_IRQn, 0x0f);
    NVIC_EnableIRQ(EXTI4_IRQn);
}

/*************************************************
* Configure C64 interface
*************************************************/
static void c64_interface_config(void)
{
    c64_address_config();
    c64_data_config();

    c64_crt_config();
    c64_irq_config();
    c64_reset_config();
    c64_clock_config();

    button_config();
}
