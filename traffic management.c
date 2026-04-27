/*
 * Smart Traffic Signal Control System with Emergency Vehicle Detection
 * Controller : STM32F407 Discovery (ARM Cortex-M4)
 * Author     : Arulmathi P
 * Description: 4-lane traffic signal system with RF-based emergency vehicle
 *              detection, GPS UART integration, and 16x2 LCD status display.
 *              Fully register-level — no HAL, no library dependency.
 *
 * PIN MAPPING:
 * ─────────────────────────────────────────────────────────
 * TRAFFIC SIGNALS (Output)
 *   Lane 1 : PD0 = RED, PD1 = YELLOW, PD2 = GREEN
 *   Lane 2 : PD3 = RED, PD4 = YELLOW, PD5 = GREEN
 *   Lane 3 : PD6 = RED, PD7 = YELLOW, PD8 = GREEN
 *   Lane 4 : PD9 = RED, PD10= YELLOW, PD11= GREEN
 *
 * RF RECEIVER (Input)
 *   PE0 = RF_IN (433MHz receiver output — HIGH when ambulance detected)
 *
 * GPS UART (UART2 — PA2=TX, PA3=RX @ 9600 baud)
 *
 * LCD (16x2, 4-bit mode)
 *   RS=PC0, EN=PC1, D4=PC2, D5=PC3, D6=PC4, D7=PC5
 * ─────────────────────────────────────────────────────────
 */

#include <stdint.h>

/* ─── Base Addresses ──────────────────────────────────────────────────────── */
#define PERIPH_BASE       0x40000000UL
#define AHB1_BASE         (PERIPH_BASE + 0x00020000UL)
#define APB1_BASE         (PERIPH_BASE + 0x00000000UL)

#define RCC_BASE          (AHB1_BASE  + 0x3800UL)
#define GPIOA_BASE        (AHB1_BASE  + 0x0000UL)
#define GPIOC_BASE        (AHB1_BASE  + 0x0800UL)
#define GPIOD_BASE        (AHB1_BASE  + 0x0C00UL)
#define GPIOE_BASE        (AHB1_BASE  + 0x1000UL)
#define USART2_BASE       (APB1_BASE  + 0x4400UL)

/* ─── Register Structs ────────────────────────────────────────────────────── */
typedef struct {
    volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR;
    volatile uint32_t IDR, ODR, BSRR, LCKR, AFR[2];
} GPIO_t;

typedef struct {
    volatile uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR;
} USART_t;

typedef struct {
    volatile uint32_t CR[3], CFGR, CIR;
    volatile uint32_t AHB1RSTR, AHB2RSTR, AHB3RSTR, RESERVED0;
    volatile uint32_t APB1RSTR, APB2RSTR, RESERVED1[2];
    volatile uint32_t AHB1ENR, AHB2ENR, AHB3ENR, RESERVED2;
    volatile uint32_t APB1ENR, APB2ENR;
} RCC_t;

#define RCC     ((RCC_t *)   RCC_BASE)
#define GPIOA   ((GPIO_t *)  GPIOA_BASE)
#define GPIOC   ((GPIO_t *)  GPIOC_BASE)
#define GPIOD   ((GPIO_t *)  GPIOD_BASE)
#define GPIOE   ((GPIO_t *)  GPIOE_BASE)
#define USART2  ((USART_t *) USART2_BASE)

/* ─── Signal Pin Bits (GPIOD ODR) ────────────────────────────────────────── */
/* Lane 1 */
#define L1_RED    (1U << 0)
#define L1_YEL    (1U << 1)
#define L1_GRN    (1U << 2)
/* Lane 2 */
#define L2_RED    (1U << 3)
#define L2_YEL    (1U << 4)
#define L2_GRN    (1U << 5)
/* Lane 3 */
#define L3_RED    (1U << 6)
#define L3_YEL    (1U << 7)
#define L3_GRN    (1U << 8)
/* Lane 4 */
#define L4_RED    (1U << 9)
#define L4_YEL    (1U << 10)
#define L4_GRN    (1U << 11)

/* All RED mask — default safe state */
#define ALL_RED   (L1_RED | L2_RED | L3_RED | L4_RED)

/* ─── Timing (simple loop delay @ ~168MHz) ───────────────────────────────── */
#define GREEN_DURATION   5000000UL   /* ~30 sec green */
#define YELLOW_DURATION  1000000UL   /* ~6  sec yellow */
#define EMERGENCY_HOLD   8000000UL   /* ~45 sec emergency green hold */

static void delay(volatile uint32_t count) {
    while (count--);
}

/* ─── Clock Enable ────────────────────────────────────────────────────────── */
static void clock_init(void) {
    /* Enable GPIOA, GPIOC, GPIOD, GPIOE, USART2 clocks */
    RCC->AHB1ENR |= (1U << 0)  /* GPIOA */
                  | (1U << 2)  /* GPIOC */
                  | (1U << 3)  /* GPIOD */
                  | (1U << 4); /* GPIOE */
    RCC->APB1ENR |= (1U << 17); /* USART2 */
}

/* ─── GPIO Init ───────────────────────────────────────────────────────────── */
static void gpio_init(void) {
    /* GPIOD PD0–PD11 → Output (traffic signals)
     * MODER: 01 = output for each pin (2 bits per pin) */
    GPIOD->MODER &= ~(0x00FFFFFFUL);  /* clear bits 0–23 */
    GPIOD->MODER |=  (0x00555555UL);  /* set 01 for PD0–PD11 */

    /* GPIOE PE0 → Input (RF receiver) — default MODER=00, no change needed */
    GPIOE->MODER &= ~(0x3U << 0);     /* PE0 = input */
    GPIOE->PUPDR &= ~(0x3U << 0);     /* no pull */

    /* GPIOA PA2=TX, PA3=RX → Alternate Function 7 (USART2) */
    GPIOA->MODER  &= ~((0x3U << 4) | (0x3U << 6));
    GPIOA->MODER  |=  ((0x2U << 4) | (0x2U << 6)); /* AF mode */
    GPIOA->AFR[0] &= ~((0xFU << 8)  | (0xFU << 12));
    GPIOA->AFR[0] |=  ((0x7U << 8)  | (0x7U << 12)); /* AF7 = USART2 */

    /* GPIOC PC0–PC5 → Output (LCD control + data pins) */
    GPIOC->MODER &= ~(0x00000FFFUL);
    GPIOC->MODER |=  (0x00000555UL); /* 01 for PC0–PC5 */
}

/* ─── UART2 Init @ 9600 baud (APB1=42MHz) ───────────────────────────────── */
static void uart_init(void) {
    /* BRR = fCK / baud = 42000000 / 9600 = 4375 = 0x1117 */
    USART2->BRR = 0x1117U;
    /* Enable USART, TX, RX */
    USART2->CR1 = (1U << 13) | (1U << 3) | (1U << 2);
}

/* Send one character via UART */
static void uart_send_char(char c) {
    while (!(USART2->SR & (1U << 7))); /* wait TXE */
    USART2->DR = (uint8_t)c;
}

/* Send string via UART */
static void uart_send_str(const char *s) {
    while (*s) uart_send_char(*s++);
}

/* Read one character from UART (blocking) */
static char uart_read_char(void) {
    while (!(USART2->SR & (1U << 5))); /* wait RXNE */
    return (char)USART2->DR;
}

/* ─── LCD (16x2, 4-bit mode) ─────────────────────────────────────────────── */
/* PC0=RS, PC1=EN, PC2=D4, PC3=D5, PC4=D6, PC5=D7 */

#define LCD_RS  (1U << 0)
#define LCD_EN  (1U << 1)
#define LCD_D4  (1U << 2)
#define LCD_D5  (1U << 3)
#define LCD_D6  (1U << 4)
#define LCD_D7  (1U << 5)

static void lcd_pulse_en(void) {
    GPIOC->ODR |=  LCD_EN;
    delay(500);
    GPIOC->ODR &= ~LCD_EN;
    delay(500);
}

/* Send 4 bits to LCD via D4–D7 */
static void lcd_send_nibble(uint8_t nibble) {
    /* Clear D4–D7 */
    GPIOC->ODR &= ~(LCD_D4 | LCD_D5 | LCD_D6 | LCD_D7);
    if (nibble & 0x1) GPIOC->ODR |= LCD_D4;
    if (nibble & 0x2) GPIOC->ODR |= LCD_D5;
    if (nibble & 0x4) GPIOC->ODR |= LCD_D6;
    if (nibble & 0x8) GPIOC->ODR |= LCD_D7;
    lcd_pulse_en();
}

/* Send command (RS=0) or data (RS=1) */
static void lcd_write(uint8_t value, uint8_t rs) {
    if (rs) GPIOC->ODR |=  LCD_RS;
    else    GPIOC->ODR &= ~LCD_RS;
    lcd_send_nibble(value >> 4);   /* high nibble first */
    lcd_send_nibble(value & 0x0F); /* low nibble */
    delay(2000);
}

#define lcd_cmd(c)  lcd_write(c, 0)
#define lcd_data(c) lcd_write(c, 1)

static void lcd_init(void) {
    delay(50000);
    /* 4-bit init sequence */
    lcd_send_nibble(0x3); delay(5000);
    lcd_send_nibble(0x3); delay(1000);
    lcd_send_nibble(0x3); delay(1000);
    lcd_send_nibble(0x2); /* switch to 4-bit */
    lcd_cmd(0x28); /* 4-bit, 2 lines, 5x8 font */
    lcd_cmd(0x0C); /* display on, cursor off */
    lcd_cmd(0x06); /* auto increment, no shift */
    lcd_cmd(0x01); /* clear display */
    delay(20000);
}

static void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? (0x80 + col) : (0xC0 + col);
    lcd_cmd(addr);
}

static void lcd_print(const char *s) {
    while (*s) lcd_data((uint8_t)*s++);
}

static void lcd_clear(void) {
    lcd_cmd(0x01);
    delay(20000);
}

/* ─── Traffic Signal Helpers ──────────────────────────────────────────────── */

/* All lanes RED — safe default */
static void all_red(void) {
    GPIOD->ODR = ALL_RED;
}

/* Set one lane GREEN, all others RED */
static void set_green(uint8_t lane) {
    uint32_t red_mask, grn_mask, yel_mask;

    const uint32_t reds[4]   = {L1_RED, L2_RED, L3_RED, L4_RED};
    const uint32_t greens[4] = {L1_GRN, L2_GRN, L3_GRN, L4_GRN};
    const uint32_t yellows[4]= {L1_YEL, L2_YEL, L3_YEL, L4_YEL};

    (void)yel_mask; /* used below */

    /* All RED first */
    GPIOD->ODR = ALL_RED;
    delay(200000); /* brief all-red gap */

    /* Active lane GREEN, others RED */
    red_mask = ALL_RED & ~reds[lane];   /* remove this lane's red */
    grn_mask = greens[lane];
    GPIOD->ODR = red_mask | grn_mask;
}

/* Transition lane from GREEN → YELLOW → RED */
static void set_yellow_then_red(uint8_t lane) {
    const uint32_t greens[4] = {L1_GRN, L2_GRN, L3_GRN, L4_GRN};
    const uint32_t yellows[4]= {L1_YEL, L2_YEL, L3_YEL, L4_YEL};
    const uint32_t reds[4]   = {L1_RED, L2_RED, L3_RED, L4_RED};

    /* Turn off green, turn on yellow */
    GPIOD->ODR &= ~greens[lane];
    GPIOD->ODR |=  yellows[lane];
    delay(YELLOW_DURATION);

    /* Turn off yellow, turn on red */
    GPIOD->ODR &= ~yellows[lane];
    GPIOD->ODR |=  reds[lane];
    delay(200000);
}

/* ─── RF Emergency Detection ──────────────────────────────────────────────── */
/* Returns 1 if ambulance RF signal detected on PE0 */
static uint8_t rf_emergency_detected(void) {
    return (GPIOE->IDR & (1U << 0)) ? 1U : 0U;
}

/* Emergency override — give green to lane that has ambulance
 * For simplicity: emergency always clears Lane 1 (can be extended
 * with multi-lane RF decoding using separate receiver pins) */
static void emergency_override(void) {
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("EMERGENCY DETECT");
    lcd_set_cursor(1, 0);
    lcd_print("Lane 1 CLEAR    ");

    uart_send_str("EMERGENCY: Lane 1 Green\r\n");

    /* Force Lane 1 GREEN, all others RED immediately */
    GPIOD->ODR = (ALL_RED & ~L1_RED) | L1_GRN;

    /* Hold green for emergency duration */
    delay(EMERGENCY_HOLD);

    /* Transition back */
    set_yellow_then_red(0);

    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Resuming Normal ");
    uart_send_str("Resuming normal cycle\r\n");
    delay(1000000);
}

/* ─── Main ────────────────────────────────────────────────────────────────── */
int main(void) {
    uint8_t lane;
    char gps_buf[64];
    uint8_t gps_idx;

    clock_init();
    gpio_init();
    uart_init();
    lcd_init();

    all_red();

    lcd_set_cursor(0, 0);
    lcd_print("Traffic System  ");
    lcd_set_cursor(1, 0);
    lcd_print("Initializing... ");
    delay(3000000);
    lcd_clear();

    uart_send_str("System Ready\r\n");

    while (1) {
        /* Cycle through 4 lanes */
        for (lane = 0; lane < 4; lane++) {

            /* Check RF before each lane switch */
            if (rf_emergency_detected()) {
                emergency_override();
                /* Restart cycle after emergency */
                lane = 0;
            }

            /* Set this lane GREEN */
            set_green(lane);

            /* Update LCD */
            lcd_clear();
            lcd_set_cursor(0, 0);
            lcd_print("Lane ");
            lcd_data('1' + lane);
            lcd_print(" : GREEN");
            lcd_set_cursor(1, 0);
            lcd_print("No Emergency    ");

            /* Send GPS status via UART */
            uart_send_str("Lane ");
            uart_send_char('1' + lane);
            uart_send_str(" GREEN — No Emergency\r\n");

            /* Hold green — check RF continuously during this time */
            volatile uint32_t t = GREEN_DURATION;
            while (t--) {
                if (rf_emergency_detected()) {
                    set_yellow_then_red(lane);
                    emergency_override();
                    lane = 0xFF; /* force restart after for-loop increment */
                    break;
                }
            }

            if (lane == 0xFF) { lane = 0; continue; }

            /* Green time over — go yellow then red */
            set_yellow_then_red(lane);
        }
    }

    return 0;
}#TRAFFICLIGHT INTELIGENT TRAFFIC SIGNAL WITH EMERGENCY OVERRIDE
