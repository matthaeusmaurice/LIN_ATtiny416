/*******************************************************************************
 * LEM copyright 2025, All rights reserved. For internal use only
 * 
 * File         : ATTiny416.c
 * Project      : HSU LIN Communication Prototype
 * Author       : ext_mmt
 * Description  : Read/Write EEPROM with HSU shunt parameters via LIN
 *
 * Created on 2025/07/03, 10:41
 * 
 ******************************************************************************/

/*******************************************************************************
 * 1. Headers and Global Definitions
 ******************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>

// --------- Constants ---------
#define LIN_BAUD                    19200 
#define LIN_FRAME_ID                0x20 
#define UART_BAUD_SELECT            ((F_CPU / (16UL * LIN_BAUD)) - 1)
#define PARAM_COUNT                 8
#define SEED_KEY_LENGTH             4
#define EEPROM_DEFAULT_VALUE        {2025, 1, 1, 0, 0, 0, 0, 0} 

// --------- Modes ---------
typedef enum 
{
    MODE_FACTORY = 0,
    MODE_CUSTOMER = 1
} OperationMode_t;

// --------- Parameter IDs ---------
enum 
{
    PARAM_YEAR = 0x000,
    PARAM_MONTH = 0x001,
    PARAM_DAY = 0x010,
    PARAM_MODULE_ID = 0x011,
    PARAM_R25 = 0x100,
    PARAM_A = 0x101, 
    PARAM_B = 0x110,
    PARAM_C = 0x111
};

// --------- Status/Control Flags ---------
volatile OperationMode_t currentMode = MODE_FACTORY;
volatile bool parameterAccessGranted = false;

/*******************************************************************************
 * 2. Data Structures
 ******************************************************************************/

typedef struct 
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint32_t module_id;
    uint32_t r25;
    uint32_t a;
    uint32_t b;
    uint32_t c;
} DeviceParams_t;

// LIN frame structure 
typedef struct 
{
    uint8_t crc;
    uint8_t reserved[2];
    uint8_t payload[4];
    uint8_t status;
} LINFrame_t;

// Global parameter instance
DeviceParams_t g_params;

/*******************************************************************************
 * 3. CRC-8 LIN
 ******************************************************************************/

// Polynomial: x^8 + x^2 + x + 1 => 0x07
uint8_t lin_crc8(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc &= 0x80)
            {
                crc = (crc << 1) ^ 0x07;
            }
            else 
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/*******************************************************************************
 * 4. EEPROM Read/Write
 ******************************************************************************/

#define EEPROM_ADDR ((uint8_t*)0x00)

void save_params_to_eeprom(DeviceParams_t *p)
{
    eeprom_write_block((const void *)p, EEPROM_ADDR, sizeof(DeviceParams_t));
}

void load_params_from_eeprom(DeviceParams_t *p)
{
    eeprom_read_block((void *)p, EEPROM_ADDR, sizeof(DeviceParams_t));
    
    // Check if EEPROM is uninitialized and set default values 
    if (p->year == 0xFFFF)
    {
        DeviceParams_t default_params = EEPROM_DEFAULT_VALUE;
        save_params_to_eeprom(&default_params);
        *p = default_params;
    }
}

/*******************************************************************************
 * 5. UART/LIN Initialization
 ******************************************************************************/

void uart_init(void)
{
    USART0.BAUD = UART_BAUD_SELECT;
    USART0.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
    USART0.CTRLA = USART_RXCIE_bm;
}

void uart_send(uint8_t data)
{
    while(!(USART0.STATUS & USART_DREIF_bm));
    USART0.TXDATAL = data;
}

uint8_t uart_receive(void)
{
    while (!(USART0.STATUS & USART_RXCIF_bm));
    return USART0.RXDATAL;
}

/*******************************************************************************
 * 6. LIN Frame Construction and Dispatch
 ******************************************************************************/

void send_lin_frame(LINFrame_t *frame)
{
    uint8_t raw[8];
    raw[0] = frame->crc;
    raw[1] = 0x00; // reserved
    raw[2] = 0x00; // reserved
    for (int i = 0; i < 4; i++)
        raw[3 + i] = frame->payload[i];
    raw[7] = frame->status;
    
    for (int i = 0; i < 8; i++)
        uart_send(raw[i]);
}

/*******************************************************************************
 * 7. Seed & Key Challenge Authentication
 ******************************************************************************/

uint8_t g_seed[SEED_KEY_LENGTH] = {0x33, 0x66, 0x99, 0xCC}; // example
uint8_t g_key[SEED_KEY_LENGTH];

void generate_seed(void)
{
    for (uint8_t i = 0; i < SEED_KEY_LENGTH; i++)
        g_seed[i] = i * 0x2A + 1; // Simple pseudo-random
}

bool validate_key(uint8_t *key)
{
    for (uint8_t i = 0; i < SEED_KEY_LENGTH; i++)
    {
        if (key[i] != (uint8_t)(~g_seed[i])) return false;
    }
    return true;
}

/*******************************************************************************
 * 8. Mode Switching and Command Handling
 ******************************************************************************/

void handle_command(uint8_t cmd, uint8_t *payload)
{
    if (cmd < 1 || cmd > 5) return;
    
    switch (cmd)
    {
        case 1: // Factory write 
            save_params_to_eeprom(&g_params);
            break;
        case 2: // Switch to customer
            currentMode = MODE_CUSTOMER;
            break;
        case 3: // Customer read
            if (parameterAccessGranted) send_lin_frame((LINFrame_t*)&g_params);
            break;
        case 4: // Receive key
            if (validate_key(payload)) parameterAccessGranted = true;
            break;
        case 5: // Send seed
            generate_seed();
            send_lin_frame((LINFrame_t*)g_seed);
            break;
        default:
            break;
    }
}

/*******************************************************************************
 * 9. CRC Scan and Integrity Check
 ******************************************************************************/

void crcscan_init(void)
{
    CRCSCAN.CTRLB = (1 << 0);   // Select source (FLASH)
    CRCSCAN.CTRLA = (1 << 7);   // Enable
}

bool check_crc_ok(void)
{
    return (CRCSCAN.STATUS & 0x01); // CRC OK bit
}

/*******************************************************************************
 * 10. Fallback Functions for Safety
 ******************************************************************************/

void fallback_safe_reset(void)
{
    cli();
    WDT.CTRLA = WDT_PERIOD_8CLK_gc; // enable watchdog with short timeout
    while (1); // wait for reset
}

void fallback_timeout_handler(void)
{
    // Log fault
}

/*******************************************************************************
 * 11. Interrupts
 ******************************************************************************/

ISR(USART0_RXC_vect)
{
    static uint8_t buffer[8];
    static uint8_t index = 0;
    
    if (index < sizeof(buffer))
    {
        buffer[index++] = USART0.RXDATAL;
    }
    
    if (index >= sizeof(buffer))
    {
        index = 0;
        uint8_t crc = lin_crc8(&buffer[3], 4);
        if (crc == buffer[0])
        {
            handle_command(buffer[7] & 0x07, &buffer[3]);
        }
    }
}

/*******************************************************************************
 * 12. Main Program Entry
 ******************************************************************************/

void system_init(void)
{
    cli();
    uart_init();
    crcscan_init();
    load_params_from_eeprom(&g_params);
    sei();
}

int main(void)
{
    system_init();
    
    while (1)
    {
        if (!check_crc_ok())
        {
            fallback_safe_reset();
        }
        
        _delay_ms(200);
    }
}