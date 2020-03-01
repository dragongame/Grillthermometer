/*
 *
 *
 *
 *
 * 2do:
 * 
 *  - Temperaturen messen, d.h. Umrechnung von A/D-Wert in Temperatur
 *
 *  - Abfrage der Drehschalter und Drehregler in main() zusammen mit Messzyklus durchführen
 *     - g_displayRange anhand des Bereichsschalters setzen
 *     - g_operationMode anhand des Drehschalter setzen
 *     - anhand der ADC-Werte vom Drehpoti Drehbwegungen detektieren und ggf. reagieren
 *
 *  - Sound von SD-Card einlesen und abspielen
 *
 *  - spezielle Sounds bei speziellen Anlässen spielen
 *
 *  - Anpassung an neue Hardware sobald fertig: Bereichseinstellung, Kalibrierung, Taster auslesen
 *
 *  - Display-Modus als Variable einführen und entsprechende Variablen definieren:
 *    -> setzt "g_displayMode", "g_displayScale" auf anzuzeigendenden Wert (enums definieren)
 *    -> setzt Flag "g_updateDisplay", um in der main() die richtigen Werte auf den DAC zu geben
 *
 *  - Ansteuerung Bluetooth-Modul (serielle umbiegen, Baudrate anpassen, AT-Befehle in Startroutine)
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/sleep.h>

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#include "usart.h"
//#include "sd-card.h"
//#include <stdio.h>
//#include <cs50.h>

#define WDT_Reset() asm("wdr")

#define ADC_BUFFER_SIZE 4
#define ADC_TEMP_CHANNELS 1
#define COMMAND_LENGTH 12

typedef enum { TempAlarm_None, TempAlarm_Over, TempAlarm_Under, TempAlarm_Window} Temperaturalarm;
typedef enum { noFlag = 0, measureTempFlag = 1, sendMessageFlag = 2, alarmTempFlag = 4, alarmTimerFlag = 8, alarmActiveFlag = 16 } Flags;
typedef enum { displayFixedValue, displayOverrange, displayUnderrange, displayAttention} DisplayMode;
typedef enum { displayRangeLow, displayRangeHigh } DisplayRange;
typedef enum { modeTemp1, modeTemp2, modeTempAlarm1, modeTempAlarm2, modeTimer } OperationMode;

volatile Flags          g_flags = noFlag;               // Flags, die das Hauptprogramm kontrollieren
volatile int16_t        g_timerAlarm = -1;              // Zeitalarm: wenn >0, dann Minuten bis Alarm
volatile uint8_t        g_timerAlarmSecs = 0;           // Zeitalarm, Sekundenzähler
volatile DisplayRange   g_displayRange = displayRangeLow; // 0–50°-Skala vs. 0–200°-Skala
volatile OperationMode  g_operationMode = modeTemp1;    // Drehschalter

volatile DisplayMode g_displayMode = displayFixedValue;

volatile Temperaturalarm g_tempAlarm[2] = {TempAlarm_None, TempAlarm_None}; // Kanal I & II : Warnmodus
volatile int16_t    g_tempAlarmMin[2]  = {0,0};
volatile int16_t    g_tempAlarmMax[2]  = {250,250};

uint8_t  g_adcIndex = 0;                                  // Index im Ringpuffer der ADC-Werte
uint16_t g_adc_vals[ADC_TEMP_CHANNELS][ADC_BUFFER_SIZE];  // ADC-Wert vom Temperatursensor


const char g_separators[] = " ,\n"; // for argument parsing

const char f_OK_msg[] PROGMEM = "OK.\n";
const char f_value_err_msg[] PROGMEM = "VALUE OUT OF RANGE ERROR.\n";


// reads signature byte for accessing µC's calibration data
uint8_t readSignatureByte(uint16_t Address) {
    NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
    uint8_t Result;
    Result = pgm_read_byte((uint8_t *)Address);
    NVM_CMD = NVM_CMD_NO_OPERATION_gc;
    return Result;
}


// init RTC with overflow IRQ every 'secs' seconds
static inline void rtc_init(uint8_t secs) {
    /* turn on internal 32kHz oscillator */
    CCP = CCP_IOREG_gc;
    OSC.CTRL |= OSC_RC32KEN_bm;
    do {
        /* Wait for the 32kHz oscillator to stabilize. */
    } while ((OSC.STATUS & OSC_RC32KRDY_bm) == 0);
    
    /* use internal 32 kHz oscillator to generate 1024 Hz */
    CCP = CCP_IOREG_gc;			      // avoid I/O glitches by 4 cycle block
    CLK.RTCCTRL =  CLK_RTCSRC_RCOSC_gc | CLK_RTCEN_bm;
    
    /* set RTC prescaler. */
    do {
    } while (( RTC.STATUS & RTC_SYNCBUSY_bm) == RTC_SYNCBUSY_bm);
    RTC.CTRL = RTC_PRESCALER_DIV1024_gc; // count seconds
    do {  } while (( RTC.STATUS & RTC_SYNCBUSY_bm) == RTC_SYNCBUSY_bm);
    RTC.CNT = 0;
    do {  } while (( RTC.STATUS & RTC_SYNCBUSY_bm) == RTC_SYNCBUSY_bm);
    RTC.PER = secs;   // overflow
    do {  } while (( RTC.STATUS & RTC_SYNCBUSY_bm) == RTC_SYNCBUSY_bm);
    RTC.COMP = 0xffff;
    do {  } while (( RTC.STATUS & RTC_SYNCBUSY_bm) == RTC_SYNCBUSY_bm);
    RTC.INTCTRL = (RTC.INTCTRL & ~( RTC_COMPINTLVL_gm | RTC_OVFINTLVL_gm )) | RTC_OVFINTLVL_LO_gc;
}

// retrieves command from serial interface (\n-terminated line of text), once \n has been detected
char* getCommand(void) {
    static char command[32]; // no stack allocation, so memory remains valid
    uint8_t     i;
    char        c = usart_getc(&USART_data);

    for (i=0; i<sizeof(command) && c != 0x0d && c != 0x0a && c != EOF; i+=1) {
        command[i] = c;
        c = usart_getc(&USART_data);
    }
    command[i] = 0x00; // end of string

    return command;
}

// initialisiere DAC zur Ansteuerung von Messinstrument und Audioverstärker
void dac_init(void) {
    // set DAC pins to output (not sure if neccessary)
    PORTB.DIRSET = 12;
    PORTB.OUTCLR = 12;
    
    DACB.CTRLA = DAC_CH1EN_bm | DAC_CH0EN_bm | DAC_ENABLE_bm; // enable CH0 and CH1
    DACB.CTRLB = DAC_CHSEL_DUAL_gc; // select both channels
    DACB.CTRLC = DAC_REFSEL_INT1V_gc; // use internal 1 Volt reference
    
    // TIMCTRL does not exist with A4U MCUs
    //DACB.TIMCTRL = DAC_CONINTVAL_2CLK_gc | DAC_REFRESH_16CLK_gc; // allow 1.5µs for DAC conversion, refresh S/H after 16 clocks
    // set calibration
    DACB.CH0GAINCAL   = readSignatureByte(offsetof(NVM_PROD_SIGNATURES_t, DACB0GAINCAL));
    DACB.CH0OFFSETCAL = readSignatureByte(offsetof(NVM_PROD_SIGNATURES_t, DACB0OFFCAL));
    DACB.CH1GAINCAL   = readSignatureByte(offsetof(NVM_PROD_SIGNATURES_t, DACB1GAINCAL));
    DACB.CH1OFFSETCAL = readSignatureByte(offsetof(NVM_PROD_SIGNATURES_t, DACB1OFFCAL));
    
    DACB.CH0DATA = 0;
    DACB.CH1DATA = 0;
}

// initialisiert IO-Ports des Controllers und sonstige Komponenten
void system_init(void) {
    // Watchdog einschalten
    uint8_t temp = WDT_ENABLE_bm | WDT_CEN_bm | WDT_WPER_2KCLK_gc;
    CCP = CCP_IOREG_gc;
    WDT.CTRL = temp;
    while( WDT.STATUS & WDT_SYNCBUSY_bm ) {};
    
    // Audioverstärker ausschalten
    PORTE.OUTCLR = 1;
    PORTE.DIRSET = 1;
    
    // Bedienschalter (Pullups einschalten)
    PORTCFG.MPCMASK = 1 | 2 | 16;
    PORTC.PIN0CTRL  = PORT_OPC_PULLUP_gc;
    PORTCFG.MPCMASK = 1 | 2 | 4 | 8 | 16 | 32;
    PORTD.PIN0CTRL  = PORT_OPC_PULLUP_gc;
    
    // MUXe im Messkanal konfigurieren
    PORTA.OUTSET = 64; // 10k default
    PORTA.OUTCLR = 128;
    PORTA.DIRSET = 64 | 128;
    
    PORTB.OUTCLR = 1 | 2; // Temp #0 default
    PORTB.DIRSET = 1 | 2;
    
    // Timer/Counter D0 für Quadraturdekodierung des Drehknopfes vorbereiten

    // 3. set quadrature decoding as the event action for a timer/counter.
    // 8. Select event channel n as the event source for the timer/counter.
    //     Set the period register of the timer/counter to ('line count' * 4 - 1), the line count of the quadrature encoder.
    //     Enable the timer/counter without clock prescaling.

    // configure C:0 and C:1 for input, pull-up, and low-level sense
    PORTC.DIRCLR = 3;
    PORTCFG.MPCMASK = 3; // configure pin 0 + 1 at once
    PORTC.PIN0CTRL  = PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
    // attach event channel #0 to C:0, enable quadrature decoding and filtering
    EVSYS.CH0MUX = EVSYS_CHMUX_PORTC_PIN0_gc; //
    EVSYS.CH0CTRL = EVSYS_QDIRM_00_gc | EVSYS_DIGFILT_6SAMPLES_gc | EVSYS_QDEN_bm;
    // attach timer D0 to quadrature decoder
    TCD0.CTRLA = TC_CLKSEL_DIV1_gc;
    TCD0.CTRLD = TC_EVACT_QDEC_gc | TC_EVSEL_CH0_gc;
}

// initialisiere ADC: Referenz extern von Port A, Kanal 0 für Messung des ext. Spannungsteilers (NTC)
void adc_init(void) {
    PORTCFG.MPCMASK = 0b000111111;                  // disable digital input buffer on analog inputs
    PORTA.PIN0CTRL  = PORT_ISC_INPUT_DISABLE_gc;
    
    ADCA.CTRLA = ADC_ENABLE_bm;
    ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc | ADC_CONMODE_bm;   // 12 bit conversion, signed, not free running
    ADCA.REFCTRL = ADC_REFSEL_AREFA_gc;     // external reference Port A
    ADCA.PRESCALER = ADC_PRESCALER_DIV4_gc;
    // set ADC calibration
    ADCA.CALL = readSignatureByte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0)); //ADC Calibration Byte 0
    ADCA.CALH = readSignatureByte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1)); //ADC Calibration Byte 1
    
    ADCA.CH0.CTRL    = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc;
    ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN1_gc | ADC_CH_MUXNEG_PIN3_gc;
    ADCA.CH0.INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;
    
    ADCA.CH0.CTRL |= ADC_CH_START_bm; // trigger conversion since first conversion is usually corrupted
}

// reads value from ADC channel #0
// FIXME: noise canceling beim ADC durch sleep möglich? (Daten dann per ISR empfangen)
int16_t adc_measureTemp(uint8_t channel) {
    if (channel < 4) {
        ADCA.REFCTRL = ADC_REFSEL_AREFA_gc;     // external reference Port A
        PORTB.OUT = (PORTB.OUT & ~3) | channel;
        ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN1_gc | ADC_CH_MUXNEG_PIN3_gc;
    } else {
        ADCA.REFCTRL = ADC_REFSEL_INT1V_gc;
        ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN2_gc | ADC_CH_MUXNEG_PIN3_gc;
    }
    ADCA.CH0.INTFLAGS |= ADC_CH_CHIF_bm;    // clear interrupt flag
    ADCA.CH0.CTRL |= ADC_CH_START_bm;       // start conversion
    while (!(ADCA.CH0.INTFLAGS & ADC_CH_CHIF_bm)) {}    // wait for conversion to complete
    return ADCA.CH0.RES;
}



/*
 const uint16_t sinTab[256] = { 512, 525, 537, 550, 562, 575, 587, 599, 612, 624, 636, 648, 660, 672, 684, 696, 708, 719, 730, 742, 753, 764, 775, 785, 796, 806, 816, 826, 836, 846, 855, 864, 873, 882, 891, 899, 907, 915, 922, 930, 937, 944, 950, 957, 963, 968, 974, 979, 984, 989, 993, 997, 1001, 1004, 1008, 1011, 1013, 1015, 1017, 1019, 1021, 1022, 1022, 1023, 1023, 1023, 1022, 1022, 1021, 1019, 1017, 1015, 1013, 1011, 1008, 1004, 1001, 997, 993, 989, 984, 979, 974, 968, 963, 957, 950, 944, 937, 930, 922, 915, 907, 899, 891, 882, 873, 864, 855, 846, 836, 826, 816, 806, 796, 785, 775, 764, 753, 742, 730, 719, 708, 696, 684, 672, 660, 648, 636, 624, 612, 599, 587, 575, 562, 550, 537, 525, 512, 499, 487, 474, 462, 449, 437, 425, 412, 400, 388, 376, 364, 352, 340, 328, 316, 305, 294, 282, 271, 260, 249, 239, 228, 218, 208, 198, 188, 178, 169, 160, 151, 142, 133, 125, 117, 109, 102, 94, 87, 80, 74, 67, 61, 56, 50, 45, 40, 35, 31, 27, 23, 20, 16, 13, 11, 9, 7, 5, 3, 2, 2, 1, 1, 1, 2, 2, 3, 5, 7, 9, 11, 13, 16, 20, 23, 27, 31, 35, 40, 45, 50, 56, 61, 67, 74, 80, 87, 94, 102, 109, 117, 125, 133, 142, 151, 160, 169, 178, 188, 198, 208, 218, 228, 239, 249, 260, 271, 282, 294, 305, 316, 328, 340, 352, 364, 376, 388, 400, 412, 425, 437, 449, 462, 474, 487, 499 };
 
 void beep(void) {
 for (uint8_t l=0; l<100; l++) {
 for (uint16_t i=0; i<=255; i++) { // Ist GCC clever genug für uint8_t als Schleifenvariable?
 DACB.CH0DATA = sinTab[i];
 _delay_us(1);
 }
 }
 }
 */

void beep(void) {
    PORTE.OUTSET = 1; // turn on audio amp
    _delay_ms(200);   // wait for amp to setlle
    for (uint8_t i=0; i<250; i++) {
        DACB.CH0DATA = 2048-30 ; // toggle output around 0.5V
        _delay_ms(1);
        DACB.CH0DATA = 2048+30;
        _delay_ms(1);
    }
    PORTE.OUTCLR = 1;
}

/*
 signed, differential, 12 Bit
 1830 -- 21°C
 
 unsigned, singleended, 12 Bit
 4010 -- 16°C ??
 3820 -- 23°C
 3450 -- 35°C
 2960 -- 49°C
 3110 -- 44°C
 */

// gibt Status über Temperaturalarm für einen Kanal über serielle Schnittstelle aus
void reportTemperatureAlarm(uint8_t channel) {
   usart_puts_P(&USART_data, "\nTemperaturalarm Kanal ");
   usart_putc(&USART_data, '0' + channel);
   usart_putc(&USART_data, ':');
   Temperaturalarm mode = g_tempAlarm[channel];
   int16_t minTemp = g_tempAlarmMin[channel];
   int16_t maxTemp = g_tempAlarmMax[channel];
    
   if (mode == TempAlarm_None) {
        usart_puts_P(&USART_data, "aus");
    } else if (mode == TempAlarm_Over) {
       usart_puts_P(&USART_data, "maximal ");
       usart_putDec(&USART_data, maxTemp);
    } else if (mode == TempAlarm_Under) {
       usart_puts_P(&USART_data, "minimal ");
       usart_putDec(&USART_data, minTemp);
    } else { // mode == TempAlarmWindow
        usart_puts_P(&USART_data, "Fenster ");
        usart_putDec(&USART_data, minTemp);
        usart_putc(&USART_data, ' ');
        usart_putDec(&USART_data, maxTemp);
    }
    usart_putc(&USART_data, '\n');
}

// writes value to DAC channel connected to meter (value taken from strtok())
// mainly intended for calibration, but who knows
void setDisplayDAC(void) {
    char* value = strtok(NULL, g_separators);
    if (value) {
        int d = atoi(value);
        if (d >= 0 && d < 4096) {
            DACB.CH1DATA = d;
            usart_puts_p(&USART_data, f_OK_msg);
        } else {
            usart_puts_p(&USART_data, f_value_err_msg);
        }
    }
}

// ADC-Kanäle auslesen
void readADC(void) {
    uint8_t channel;
    for (channel=0; channel<ADC_TEMP_CHANNELS; channel++) {
        uint16_t adc = adc_measureTemp(channel);
        usart_putDec(&USART_data, adc);
        usart_putc(&USART_data, ' ');
    }
    usart_putc(&USART_data, '\n');
}

// Weckalarm zurücksetzen
void cancelTimer(void) {
    g_timerAlarm = -1;
    usart_puts_p(&USART_data, f_OK_msg);
}

// Weckalarm setzen: Parameter Zeit in Minuten
void setTimer(void) {
    int dur = atoi(strtok(NULL, g_separators));
    if (dur > 0 && dur <= 250) {
        g_timerAlarmSecs = 60;
        g_timerAlarm = dur;
        usart_puts_p(&USART_data, f_OK_msg);
    } else {
        usart_puts_p(&USART_data, f_value_err_msg);
    }
}

// Temperaturalarm setzen: setTempAlarm Kanal Typ Temp1 [Temp2]
void setTempAlarm(void) {
    int channel = atoi(strtok(NULL, g_separators));
    if (channel ==1 || channel==2) {
        char* type = strtok(NULL, g_separators);
        channel -= 1; // auf Array-Index bringen
        if (strcasecmp(type,"off")==0) {
            g_tempAlarm[channel] = TempAlarm_None;
        } else if (strcasecmp(type,"over")==0) {
            int maxTemp = atoi(strtok(NULL, g_separators));
            g_tempAlarm[channel] = TempAlarm_Over;
            g_tempAlarmMax[channel] = maxTemp;
        } else if (strcasecmp(type,"under")==0) {
            int minTemp = atoi(strtok(NULL, g_separators));
            g_tempAlarm[channel] = TempAlarm_Under;
            g_tempAlarmMin[channel] = minTemp;
        } else if (strcasecmp(type,"window")==0) {
            int minTemp = atoi(strtok(NULL, g_separators));
            int maxTemp = atoi(strtok(NULL, g_separators));
            g_tempAlarm[channel] = TempAlarm_Window;
            g_tempAlarmMax[channel] = maxTemp;
            g_tempAlarmMin[channel] = minTemp;
        } else {
            usart_puts_P(&USART_data, "?Invalid Alarm Type Error.\n");
        }
    } else {
        usart_puts_P(&USART_data, "?Invalid Channel Error.\n");
    }
}

// "ls" auf der SD-Karte
void SD_ls(void) {
    //if (sd_ls() == 0) usart_puts_p(&USART_data, f_OK_msg);
}

// cat <FILE> auf der SD-Karte
void SD_cat(void) {
    /*
    char *filename = strtok(NULL, g_separators);
    if (filename) {
        if (sd_catFile(filename) == 0) usart_puts_p(&USART_data, f_OK_msg);
    } else {
        usart_puts_P(&USART_data, "?MISSING FILENAME\n");
    }
     */
}

// aktiven Alarm zurücksetzen
void cancelAlarm(void) {
    cli();
    g_flags &= ~alarmActiveFlag;
    sei();
    usart_puts_p(&USART_data, f_OK_msg);
}

void softwareReset(void) {
    CCP = CCP_IOREG_gc;
    RST.CTRL = 1;
}

// stellt HMUX und LMUX ein (fuers Debuggen/Kalibrieren)
// um diese Methode anwenden zu können, darf adc_measureTemp() nicht in der main() ausgeführt werden!
void setMUX(void) {
    int lmux = atoi(strtok(NULL, g_separators)); // Parameter parsieren
    int hmux = atoi(strtok(NULL, g_separators));

    if (lmux >= 0 && lmux<4 && hmux>=0 && hmux<4) {
        PORTA.OUT = (PORTA.OUT & 0b00111111) | (((uint8_t) hmux) << 6); // HMUX einstellen
        PORTB.OUT = (PORTB.OUT & 0x03) | ((uint8_t) lmux); // LMUX einstellen
        usart_puts_p(&USART_data, f_OK_msg);
    } else {
        usart_puts_p(&USART_data, f_value_err_msg);
    }
}

// wie readADC(), aber mit direkter Kontrolle der MUXe (adc_measureTemp() darf nicht in der Hauptschleife aufgerufen werden, um Einstellungen aus setMUX() nicht zu verlieren!)
void getADC(void) {
    int adc_mux = atoi(strtok(NULL, g_separators)); // Parameter parsieren: MUXe des internen ADCs im differentiellen Modus
    uint8_t saved_mux = ADCA.CH0.MUXCTRL;   // Einstellung des MUX sichern
    ADCA.CH0.MUXCTRL = adc_mux;
    ADCA.CH0.CTRL |= ADC_CH_START_bm;       // start conversion
    while (!(ADCA.CH0.INTFLAGS & ADC_CH_CHIF_bm)) {}    // wait for conversion to complete
    int res = ADCA.CH0.RES;

    ADCA.CH0.MUXCTRL = saved_mux;
    usart_puts_P(&USART_data, "ADC (MUX=");
    usart_putDec(&USART_data, adc_mux);
    usart_putc(&USART_data, ')');
    usart_putc(&USART_data, '=');
    usart_putDec(&USART_data, res);
    usart_putc(&USART_data, '\n');
}

// Kommandointerpreter

// Datenstruktur für Befehle: Name und Funktion
typedef struct {
    const char name[COMMAND_LENGTH];
    void (*function)(void);
} Command;

// Array aller Befehle im Flashspeicher
const Command remoteCommands[] PROGMEM = {{"beep", &beep}, {"DAC", &setDisplayDAC}, {"ADC", &readADC}, {"cancelTimer", &cancelTimer}, {"setTempAlarm", &setTempAlarm}, {"ls", &SD_ls}, {"cat", &SD_cat}, {"setTimer", &setTimer}, {"cancelAlarm", &cancelAlarm}, {"reset", &softwareReset}, {"setMUX", &setMUX}, {"getADC", &getADC}};

// sucht passenden Befehl zu Befehlszeile, Parsieren der Parameter ist Sache der ausführenden Funktion
void handleCommand(char* commandLine) {
    char* command = strtok(commandLine, g_separators); // Parsieren des Formates CMD Par1 Par2 ... \n
    uint8_t i;
    const Command *cmd_P = remoteCommands;
    Command cmd;

    if (!*command) return;
    for (i=0; i<sizeof(remoteCommands)/sizeof(Command); i++) {
        memcpy_P(&cmd, cmd_P,sizeof(Command)); // retrieve command from flash memory
        if (strcasecmp(command, cmd.name) == 0) {
            (cmd.function)();
            return;
        } else {
            cmd_P++;
        }
    }
    usart_puts_P(&USART_data, "?Syntax Error.\n");
}


// initialisiert Timer/Count 0 auf Port C für Overflow-IRQ 4x pro Sekunde
void timer_init(void) {
    TCC0.INTCTRLA = TC_OVFINTLVL_LO_gc;
    TCC0.INTCTRLB = 0x00;
    TCC0.CTRLB = 0x00;
    TCC0.CTRLC = 0x00;
    TCC0.CTRLD = 0x00;
    TCC0.CTRLE = 0x00;
    TCC0.CNT = 0;
    TCC0.PER = 62500;
    TCC0.CTRLA = TC_CLKSEL_DIV8_gc;
}

// avr-size -C --mcu=xmega32 thermo.elf

OperationMode readOperationMode(void) {
    uint8_t pins = PORTD.IN;
    g_displayRange = (pins & 32) ? displayRangeLow : displayRangeHigh;
    switch (pins & 31) {
        case 1: return modeTemp1;
        case 2: return modeTemp2;
        case 4: return modeTempAlarm1;
        case 8: return modeTempAlarm2;
        case 16: return modeTimer;
    }
    return modeTemp1; // das sollte nicht passieren...
}

// bestimmt die Temperatur eines Kanals anhand der gepufferten Werte
int16_t adc_to_temperature(uint8_t tempChannel) {
    uint16_t adc_avg;
    uint8_t i;

    if (tempChannel >= ADC_TEMP_CHANNELS) return 0;
    adc_avg = g_adc_vals[tempChannel][0];
    for(i=1; i<ADC_BUFFER_SIZE; i++) {
        adc_avg += g_adc_vals[tempChannel][i];
    }
    adc_avg /= ADC_BUFFER_SIZE;
    // FIXME: ADC to temp converseion goes here...
    
    return adc_avg;
}

// sets display to 'val', showing over-/underrange if value doesn't fit with range
void displayValue(uint16_t val) {
    if (val < 0) {
        g_displayMode = displayUnderrange;
    } else if (g_displayRange == displayRangeLow) {
        if (val > 100) {
            g_displayRange = displayOverrange;
        } else {
            g_displayMode = displayFixedValue;
            DACB.CH1DATA = (val*41)-(val/20); // magic calibration
        }
    } else {
        if (val > 250) {
            g_displayMode = displayOverrange;
        } else {
            g_displayMode = displayFixedValue;
            DACB.CH1DATA = (val*16)+(val/3); // eigentlich x 16.38
        }
    }
}

#define MODE_BUF_COUNT 4

// Hauptprogramm: System initialisieren und auf in Interrupts erzeugte Nachrichten reagieren
int __attribute__((OS_main)) main(void) {
    uint8_t         i,j;                          // fuer duett un datt
    OperationMode   current_mode[MODE_BUF_COUNT]; // letzte Werte des Mode-Schalters, um Kontaktprellen zu filtern
    uint8_t         mode_idx = 0;                 // Index fuer Ringpuffer current_mode []
    
    // Array der ADC-Messungen für gleitenden Mittelwert initialisieren
    // Nicht wirklich notwendig...
    memset(g_adc_vals, 0, sizeof(g_adc_vals));
    
    // System und Controller initialisieren
    system_init();  // ext. Hardwareanbindung einstellen
    usart_init();   // Initialisiere USART mit der Verbindung zur USB-Brücke
    adc_init();     // ADC für Temperaturmessung mit ext. Referenz
    dac_init();     // DAC für Anzeigeinstrument und Ton
    rtc_init(1);    // per RTC jede Sekunde IRQ auslösen (triggert Übertragung der Messdaten, bedient Wecker)
    timer_init();   // timer-IRQ 4x pro Sekunde auslösen (triggert Messungen)
    sei();
    WDT_Reset();

    // Los geht's!
    usart_puts_P(&USART_data, "\nRESET: ");
    i = RST.STATUS;
    if (i & 32) usart_puts_P(&USART_data, "software ");
    if (i & 16) usart_puts_P(&USART_data, "PDI ");
    if (i & 8) usart_puts_P(&USART_data, "watchdog ");
    if (i & 4) usart_puts_P(&USART_data, "brown-out ");
    if (i & 2) usart_puts_P(&USART_data, "external ");
    if (i & 1) usart_puts_P(&USART_data, "power-on");
    usart_putc(&USART_data, '\n');
    RST.STATUS = 63;

//    sd_init();      // init SD card
    
    WDT_Reset();
    set_sleep_mode(SLEEP_MODE_IDLE);
    beep();
    WDT_Reset();
    
    // Batteriespannung nach dem Einschalten kurz anzeigen
    int bat = adc_measureTemp(4);
    usart_putDec(&USART_data, bat);
    bat = 3*bat + (bat*2)/3; // magic conversion to 0..100%
    DACB.CH1DATA = bat;
    usart_putc(&USART_data, '\n');
    _delay_ms(750);
    WDT_Reset();

    // eigentliches Hauptprogramm: auf in den ISRs gesetzte Flags reagieren
    while (1) {
        WDT_Reset();
        if (g_flags & measureTempFlag) {         // Flag, dass es wieder Zeit ist die Temperatur zu messen
            cli();
            g_flags &= ~measureTempFlag;
            sei();
            
            for (i=0; i<ADC_TEMP_CHANNELS; i++) {
                g_adc_vals[i][g_adcIndex] = adc_measureTemp(i);
            }
            g_adcIndex +=1;
            if (g_adcIndex == ADC_BUFFER_SIZE) g_adcIndex = 0;
            
            // update display...
            switch (g_operationMode) {
                case modeTemp1:
                    displayValue(adc_to_temperature(0));
                    break;
                case modeTemp2:
                    displayValue(adc_to_temperature(1));
                    break;
                case modeTempAlarm1:
                    displayValue(TCD0.CNT);
                    break;
                case modeTempAlarm2:
                    displayValue(TCD0.CNT);
                    break;
                case modeTimer:
                    displayValue(TCD0.CNT);
                    break;
                default:
                    break;
            }
            // Wahlschalter abfragen
            OperationMode newMode = readOperationMode();
            current_mode[mode_idx] = newMode;
            mode_idx += 1;
            if (mode_idx >= MODE_BUF_COUNT) mode_idx = 0;

            // Umschalten der Betriebsart erkennen
            if (newMode != g_operationMode) {
                for (i=0, j=1; j && i<MODE_BUF_COUNT; i++) {
                    j = (current_mode[i] == newMode);
                }
                if (j) {
                    switch (g_operationMode) {
                            // je nach alter Betriebsart müssen am Drehregler eingestellte Werte übernommen werden
                        case modeTempAlarm1:
                            g_tempAlarmMax[0] = TCD0.CNT >> 2; // FIXME: ist >>2 bzw. weiter unten <<2 sinnvoll?
                            g_tempAlarm[0] = TempAlarm_Over;
                            break;
                            // FIXME: acknowledge sound
                        case modeTemp2:
                            g_tempAlarmMax[1] = TCD0.CNT >> 2;
                            g_tempAlarm[1] = TempAlarm_Over;
                            break;
                        case modeTimer:
                            g_timerAlarmSecs = 0;
                            uint16_t cnt = TCD0.CNT >> 2;
                            g_timerAlarm = (cnt>0) ? cnt : -1; // Wert nur übernehmen (= Wecker aktivieren), wenn Wert > 0 eingestellt wurde
                            break;
                        default:
                            break;
                    }
                    g_operationMode = newMode;
                    // je nach neuer Betriebsart muss der Zähler des Einstellreglers neu gesetzt werden
                    switch (g_operationMode) {
                        case modeTempAlarm1:
                            TCD0.CNT = g_tempAlarm[0]<<2;
                            TCD0.PER = 300<<2; // max. 300°C
                            break;
                        case modeTempAlarm2:
                            TCD0.CNT = g_tempAlarm[1]<<2;
                            TCD0.PER = 300<<2;
                            break;
                        case modeTimer:
                            TCD0.CNT = (g_timerAlarm>0) ? (g_timerAlarm<<2) : 0;
                            TCD0.PER = 480<<2;
                            break;
                        default:
                            break;
                    }
                }
            }
            
        }
        if (g_flags & sendMessageFlag) {
            cli();
            g_flags &= ~sendMessageFlag;
            sei();

            for (j=0; j<ADC_TEMP_CHANNELS; j++) {
                usart_puts_P(&USART_data, "Temp. #");
                usart_putc(&USART_data, '1'+j);
                usart_putc(&USART_data, ' ');
                usart_putDec(&USART_data, adc_to_temperature(j));
                reportTemperatureAlarm(j);
            }
            usart_puts_P(&USART_data, "Wecksignal: ");
            if (g_timerAlarm < 0) {
                usart_puts_P(&USART_data, "aus");
            } else {
                usart_putDec(&USART_data, g_timerAlarm);
                usart_putc(&USART_data, ':');
                usart_putDec(&USART_data, g_timerAlarmSecs);
            }
            usart_putc(&USART_data, '\n');

        }
        if (g_flags & alarmTimerFlag) {
            cli();
            g_flags &= ~alarmTimerFlag;
            g_flags |= alarmActiveFlag;
            sei();
            g_timerAlarm = -1;
            usart_puts_P(&USART_data, "\nHonk! Zeit ist um\n"); // FIXME: echter Alarm
        }
        if (g_flags & alarmTempFlag) {
            cli();
            g_flags &= ~alarmTempFlag;
            g_flags |= alarmActiveFlag;
            sei();
            usart_puts_P(&USART_data, "\nHonk! Kritische Temperatur!\n"); // FIXME: echter Alarm
        }
        if (g_flags & alarmActiveFlag) {
            usart_puts_P(&USART_data, "\nAlarm!\n");
        }
        
        // Flag, dass ein Kommando über die serielle Schnittstelle kam
        if (g_commandReceived) {
            cli();
            g_commandReceived -= 1;
            sei();
            handleCommand(getCommand());
        }
        cli();
        if (!g_commandReceived && !g_flags) {
            sleep_enable();
            sei();
            sleep_cpu();
            sleep_disable();
        }
        sei();
    }
}

//####################################################### IRQ RTC
// 1x pro Sekunde:
ISR(RTC_OVF_vect) {
    static uint8_t cnt = 0;
    
    if (cnt >= 4) { // FIXME: fürs Debugging alle 4 Sekunden Status senden, später auf 10 Sekunden hochstellen
        g_flags |= sendMessageFlag;
        cnt = 0;
    } else {
        cnt++;
    }

    // Timer(-alarm) behandeln
    if (g_timerAlarm >= 0) {
        if (g_timerAlarmSecs) {
            g_timerAlarmSecs -= 1;
        } else {
            if (g_timerAlarm == 0) {
                g_flags |= alarmTimerFlag;
            } else {
                g_timerAlarm -=1;
                g_timerAlarmSecs = 60;
            }
        }
    }
}

//####################################################### IRQ T/C C:0
// 4x pro Sekunde:
ISR(TCC0_OVF_vect) {
    static uint8_t cnt=0;

    g_flags |= measureTempFlag;
    
    // handle specific display modes
    if (g_displayMode != displayFixedValue) {
        switch (g_displayMode) {
            case displayUnderrange:
                DACB.CH1DATA = (cnt >= 2) ? 500 : 0;
                break;
            case displayOverrange:
                DACB.CH1DATA = (cnt >= 2) ? 3595 : 4095;
                break;
            case displayAttention:
                DACB.CH1DATA = (cnt >= 2) ? 1048 : 3048;
                break;
                
            default:
                break;
        }
    }
    cnt = (cnt+1) & 3;
}