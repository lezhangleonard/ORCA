/*
 * This project is forked from msp430fr599x_eusci_spi_standard_master.c by Nima Eskandari and Ryan Meredith in TI MSP430 official codes
 * Source: https://dev.ti.com/tirex/explore/node?node=ACHFXHTiNG.iukEIl1BQ2w__IOGqZri__LATEST
 *
 * Interface MSP430FR5994 with Adafruit RFM95 by SPI
 * Datasheet: https://cdn.sparkfun.com/assets/learn_tutorials/8/0/4/RFM95_96_97_98W.pdf
 *
 * SPI_Master_Communicate is flexible and standard SPI protocol
 * Could be used to interface with different hardware
 *
 */

// MSP430FR5994 SPI Pins Configure
//
//                   MSP430FR5994
//                 -----------------
//            /|\ |             P1.0|-> Comms LED
//             |  |                 |
//             ---|RST          P1.4|-> Slave Reset (GPIO)
//                |             P1.5|-> Slave Interrupt (IX)
//                |             P5.0|-> Data Out (UCB1SIMO)
//                |                 |
//       Button ->|P5.5         P5.1|<- Data In (UCB1SOMI)
//   Button LED <-|P1.1             |
//                |             P5.2|-> Serial Clock Out (UCB1CLK)
//                |                 |
//                |             P5.3|-> Slave Chip Select (GPIO)
//

#include <msp430.h>
#include <stdint.h>
#include <gpio.h>
#include <math.h>
#include "lora.h"
#include "reg.h"
#include "fsk.h"

#define SPI_PORT GPIO_PORT_P5
#define SPI_TX GPIO_PIN0
#define SPI_RX GPIO_PIN1
#define SPI_SCK GPIO_PIN2
#define SPI_CS GPIO_PIN3

#define SLAVE_PORT GPIO_PORT_P1
#define SLAVE_RST GPIO_PIN4
#define SLAVE_IX GPIO_PIN5

#define VIN_PORT GPIO_PORT_P3
#define VIN_PIN GPIO_PIN7

#define MAX_BUFFER_SIZE 256

#define FREQ_STEP 61.03515625
#define RF_FREQUENCY 915000000
#define RF_MID_BAND_THRESH 525000000
#define TX_OUTPUT_POWER 17
#define LORA_BANDWIDTH 0        // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1       // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 3   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define PAYLOAD_SIZE 10
#define MESSAGE_SIZE PAYLOAD_SIZE + 9     // 9 Bytes MHDR+FHDR+FPort
#define PHY_PAYLOAD_SIZE MESSAGE_SIZE + 4 // 4 Bytes Message Integrity Check

#define TIMEOUT 0x64

#define RF_IDLE 0
#define RF_RX_RUNNING 1
#define RF_TX_RUNNING 2
#define RF_CAD 3

#define RSSI_OFFSET_LF -164
#define RSSI_OFFSET_HF -157

uint32_t freq = 0;
uint8_t state = 0;
uint8_t paConfig = 0;
uint8_t paDac = 0;
int8_t pwr = 0;
uint8_t lowDatarateOptimize = 0;
int8_t snr = 0;
int16_t rssi = 0;

uint8_t ReceiveBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t TransmitBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t recv_buffer[MAX_BUFFER_SIZE] = {0};

uint16_t RXByteCtr = 0;
uint16_t TXByteCtr = 0;
uint16_t RXBytePtr = 0;
uint16_t TXBytePtr = 0;

void SendUCB1Data(uint8_t val)
{
    while (!(UCB1IFG & UCTXIFG))
        ; // USCI_B1 TX buffer ready?
    UCB1TXBUF = val;
}

void spi_comm(uint16_t rx_size, uint16_t tx_size)
{
    GPIO_setOutputLowOnPin(SPI_PORT, SPI_CS);

    RXBytePtr = 0;
    TXBytePtr = 0;
    RXByteCtr = rx_size;
    TXByteCtr = tx_size;

    TXByteCtr--;
    SendUCB1Data(TransmitBuffer[TXBytePtr++]);

    __bis_SR_register(LPM3_bits | GIE);
    GPIO_setOutputHighOnPin(SPI_PORT, SPI_CS);
}

void initSPI()
{
    // Clock Polarity: The inactive state is high
    // MSB First, 8-bit, Master, 3-pin mode, Synchronous
    UCB1CTLW0 = UCSWRST;                                          // **Put state machine in reset**
    UCB1CTLW0 |= UCCKPL | UCMSB | UCSYNC | UCMST | UCSSEL__SMCLK; // 3-pin, 8-bit SPI Slave
    UCB1BRW = 0x20;                                               // Set up SPI CLK. Bit clock prescaler setting. divide the clock of UCSSEL__****.
                                                                  // e.g. SMCLK = 16MHz, UCB1BRW = 0x20 (32), SPI = 16MHz / 32 = 500KHz
    UCB1CTLW0 &= ~UCSWRST;                                        // **Initialize USCI state machine**
    UCB1IE |= UCRXIE;                                             // Enable USCI0 RX interrupt
}

void initGPIO()
{
    // Configure SPI
    P5SEL0 |= BIT0 | BIT1 | BIT2;

    GPIO_setAsInputPin(SLAVE_PORT, SLAVE_IX);

    GPIO_setAsOutputPin(SLAVE_PORT, SLAVE_RST);
    GPIO_setOutputLowOnPin(SLAVE_PORT, SLAVE_RST);

    GPIO_setAsOutputPin(SPI_PORT, SPI_CS);
    GPIO_setOutputHighOnPin(SPI_PORT, SPI_CS);

    GPIO_setAsOutputPin(VIN_PORT, VIN_PIN);
    GPIO_setOutputHighOnPin(VIN_PORT, VIN_PIN);

    GPIO_setAsInputPinWithPullDownResistor(SLAVE_PORT, SLAVE_IX); // IRQ pin
    GPIO_selectInterruptEdge(SLAVE_PORT, SLAVE_IX, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_clearInterrupt(SLAVE_PORT, SLAVE_IX);
    GPIO_enableInterrupt(SLAVE_PORT, SLAVE_IX);

    PM5CTL0 &= ~LOCKLPM5;
}

void initClockTo16MHz()
{
    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    // Clock System Setup
    CSCTL0_H = CSKEY_H; // Unlock CS registers
    CSCTL1 = DCOFSEL_0; // Set DCO to 1MHz

    // Set SMCLK = MCLK = DCO, ACLK = VLOCLK
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;

    // Per Device Errata set divider to 4 before changing frequency to
    // prevent out of spec operation from overshoot transient
    CSCTL3 = DIVA__4 | DIVS__4 | DIVM__4; // Set all corresponding clk sources to divide by 4 for errata
    CSCTL1 = DCOFSEL_4 | DCORSEL;         // Set DCO to 16MHz

    // Delay by ~10us to let DCO settle. 60 cycles = 20 cycles buffer + (10us / (1/4MHz))
    __delay_cycles(60);
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1; // Set all dividers to 1 for 16MHz operation
    CSCTL0_H = 0;                         // Lock CS registers
}

void start_timer(uint16_t cycles)
{
    TA1CCTL0 = CCIE; // TACCR0 interrupt enabled
    TA1CCR0 = cycles;
    TA1CTL = TASSEL__ACLK | MC__CONTINUOUS | TACLR; // ACLK, continuous mode
    __bis_SR_register(LPM3_bits | GIE);             // Enter LPM3 w/ interrupt
}

void lora_reset()
{
    __delay_cycles(60000);
    GPIO_setAsOutputPin(SLAVE_PORT, SLAVE_RST); // Reset
    GPIO_setOutputLowOnPin(SLAVE_PORT, SLAVE_RST);

    __delay_cycles(120000);

    GPIO_setOutputHighOnPin(SLAVE_PORT, SLAVE_RST);
    GPIO_setAsInputPin(SLAVE_PORT, SLAVE_RST); // Reset

    __delay_cycles(240000);
}

void lora_write(uint8_t addr, uint8_t data)
{
    uint8_t slave_addr = addr | 0x80;
    TransmitBuffer[0] = slave_addr;
    TransmitBuffer[1] = data;
    spi_comm(0, 2);
}

void lora_write_buffer(uint8_t addr, uint8_t *data, uint8_t len)
{
    uint8_t slave_addr = addr | 0x80;
    TransmitBuffer[0] = slave_addr;
    uint16_t i = 1;
    while (i - 1 < len)
    {
        TransmitBuffer[i] = data[i - 1];
        i++;
    }
    spi_comm(0, len + 1);
}

uint8_t lora_read(uint8_t addr)
{
    uint8_t slave_addr = addr & 0x7f;
    TransmitBuffer[0] = slave_addr;
    spi_comm(2, 1);
    return ReceiveBuffer[1];
}

uint8_t *lora_read_buffer(uint8_t addr, uint8_t *data, uint8_t len)
{
    if (len == 0)
    {
        return data;
    }
    uint8_t slave_addr = addr & 0x7f;
    TransmitBuffer[0] = slave_addr;
    TransmitBuffer[1] = 0x00;
    uint16_t i = 0;
    spi_comm(2, len);
    while (i < len)
    {
        data[i] = ReceiveBuffer[i];
        i++;
    }
    return data;
}

void lora_set_opmode(uint8_t opmode)
{
    // REG_OPMODE   7: Long range  0=FSK/OOK, 1=LoRa  6-5: 00=FSK, 01=OOK   4: Low Freq Mode    3-0: Opmode 000=Sleep 001=Stdby 010=FSTx 011=Tx 100=FSRx 101=Rx
    uint8_t opmode_reg;
    lora_write(REG_OPMODE, (lora_read(REG_OPMODE) & RF_OPMODE_MASK) | opmode);
    opmode_reg = lora_read(REG_OPMODE);
}

void lora_set_channel()
{
    // FREQ_STEP ~ 61: RFM95 Specific
    // FRF should then have 0xE4C026 for 915Mhz
    // FRF should then have 0xD90000 for 868Mhz
    freq = 0xE4C026;
    lora_write(REG_FRFMSB, (uint8_t)((freq >> 16) & 0xFF));
    lora_write(REG_FRFMID, (uint8_t)((freq >> 8) & 0xFF));
    lora_write(REG_FRFLSB, (uint8_t)((freq) & 0xFF));

    // HF mode in OPMODE
    lora_write(REG_OPMODE, (lora_read(REG_OPMODE) & RFLR_OPMODE_FREQMODE_ACCESS_MASK) | RFLR_OPMODE_FREQMODE_ACCESS_HF);
}

void lora_set_modem()
{
    lora_set_opmode(RF_OPMODE_SLEEP);
    lora_write(REG_OPMODE, (lora_read(REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK) | RFLR_OPMODE_LONGRANGEMODE_ON);
    lora_write(REG_DIOMAPPING1, 0x00);
    lora_write(REG_DIOMAPPING2, 0x00);
}

void lora_set_sf(uint8_t sf)
{
    if (sf > 12)
        sf = 7;

    if ((sf == 11) || (sf == 12))
    {
        lowDatarateOptimize = 0x01;
    }
    else
    {
        lowDatarateOptimize = 0x00;
    }
    // REG_LR_MODEMCONFIG2  7-4: SF, 3: TX Continuous Mode, 2: Rx CRC, 1: Symbol timeout
    lora_write(REG_LR_MODEMCONFIG2, (lora_read(REG_LR_MODEMCONFIG2) & RFLR_MODEMCONFIG2_SF_MASK) | (sf << 4));
    // REG_LR_MODEMCONFIG3  3: Low Data Rate Optim
    lora_write(REG_LR_MODEMCONFIG3, (lora_read(REG_LR_MODEMCONFIG3) & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK) | (lowDatarateOptimize << 3));
}

void lora_set_power(int8_t pa_power)
{
    if (pa_power > 17)
        pa_power = 2;
    lora_set_opmode(RFLR_OPMODE_STANDBY);

    // REG_LR_PACONFIG  7: PASEL (0: 14dBm, 1: PA_Boost 20dBm), 6-4: Max Power Pmax=(10.8+0.6*MaxPower),
    // 3-0: OutputPower (PASEL=0, Pout=Pmax-(15-OutputPower); PASEL=1, Pout=17-(15-OutputPower))
    paConfig = lora_read(REG_LR_PACONFIG);
    paConfig = 0x80;

    lora_write(REG_LR_PACONFIG, paConfig | (pa_power - 2));
    lora_set_opmode(RFLR_OPMODE_SLEEP);
}

void lora_set_rxconfig(uint32_t bandwidth,
                       uint32_t datarate, uint8_t coderate, uint16_t preambleLen,
                       uint16_t symbTimeout, bool fixLen,
                       uint8_t payloadLen, bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                       bool iqInverted, bool rxContinuous)
{

    if (bandwidth > 2)
    {
        while (1)
            ;
    }
    bandwidth += 7;

    if (datarate > 12)
        datarate = 12;
    else if (datarate < 6)
        datarate = 6;

    if (((bandwidth == 7) && ((datarate == 11) || (datarate == 12))) || ((bandwidth == 8) && (datarate == 12)))
    {
        lowDatarateOptimize = 0x01;
    }
    else
    {
        lowDatarateOptimize = 0x00;
    }

    lora_write(REG_LR_MODEMCONFIG1,
               (lora_read(REG_LR_MODEMCONFIG1) & RFLR_MODEMCONFIG1_BW_MASK & RFLR_MODEMCONFIG1_CODINGRATE_MASK & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK) |
                   (bandwidth << 4) | (coderate << 1) |
                   fixLen);

    lora_write(REG_LR_MODEMCONFIG2,
               (lora_read(REG_LR_MODEMCONFIG2) & RFLR_MODEMCONFIG2_SF_MASK & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK) |
                   (datarate << 4) | (crcOn << 2) |
                   ((symbTimeout >> 8) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK));

    lora_write(REG_LR_MODEMCONFIG3,
               (lora_read(REG_LR_MODEMCONFIG3) & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK) |
                   (lowDatarateOptimize << 3));

    lora_write(REG_LR_SYMBTIMEOUTLSB, (uint8_t)(symbTimeout & 0xFF));
    lora_write(REG_LR_PREAMBLEMSB, (uint8_t)((preambleLen >> 8) & 0xFF));
    lora_write(REG_LR_PREAMBLELSB, (uint8_t)((preambleLen) & 0xFF));

    if (fixLen == 1)
        lora_write(REG_LR_PAYLOADLENGTH, payloadLen);

    if (bandwidth == 9)
    {
        // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
        lora_write(REG_LR_TEST36, 0x02);
        lora_write(REG_LR_TEST3A, 0x64);
    }
    else if (bandwidth == 9)
    {
        // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
        lora_write(REG_LR_TEST36, 0x02);
        lora_write(REG_LR_TEST3A, 0x7F);
    }
    else
    {
        // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
        lora_write(REG_LR_TEST36, 0x03);
    }

    if (datarate == 6)
    {
        lora_write(REG_LR_DETECTOPTIMIZE,
                   (lora_read(REG_LR_DETECTOPTIMIZE) & RFLR_DETECTIONOPTIMIZE_MASK) |
                       RFLR_DETECTIONOPTIMIZE_SF6);
        lora_write(REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF6);
    }
    else
    {
        lora_write(REG_LR_DETECTOPTIMIZE,
                   (lora_read(REG_LR_DETECTOPTIMIZE) & RFLR_DETECTIONOPTIMIZE_MASK) |
                       RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
        lora_write(REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12);
    }
}

uint32_t lora_timeonair(uint32_t datarate, uint8_t pktLen)
{
    uint32_t airTime = 0;

    double bw = 0.0;
    // REMARK: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
    switch (LORA_BANDWIDTH)
    {
    case 0: // 125 kHz
        bw = 125e3;
        break;
    case 1: // 250 kHz
        bw = 250e3;
        break;
    case 2: // 500 kHz
        bw = 500e3;
        break;
    }

    // Symbol rate : time for one symbol (secs)
    double rs = bw / (1 << datarate);
    double ts = 1 / rs;
    // time of preamble
    double tPreamble = (LORA_PREAMBLE_LENGTH + 4.25) * ts;
    // Symbol length of payload and time
    double tmp = ceil((8 * pktLen - 4 * datarate +
                       28 + 16 * 1 -
                       (LORA_FIX_LENGTH_PAYLOAD_ON ? 20 : 0)) /
                      (double)(4 * datarate -
                               ((lowDatarateOptimize > 0) ? 2 : 0))) *
                 (LORA_CODINGRATE + 4);
    double nPayload = 8 + ((tmp > 0) ? tmp : 0);
    double tPayload = nPayload * ts;
    // Time on air
    double tOnAir = tPreamble + tPayload;
    // return us secs
    airTime = floor(tOnAir * 1e3 + 0.999);
    return airTime;
}

void lora_set_txconfig(int8_t power, uint32_t fdev,
                       uint32_t bandwidth, uint32_t datarate,
                       uint8_t coderate, uint16_t preambleLen,
                       bool fixLen, bool crcOn, bool freqHopOn,
                       uint8_t hopPeriod, bool iqInverted, uint32_t timeout)
{

    paConfig = lora_read(REG_PACONFIG);
    uint8_t paDac = lora_read(REG_PADAC);

    lora_set_power(power);

    if (power < -1)
        power = -1;
    if (power > 14)
        power = 14;

    // paDac    2-0: RF_PADAC_20DBM_OFF = Default, RF_PADAC_20DBM_ON = PA_BOOST enable
    lora_write(REG_PADAC, 0x84);

    if (bandwidth > 2)
    {
        // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
        while (1)
            ;
    }
    bandwidth += 7;

    if (datarate > 12)
        datarate = 12;
    else if (datarate < 6)
        datarate = 6;

    if (((bandwidth == 7) && ((datarate == 11) || (datarate == 12))) || ((bandwidth == 8) && (datarate == 12)))
    {
        lowDatarateOptimize = 0x01;
    }
    else
    {
        lowDatarateOptimize = 0x00;
    }

    // MODEMCONFIG1 7-4: Bw, 3-1: Coding Rate, 0: implicit header
    lora_write(REG_LR_MODEMCONFIG1,
               (lora_read(REG_LR_MODEMCONFIG1) & RFLR_MODEMCONFIG1_BW_MASK & RFLR_MODEMCONFIG1_CODINGRATE_MASK & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK) |
                   (bandwidth << 4) | (coderate << 1) |
                   fixLen); // fixlen is used to determine the use of Implicit/Explicit header

    uint8_t continous_rx = 0;
    lora_write(REG_LR_MODEMCONFIG2,
               (lora_read(REG_LR_MODEMCONFIG2) & RFLR_MODEMCONFIG2_SF_MASK & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK) |
                   (datarate << 4) | (crcOn << 2) | (continous_rx << 3));

    lora_write(REG_LR_MODEMCONFIG3,
               (lora_read(REG_LR_MODEMCONFIG3) & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK) |
                   (lowDatarateOptimize << 3));

    // Preamble Length
    lora_write(REG_LR_PREAMBLEMSB, (preambleLen >> 8) & 0x00FF);
    lora_write(REG_LR_PREAMBLELSB, (preambleLen) & 0xFF);
}

void rf_init_lora()
{
    lora_reset();
    lora_set_opmode(RFLR_OPMODE_SLEEP);
    lora_set_modem();
    lora_set_channel();
    lora_set_txconfig(TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
    lora_set_sf(LORA_SPREADING_FACTOR);

    // Full buffer used for Tx
    lora_write(REG_LR_FIFOTXBASEADDR, 0);
    lora_write(REG_LR_FIFOADDRPTR, 0);

    // Initializes the payload size
    lora_write(REG_LR_PAYLOADLENGTH, PAYLOAD_SIZE); // Initializes the payload size
                                                    //    #endif
                                                    //

    lora_set_opmode(RFLR_OPMODE_STANDBY);
}

void lora_send(uint8_t *buffer, uint8_t size)
{

    lora_write(REG_LR_PAYLOADLENGTH, size);

    // Set full buffer used for Tx
    lora_write(REG_LR_FIFOTXBASEADDR, 0);
    lora_write(REG_LR_FIFOADDRPTR, 0); // Has to be sent every time: Sentbufferaddress, return to 0

    lora_write(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        // RFLR_IRQFLAGS_TXDONE |
                                        RFLR_IRQFLAGS_CADDONE |
                                        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                        RFLR_IRQFLAGS_CADDETECTED);

    lora_write(REG_LR_DIOMAPPING1, (lora_read(REG_LR_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_01);

    // FIFO operations can not take place in Sleep mode
    lora_set_opmode(RFLR_OPMODE_STANDBY);

    if ((lora_read(REG_OPMODE) & ~RF_OPMODE_MASK) == RF_OPMODE_SLEEP)
    {
        lora_set_opmode(RF_OPMODE_STANDBY);
        state = RF_IDLE;
    }

    state = RF_TX_RUNNING;

    // Write payload buffer, addr=0x00
    lora_write_buffer(0x00, buffer, size);

    // Start to transmit
    lora_set_opmode(RFLR_OPMODE_TRANSMITTER);
}

void lora_recv()
{
    lora_write(REG_LR_DETECTOPTIMIZE, lora_read(REG_LR_DETECTOPTIMIZE) | 0x80);
    lora_write(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                        // RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        RFLR_IRQFLAGS_TXDONE |
                                        RFLR_IRQFLAGS_CADDONE |
                                        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                        RFLR_IRQFLAGS_CADDETECTED);

    // DIO0=RxDone
    lora_write(REG_DIOMAPPING1, (lora_read(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_00);

    state = RF_RX_RUNNING;

    lora_write(REG_LR_FIFORXBASEADDR, 0);
    lora_write(REG_LR_FIFOADDRPTR, 0);

    lora_set_opmode(RFLR_OPMODE_RECEIVER_SINGLE);
}

void lora_timeout()
{
    if (state == RF_RX_RUNNING)
    {
        P1IES ^= BIT1;
        lora_write(REG_LR_IRQFLAGS, 0);
        lora_read(REG_LR_IRQFLAGS);
        state = RF_IDLE;
        lora_set_opmode(RFLR_OPMODE_SLEEP);
    }
}

void lora_on_dio0irq()
{
    volatile uint8_t irqFlags = 0;

    lora_read(0x01);
    lora_read(REG_LR_IRQFLAGS);

    switch (state)
    {
    case RF_RX_RUNNING:
        irqFlags = lora_read(REG_LR_IRQFLAGS);

        if ((irqFlags & RFLR_IRQFLAGS_RXTIMEOUT_MASK) == RFLR_IRQFLAGS_RXTIMEOUT)
        {
            lora_write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXTIMEOUT);
            state = RF_IDLE;
            lora_set_opmode(RFLR_OPMODE_SLEEP);
        }

        else
        {
            // Clear Slave Irq
            lora_write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);

            lora_read(REG_LR_IRQFLAGS);

            irqFlags = lora_read(REG_LR_IRQFLAGS);
            if ((irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK) == RFLR_IRQFLAGS_PAYLOADCRCERROR)
            {
                // Make sure clear Slave Irq
                lora_write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR);
                state = RF_IDLE;
            }

            state = RF_IDLE;
            lora_set_opmode(RFLR_OPMODE_STANDBY);

            uint16_t size = lora_read(REG_LR_RXNBBYTES);
            lora_read_buffer(0x00, recv_buffer, size);

            state = RF_IDLE;
            lora_set_opmode(RFLR_OPMODE_SLEEP);

            snr = lora_read(REG_LR_PKTSNRVALUE);
            if (snr & 0x80)
            {
                snr = ((~snr + 1) & 0xFF) >> 2;
                snr = -snr;
            }
            else
            {
                snr = (snr & 0xFF) >> 2;
            }

            rssi = lora_read(REG_LR_PKTRSSIVALUE);
            if (snr < 0)
            {
                rssi = RSSI_OFFSET_HF + rssi + (rssi >> 4) + snr;
            }
            else
            {
                rssi = RSSI_OFFSET_HF + rssi + (rssi >> 4);
            }
        }
        lora_timeout();
        break;

    case RF_TX_RUNNING:
        // Tx finished, clear slave IRQ, and set to sleep
        state = RF_IDLE;
        lora_write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE);
        lora_read(REG_LR_IRQFLAGS);
        lora_set_opmode(RFLR_OPMODE_SLEEP);
        break;

    default:
        state = RF_IDLE;
        break;
    }
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

    initClockTo16MHz();
    initGPIO();
    initSPI();

    // RFM95 initialization
    rf_init_lora();

    // Tx payload
    uint8_t send_buffer[10] = {0, 0, 0, 0, 1, 2, 3, 4, 5, 6};

    // Config Tx and Rx params
    lora_set_txconfig(TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
    lora_set_rxconfig(LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH,
                      TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      PAYLOAD_SIZE, true, 0, 0, 0, 0);

    while (1)
    {
        // Tx
        lora_send(send_buffer, 10);

        // Wait for TxDone interrupt
        __bis_SR_register(LPM3_bits | GIE);

        __no_operation();

        // Sleep
        __delay_cycles(10000000);

        // Setup Rx
        lora_recv();

        // Rx waiting window
        start_timer(20000);

        __no_operation();
    }
}

/* TxDone interrupt */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = PORT1_VECTOR
__interrupt void port1_isr_handler(void)
#elif defined(__GNUC__)
void __attribute__((interrupt(PORT1_VECTOR))) port1_isr_handler(void)
#else
#error Compiler not supported!
#endif
{
    switch (__even_in_range(P1IV, P1IV__P1IFG7))
    {
    case P1IV__NONE:
        break; // Vector  0:  No interrupt
    case P1IV__P1IFG0:
        break; // Vector  2:  P1.0 interrupt flag
    case P1IV__P1IFG1:
        break;
    case P1IV__P1IFG2:
        break; // Vector  6:  P1.2 interrupt flag
    case P1IV__P1IFG3:
        break; // Vector  8:  P1.3 interrupt flag
    case P1IV__P1IFG4:
        break; // Vector  10:  P1.4 interrupt flag
    case P1IV__P1IFG5:
        P1IES ^= BIT1; // Toggle interrupt edge
        lora_on_dio0irq();
        __bic_SR_register_on_exit(LPM3_bits); // Exit LPM4
        break;
    case P1IV__P1IFG6:
        break; // Vector  14:  P1.6 interrupt flag
    case P1IV__P1IFG7:
        break; // Vector  16:  P1.7 interrupt flag
    default:
        break;
    }
}

/* SPI to RFM95 */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void)
#elif defined(__GNUC__)
void __attribute__((interrupt(USCI_B1_VECTOR))) USCI_B1_ISR(void)
#else
#error Compiler not supported!
#endif
{
    uint8_t ucb1_rx_val = 0;
    switch (__even_in_range(UCB1IV, USCI_SPI_UCTXIFG))
    {
    case USCI_NONE:
        break;
    case USCI_SPI_UCRXIFG:
        ucb1_rx_val = UCB1RXBUF;
        UCB1IFG &= ~UCRXIFG;
        if (TXByteCtr)
        {
            TXByteCtr--;
            SendUCB1Data(TransmitBuffer[TXBytePtr++]);
        }
        else if (RXByteCtr)
        {
            SendUCB1Data(0x00);
            RXByteCtr--;
            ReceiveBuffer[RXBytePtr++] = ucb1_rx_val;
        }
        else
        {
            __bic_SR_register_on_exit(LPM3_bits);
        }
        break;
    case USCI_SPI_UCTXIFG:
        break;
    default:
        break;
    }
}

/* Timer for Rx windows */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__((interrupt(TIMER1_A0_VECTOR))) Timer1_A0_ISR(void)
#else
#error Compiler not supported!
#endif
{
    TA1CTL &= ~TASSEL_2; // Stop timer
    TA1CTL |= TACLR;     // Clear Timer
    lora_timeout();
    __bic_SR_register_on_exit(LPM3_bits);
}
