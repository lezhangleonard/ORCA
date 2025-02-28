# ORCA: Offload Rethinking by Cloud Assistance for Efficient Environmental Sound Recognition on LPWANs

A usable LoRa driver for intermittent communication on MSP430FR5994 for ultra-low-power edge-cloud collaborative learning. 

This paper is accepted by [SenSys'25] [Offload Rethinking by Cloud Assistance for Efficient Environmental Sound Recognition on LPWANs](https://arxiv.org/abs/2502.15285)


## File Structure

```
.
├── driverlib/MSP430FR5xx_6xx         // MSP430 Drivers
├── targetConfigs                     // MSP430 Configs
├── README.md
├── lora.h                            // RFM95 macros from RadioHead
├── fsk.h                             // RFM95 macros from RadioHead
├── reg.h                             // RFM95 registers macros from RadioHead
├── main.c                            // Entry point, RFM95 function calls
```

## Prerequisites

MSP430FR5994 Launchpad, RFM95 (SX1276) LoRa breakout, and jumper wires

## Wiring

```
                   MSP430FR5994
                 -----------------
            /|\ |             P1.0|-> Comms LED
             |  |                 |
             ---|RST          P1.4|-> RFM95 Reset (GPIO)
                |             P1.5|-> RFM95 Interrupt (IX)
                |             P5.0|-> Data Out (UCB1SIMO)
                |                 |
       Button ->|P5.5         P5.1|<- Data In (UCB1SOMI)
   Button LED <-|P1.1             |
                |             P5.2|-> Serial Clock Out (UCB1CLK)
                |                 |
                |             P5.3|-> RFM95 Chip Select (GPIO)
```

## Get Started

### Intialize RFM95

```
rf_init_lora();
```

### Configure Tx and Rx

```
lora_set_txconfig();
lora_set_rxconfig();
```

### Tx 

```
lora_send(SEND_BUFFER, SEND_SIZE);
__bis_SR_register(LPM3_bits | GIE);       // Wait for TxDone interrupt
```

### Rx

```
lora_recv();                              // Set up Rx
start_timer(RX_WINDOW_SIZE);              // Set up a timer for Rx window
```

## Acknowledgements
This repo is developed and adapted from [MSP430_LoRaWAN](https://github.com/EccoB/MSP430_LoRaWAN) and [RadioHead](https://www.airspayce.com/mikem/arduino/RadioHead/). 

The machine learning library is referred to [Capuchin](https://github.com/lezhangleonard/Capuchin), a highly customizable ultra-low-power machine learning model generator for MSP430. 

## Cite me

@misc{zhang2025offloadrethinkingcloudassistance,
      title={Offload Rethinking by Cloud Assistance for Efficient Environmental Sound Recognition on LPWANs}, 
      author={Le Zhang and Quanling Zhao and Run Wang and Shirley Bian and Onat Gungor and Flavio Ponzina and Tajana Rosing},
      year={2025},
      eprint={2502.15285},
      archivePrefix={arXiv},
      primaryClass={cs.SD},
      url={https://arxiv.org/abs/2502.15285}, 
}

