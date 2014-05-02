#include "sx127x_lora.h"

/* SX127x driver
 * Copyright (c) 2013 Semtech
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

SX127x_lora::SX127x_lora(SX127x& r) : m_xcvr(r)
{
    RegModemConfig.octet = m_xcvr.read_reg(REG_LR_MODEMCONFIG);
    RegModemConfig2.octet = m_xcvr.read_reg(REG_LR_MODEMCONFIG2);
    RegTest31.octet = m_xcvr.read_reg(REG_LR_TEST31);
    
    // CRC for TX is disabled by default
    setRxPayloadCrcOn(true);
}

SX127x_lora::~SX127x_lora()
{
}
    
void SX127x_lora::write_fifo(uint8_t len)
{
    int i;
    
    m_xcvr.m_cs = 0;
    m_xcvr.m_spi.write(REG_FIFO | 0x80); // bit7 is high for writing to radio
    
    for (i = 0; i < len; i++) {
        m_xcvr.m_spi.write(m_xcvr.tx_buf[i]);
    }
    m_xcvr.m_cs = 1;
}

void SX127x_lora::read_fifo(uint8_t len)
{
    int i;
     
    m_xcvr.m_cs = 0;
    m_xcvr.m_spi.write(REG_FIFO); // bit7 is low for reading from radio
    for (i = 0; i < len; i++) {
        m_xcvr.rx_buf[i] = m_xcvr.m_spi.write(0);
    }
    m_xcvr.m_cs = 1;
}

void SX127x_lora::enable()
{
    m_xcvr.set_opmode(RF_OPMODE_SLEEP);
    
    m_xcvr.RegOpMode.bits.LongRangeMode = 1;
    m_xcvr.write_reg(REG_OPMODE, m_xcvr.RegOpMode.octet);
    
    /*RegOpMode.octet = read_reg(REG_OPMODE);
    printf("setloraon:%02x\r\n", RegOpMode.octet);*/
    

    /*                                // RxDone               RxTimeout                   FhssChangeChannel           CadDone
    SX1272LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                    // CadDetected          ModeReady
    SX1272LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
    SX1272WriteBuffer( REG_LR_DIOMAPPING1, &SX1272LR->RegDioMapping1, 2 );*/
    m_xcvr.RegDioMapping1.bits.Dio0Mapping = 0;    // DIO0 to RxDone
    m_xcvr.RegDioMapping1.bits.Dio1Mapping = 0;
    m_xcvr.write_reg(REG_DIOMAPPING1, m_xcvr.RegDioMapping1.octet);
    
    // todo: read LoRa regsiters
    //SX1272ReadBuffer( REG_LR_OPMODE, SX1272Regs + 1, 0x70 - 1 );
        
    m_xcvr.set_opmode(RF_OPMODE_STANDBY);            
}

uint8_t SX127x_lora::getCodingRate(bool from_rx)
{
    if (from_rx) {
        // expected RegModemStatus was read on RxDone interrupt
        return RegModemStatus.bits.RxCodingRate;    
    } else {    // transmitted coding rate...
        if (m_xcvr.type == SX1276)
            return RegModemConfig.sx1276bits.CodingRate;
        else if (m_xcvr.type == SX1272)
            return RegModemConfig.sx1272bits.CodingRate;
        else
            return 0;
    }
}



void SX127x_lora::setCodingRate(uint8_t cr)
{
    if (!m_xcvr.RegOpMode.bits.LongRangeMode)
        return;
        
    if (m_xcvr.type == SX1276)
        RegModemConfig.sx1276bits.CodingRate = cr;
    else if (m_xcvr.type == SX1272)
        RegModemConfig.sx1272bits.CodingRate = cr;
    else
        return;
        
    m_xcvr.write_reg(REG_LR_MODEMCONFIG, RegModemConfig.octet);
}



bool SX127x_lora::getHeaderMode(void)
{
    if (m_xcvr.type == SX1276)
        return RegModemConfig.sx1276bits.ImplicitHeaderModeOn;
    else if (m_xcvr.type == SX1272)
        return RegModemConfig.sx1272bits.ImplicitHeaderModeOn;
    else
        return false;
}

void SX127x_lora::setHeaderMode(bool hm)
{
    if (m_xcvr.type == SX1276)
        RegModemConfig.sx1276bits.ImplicitHeaderModeOn = hm;
    else if (m_xcvr.type == SX1272)
        RegModemConfig.sx1272bits.ImplicitHeaderModeOn = hm;
    else
        return;
        
    m_xcvr.write_reg(REG_LR_MODEMCONFIG, RegModemConfig.octet);
}


uint8_t SX127x_lora::getBw(void)
{
    if (m_xcvr.type == SX1276)
        return RegModemConfig.sx1276bits.Bw;
    else if (m_xcvr.type == SX1272)
        return RegModemConfig.sx1272bits.Bw;
    else
        return 0;
}

void SX127x_lora::setBw(uint8_t bw)
{
    if (!m_xcvr.RegOpMode.bits.LongRangeMode)
        return;
        
    if (m_xcvr.type == SX1276)
        RegModemConfig.sx1276bits.Bw = bw;
    else if (m_xcvr.type == SX1272) {
        RegModemConfig.sx1272bits.Bw = bw;
        if (RegModemConfig2.sx1272bits.SpreadingFactor > 10)
            RegModemConfig.sx1272bits.LowDataRateOptimize = 1;
        else
            RegModemConfig.sx1272bits.LowDataRateOptimize = 0;
    } else
        return;
        
    m_xcvr.write_reg(REG_LR_MODEMCONFIG, RegModemConfig.octet);
}



uint8_t SX127x_lora::getSf(void)
{
    // spreading factor same between sx127[26]
    return RegModemConfig2.sx1276bits.SpreadingFactor;
}

void SX127x_lora::set_nb_trig_peaks(int n)
{
    RegTest31.bits.detect_trig_same_peaks_nb = n;
    m_xcvr.write_reg(REG_LR_TEST31, RegTest31.octet);
}


void SX127x_lora::setSf(uint8_t sf)
{
    if (!m_xcvr.RegOpMode.bits.LongRangeMode)
        return;
            
    // false detections vs missed detections tradeoff
    switch (sf) {
        case 6:
            set_nb_trig_peaks(3);
            break;
        case 7:
            set_nb_trig_peaks(4);
            break;
        default:
            set_nb_trig_peaks(5);
            break;
    }
    
    // write register at 0x37 with value 0xc if at SF6
    if (sf < 7)
        m_xcvr.write_reg(REG_LR_DETECTION_THRESHOLD, 0x0c);
    else
        m_xcvr.write_reg(REG_LR_DETECTION_THRESHOLD, 0x0a);
    
    RegModemConfig2.sx1276bits.SpreadingFactor = sf; // spreading factor same between sx127[26]
    m_xcvr.write_reg(REG_LR_MODEMCONFIG2, RegModemConfig2.octet);
    
    if (m_xcvr.type == SX1272) {
        if (sf > 10 && RegModemConfig.sx1272bits.Bw == 0)   // if bw=125KHz and sf11 or sf12
            RegModemConfig.sx1272bits.LowDataRateOptimize = 1;
        else
            RegModemConfig.sx1272bits.LowDataRateOptimize = 0;
        m_xcvr.write_reg(REG_LR_MODEMCONFIG, RegModemConfig.octet);
    } else if (m_xcvr.type == SX1276) {
        if (sf > 10 && RegModemConfig.sx1272bits.Bw == 0)   // if bw=125KHz and sf11 or sf12
            RegModemConfig3.sx1276bits.LowDataRateOptimize = 1;
        else
            RegModemConfig3.sx1276bits.LowDataRateOptimize = 0;
        m_xcvr.write_reg(REG_LR_MODEMCONFIG3, RegModemConfig3.octet);
    }
}


        
bool SX127x_lora::getRxPayloadCrcOn(void)
{
    if (m_xcvr.type == SX1276)
        return RegModemConfig2.sx1276bits.RxPayloadCrcOn;
    else if (m_xcvr.type == SX1272)
        return RegModemConfig.sx1272bits.RxPayloadCrcOn;
    else
        return 0;
}


void SX127x_lora::setRxPayloadCrcOn(bool on)
{
    if (m_xcvr.type == SX1276) {
        RegModemConfig2.sx1276bits.RxPayloadCrcOn = on;
        m_xcvr.write_reg(REG_LR_MODEMCONFIG2, RegModemConfig2.octet);
    } else if (m_xcvr.type == SX1272) {
        RegModemConfig.sx1272bits.RxPayloadCrcOn = on;
        m_xcvr.write_reg(REG_LR_MODEMCONFIG, RegModemConfig.octet);
    }   
}



bool SX127x_lora::getAgcAutoOn(void)
{
    if (m_xcvr.type == SX1276) {
        RegModemConfig3.octet = m_xcvr.read_reg(REG_LR_MODEMCONFIG3);
        return RegModemConfig3.sx1276bits.AgcAutoOn;
    } else if (m_xcvr.type == SX1272) {
        RegModemConfig2.octet = m_xcvr.read_reg(REG_LR_MODEMCONFIG2);
        return RegModemConfig2.sx1272bits.AgcAutoOn;
    } else
        return 0;
}

void SX127x_lora::setAgcAutoOn(bool on)
{
    if (m_xcvr.type == SX1276) {
        RegModemConfig3.sx1276bits.AgcAutoOn = on;
        m_xcvr.write_reg(REG_LR_MODEMCONFIG3, RegModemConfig3.octet);
    } else if (m_xcvr.type == SX1272) {
        RegModemConfig2.sx1272bits.AgcAutoOn = on;
        m_xcvr.write_reg(REG_LR_MODEMCONFIG2, RegModemConfig2.octet);
    }
    
}



void SX127x_lora::start_tx(uint8_t len)
{
    if (m_xcvr.type == SX1276) {
        // PA_BOOST on LF, RFO on HF
        if (m_xcvr.HF) {
            if (m_xcvr.RegPaConfig.bits.PaSelect) {
                m_xcvr.RegPaConfig.bits.PaSelect = 0;
                m_xcvr.write_reg(REG_PACONFIG, m_xcvr.RegPaConfig.octet);
            }                
        } else { // LF...
            if (!m_xcvr.RegPaConfig.bits.PaSelect) {
                m_xcvr.RegPaConfig.bits.PaSelect = 1;
                m_xcvr.write_reg(REG_PACONFIG, m_xcvr.RegPaConfig.octet);
            }        
        }
    } else if (m_xcvr.type == SX1272) {
        // always PA_BOOST
        if (!m_xcvr.RegPaConfig.bits.PaSelect) {
            m_xcvr.RegPaConfig.bits.PaSelect = 1;
            m_xcvr.write_reg(REG_PACONFIG, m_xcvr.RegPaConfig.octet);
        }
    }
                    
                    
    // DIO0 to TxDone
    if (m_xcvr.RegDioMapping1.bits.Dio0Mapping != 1) {
        m_xcvr.RegDioMapping1.bits.Dio0Mapping = 1;
        m_xcvr.write_reg(REG_DIOMAPPING1, m_xcvr.RegDioMapping1.octet);
    }
    
    // set FifoPtrAddr to FifoTxPtrBase
    m_xcvr.write_reg(REG_LR_FIFOADDRPTR, m_xcvr.read_reg(REG_LR_FIFOTXBASEADDR));
    
    // write PayloadLength bytes to fifo
    write_fifo(len);

    if (m_xcvr.HF)
        m_xcvr.femctx = 1;
    else
        m_xcvr.femcps = 0;
        
    // radio doesnt provide FhssChangeChannel with channel=0 for TX    
    if (RegHopPeriod > 0)
        m_xcvr.write_reg_u24(REG_FRFMSB, m_xcvr.frfs[0]);
        
    m_xcvr.set_opmode(RF_OPMODE_TRANSMITTER);
}

void SX127x_lora::start_rx()
{
    if (!m_xcvr.RegOpMode.bits.LongRangeMode)
        return;

    if (m_xcvr.HF)
        m_xcvr.femctx = 0;
    else
        m_xcvr.femcps = 1;
        
    if (m_xcvr.RegDioMapping1.bits.Dio0Mapping != 0) {
        m_xcvr.RegDioMapping1.bits.Dio0Mapping = 0;    // DIO0 to RxDone
        m_xcvr.write_reg(REG_DIOMAPPING1, m_xcvr.RegDioMapping1.octet);
    }
    
    m_xcvr.write_reg(REG_LR_FIFOADDRPTR, m_xcvr.read_reg(REG_LR_FIFORXBASEADDR));
    
    // shouldn't be necessary, radio should provide FhssChangeChannel with channel=0 for RX  
    if (RegHopPeriod > 0)
        m_xcvr.write_reg_u24(REG_FRFMSB, m_xcvr.frfs[0]);
    
    m_xcvr.set_opmode(RF_OPMODE_RECEIVER);
}

float SX127x_lora::get_pkt_rssi()
{
    /* TODO: calculating with pktSNR to give meaningful result below noise floor */
    if (m_xcvr.type == SX1276)
        return RegPktRssiValue - 137;
    else
        return RegPktRssiValue - 125;
}

service_action_e SX127x_lora::service()
{
    if (m_xcvr.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER) {
        if (poll_vh) {
            RegIrqFlags.octet = m_xcvr.read_reg(REG_LR_IRQFLAGS);
            if (RegIrqFlags.bits.ValidHeader) {
                RegIrqFlags.octet = 0;
                RegIrqFlags.bits.ValidHeader = 1;
                m_xcvr.write_reg(REG_LR_IRQFLAGS, RegIrqFlags.octet);
                printf("VH\r\n");
            }
        }
    }
    
    // FhssChangeChannel
    if (m_xcvr.RegDioMapping1.bits.Dio1Mapping == 1) {
        if (m_xcvr.dio1) {
            RegHopChannel.octet = m_xcvr.read_reg(REG_LR_HOPCHANNEL);    
            m_xcvr.write_reg_u24(REG_FRFMSB, m_xcvr.frfs[RegHopChannel.bits.FhssPresentChannel]);
            printf("hopch:%d\r\n", RegHopChannel.bits.FhssPresentChannel);
            RegIrqFlags.octet = 0;
            RegIrqFlags.bits.FhssChangeChannel = 1;
           m_xcvr. write_reg(REG_LR_IRQFLAGS, RegIrqFlags.octet);
            
        }
    }
    
    if (m_xcvr.dio0 == 0)
        return SERVICE_NONE;
        
    switch (m_xcvr.RegDioMapping1.bits.Dio0Mapping) {
        case 0: // RxDone
            /* user checks for CRC error in IrqFlags */
            RegIrqFlags.octet = m_xcvr.read_reg(REG_LR_IRQFLAGS);  // save flags
            RegHopChannel.octet = m_xcvr.read_reg(REG_LR_HOPCHANNEL);
            if (RegIrqFlags.bits.FhssChangeChannel) {
                m_xcvr.write_reg_u24(REG_FRFMSB, m_xcvr.frfs[RegHopChannel.bits.FhssPresentChannel]);
            }
            //printf("[%02x]", RegIrqFlags.octet);
            m_xcvr.write_reg(REG_LR_IRQFLAGS, RegIrqFlags.octet); // clear flags in radio
            
            /* any register of interest on received packet is read(saved) here */        
            RegModemStatus.octet = m_xcvr.read_reg(REG_LR_MODEMSTAT);          
            RegPktSnrValue = m_xcvr.read_reg(REG_LR_PKTSNRVALUE);
            RegPktRssiValue = m_xcvr.read_reg(REG_LR_PKTRSSIVALUE);
            RegRxNbBytes = m_xcvr.read_reg(REG_LR_RXNBBYTES);
    
            m_xcvr.write_reg(REG_LR_FIFOADDRPTR, m_xcvr.read_reg(REG_LR_FIFORXCURRENTADDR));
            read_fifo(RegRxNbBytes);
            return SERVICE_READ_FIFO;
        case 1: // TxDone
            if (m_xcvr.HF)
                m_xcvr.femctx = 0;
            else
                m_xcvr.femcps = 1;

            RegIrqFlags.octet = 0;
            RegIrqFlags.bits.TxDone = 1;
            m_xcvr.write_reg(REG_LR_IRQFLAGS, RegIrqFlags.octet);                  
            return SERVICE_TX_DONE;        
    } // ...switch (RegDioMapping1.bits.Dio0Mapping)
    
    return SERVICE_ERROR;    
}