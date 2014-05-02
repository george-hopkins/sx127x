#include "sx127x.h"

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

SX127x::SX127x(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName rst, PinName dio_0, PinName dio_1, PinName fem_ctx, PinName fem_cps) :
                m_spi(mosi, miso, sclk),                  m_cs(cs), reset_pin(rst), dio0(dio_0), dio1(dio_1), femctx(fem_ctx), femcps(fem_cps)
{
    reset_pin.input();
    m_cs = 1;
    m_spi.format(8, 0);
    m_spi.frequency(3000000);
    
    init();
}

SX127x::~SX127x()
{
    set_opmode(RF_OPMODE_SLEEP);
}

void SX127x::init()
{
    type = SX_NONE;

    RegOpMode.octet = read_reg(REG_OPMODE);
    RegPaConfig.octet = read_reg(REG_PACONFIG);
    RegLna.octet = read_reg(REG_LNA);
    RegDioMapping1.octet = read_reg(REG_DIOMAPPING1);
    RegDioMapping2.octet = read_reg(REG_DIOMAPPING2);
    
/*    if (!RegOpMode.bits.LongRangeMode) {
        if (RegOpMode.bits.Mode != RF_OPMODE_SLEEP)
            set_opmode(RF_OPMODE_SLEEP);
        RegOpMode.bits.LongRangeMode = 1;
        write_reg(REG_OPMODE, RegOpMode.octet);
    }*/
    
    get_type();
    
    if (type == SX1272) {
        // turn on PA BOOST, eval boards are wired for this connection
        RegPaConfig.bits.PaSelect = 1;
        write_reg(REG_PACONFIG, RegPaConfig.octet);
    }
}

void SX127x::get_type()
{
    RegOpMode.octet = read_reg(REG_OPMODE);
    if (RegOpMode.sx1276LORAbits.LowFrequencyModeOn)
        type = SX1276;
    else {
        RegOpMode.sx1276LORAbits.LowFrequencyModeOn = 1;
        write_reg(REG_OPMODE, RegOpMode.octet);
        RegOpMode.octet = read_reg(REG_OPMODE);
        if (RegOpMode.sx1276LORAbits.LowFrequencyModeOn)
            type = SX1276;
        else
            type = SX1272;
    }
}

void
SX127x::ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;
    
    m_cs = 0;
    
    m_spi.write(addr); // bit7 is low for reading from radio

    for( i = 0; i < size; i++ )
    {
        buffer[i] = m_spi.write(0x00);
    }
    
    m_cs = 1;
}

uint8_t SX127x::read_reg(uint8_t addr)
{
    uint8_t ret;
    // Select the device by seting chip select low
    m_cs = 0;

    m_spi.write(addr); // bit7 is low for reading from radio
 
    // Send a dummy byte to receive the contents of register
    ret = m_spi.write(0x00);
 
    // Deselect the device
    m_cs = 1;
    
    return ret;
}

int16_t SX127x::read_s16(uint8_t addr)
{
    int16_t ret;
    // Select the device by seting chip select low
    m_cs = 0;

    m_spi.write(addr); // bit7 is low for reading from radio
 
    // Send a dummy byte to receive the contents of register
    ret = m_spi.write(0x00);
    ret <<= 8;
    ret += m_spi.write(0x00);
 
    // Deselect the device
    m_cs = 1;
    
    return ret;
}

uint16_t SX127x::read_u16(uint8_t addr)
{
    uint16_t ret;
    // Select the device by seting chip select low
    m_cs = 0;

    m_spi.write(addr); // bit7 is low for reading from radio
 
    // Send a dummy byte to receive the contents of register
    ret = m_spi.write(0x00);
    ret <<= 8;
    ret += m_spi.write(0x00);
 
    // Deselect the device
    m_cs = 1;
    
    return ret;
}

void SX127x::write_u16(uint8_t addr, uint16_t data)
{
    m_cs = 0;   // Select the device by seting chip select low

    m_spi.write(addr | 0x80); // bit7 is high for writing to radio
    m_spi.write((data >> 8) & 0xff);
    m_spi.write(data & 0xff);
 
    m_cs = 1;   // Deselect the device
}

void SX127x::write_u24(uint8_t addr, uint32_t data)
{
    m_cs = 0;   // Select the device by seting chip select low

    m_spi.write(addr | 0x80); // bit7 is high for writing to radio
    m_spi.write((data >> 16) & 0xff);
    m_spi.write((data >> 8) & 0xff);
    m_spi.write(data & 0xff);
 
    m_cs = 1;   // Deselect the device
    
    if (addr == REG_FRFMSB) {
        if (data < 0x8340000)   // < 525MHz
            HF = false;
        else
            HF = true;
    }
}

void SX127x::write_reg(uint8_t addr, uint8_t data)
{
    m_cs = 0;   // Select the device by seting chip select low

    m_spi.write(addr | 0x80); // bit7 is high for writing to radio
    m_spi.write(data);
 
    m_cs = 1;   // Deselect the device
}

void SX127x::WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    m_cs = 0;   // Select the device by seting chip select low

    m_spi.write(addr | 0x80); // bit7 is high for writing to radio
    for( i = 0; i < size; i++ )
    {
        m_spi.write(buffer[i]);
    }

    m_cs = 1;   // Deselect the device
}

void SX127x::set_opmode(chip_mode_e mode)
{
    RegOpMode.bits.Mode = mode;
    write_reg(REG_OPMODE, RegOpMode.octet);
}

void SX127x::set_frf_MHz( float MHz )
{
    uint32_t frf;
    
    frf = MHz / FREQ_STEP_MHZ;
    write_u24(REG_FRFMSB, frf);
    
    if (MHz < 525)
        HF = false;
    else
        HF = true;
}

float SX127x::get_frf_MHz(void)
{
    uint32_t frf;
    uint8_t lsb, mid, msb;
    float MHz;
    
    msb = read_reg(REG_FRFMSB);
    mid = read_reg(REG_FRFMID);
    lsb = read_reg(REG_FRFLSB);
    frf = msb;
    frf <<= 8;
    frf += mid;
    frf <<= 8;
    frf += lsb;
    
    MHz = frf * FREQ_STEP_MHZ;
    
    if (MHz < 525)
        HF = false;
    else
        HF = true;
        
    return MHz;
}

void SX127x::hw_reset()
{
    /* only a french-swiss design would have hi-Z deassert */
    reset_pin.output();
    reset_pin.write(1);
    wait(0.05);
    reset_pin.input();
    wait(0.05);
}

