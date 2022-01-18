#include "driver/i2c.h"

#include "charger_BQ25703A.h"

#define TAG "PowerCharger"


static void charger_writeByte(uint8_t regAddress, uint8_t data_in)
{
	i2c_cmd_handle_t packet = i2c_cmd_link_create();
	ERROR_CHECK(TAG,i2c_master_start(packet));

	ERROR_CHECK(TAG,i2c_master_write_byte(packet, (BQ25703A_CHIP_ADDR << 1) | I2C_MASTER_WRITE, true));
	ERROR_CHECK(TAG,i2c_master_write_byte(packet, regAddress, true));
	ERROR_CHECK(TAG,i2c_master_write_byte(packet, data_in, true));

	ERROR_CHECK(TAG,i2c_master_stop(packet));
	ERROR_CHECK(TAG,i2c_master_cmd_begin(PM_I2C_NUM, packet, 1000 / portTICK_RATE_MS));
	i2c_cmd_link_delete(packet);
}

static uint8_t charger_readByte(uint8_t regAddress)
{
	uint8_t data_out;

	i2c_cmd_handle_t packet = i2c_cmd_link_create();
	ERROR_CHECK(TAG,i2c_master_start(packet));

	ERROR_CHECK(TAG,i2c_master_write_byte(packet, (BQ25703A_CHIP_ADDR << 1) | I2C_MASTER_WRITE, true));
	ERROR_CHECK(TAG,i2c_master_write_byte(packet, regAddress, true));
	ERROR_CHECK(TAG,i2c_master_start(packet));
	ERROR_CHECK(TAG,i2c_master_write_byte(packet, (BQ25703A_CHIP_ADDR << 1) | I2C_MASTER_READ, true));
	ERROR_CHECK(TAG,i2c_master_read_byte(packet, &data_out, I2C_MASTER_NACK));

	ERROR_CHECK(TAG,i2c_master_stop(packet));
	ERROR_CHECK(TAG,i2c_master_cmd_begin(PM_I2C_NUM, packet, 1000 / portTICK_RATE_MS));
	i2c_cmd_link_delete(packet);
    
	return data_out;
}

static void charger_setSettings(ChargerOption_t option)
{
    if(option == OPTION_ENABLE_LOWPOWER)
        charger_writeByte(REG_CHARGEOPTION0_HIGH, 
                    charger_readByte(REG_CHARGEOPTION0_HIGH) | option);
    else
        charger_writeByte(REG_CHARGEOPTION0_HIGH, 
                    charger_readByte(REG_CHARGEOPTION0_HIGH) & option);
}

static void charger_configureADC(ChargerADCOption_t option)
{
    if(option == ADC_VBUS_DISABLE || option == ADC_CURRENT_DISABLE)
        charger_writeByte(REG_ADCOPTION_WORD, 
                    charger_readByte(REG_ADCOPTION_WORD) & option);
    else
    {
        charger_writeByte(REG_ADCOPTION_WORD, //Set bit in low byte
                    charger_readByte(REG_ADCOPTION_WORD) | option); 
        charger_writeByte(REG_ADCOPTION_WORD + 1, //start ADC in high byte
                    charger_readByte(REG_ADCOPTION_WORD + 1) | 0x40);  
    }
}

static void charger_setCurrent(ChargerCurrentOption_t option)
{
    charger_setSettings(OPTION_DISABLE_LOWPOWER);

    uint16_t regValue = charger_readByte(REG_CHARGECURRENT_WORD);               //read low byte
    regValue |= ((uint16_t)charger_readByte(REG_CHARGECURRENT_WORD + 1)) << 8;  //read high byte

    regValue = (regValue & ~CURRENT_MAX) | option;
    charger_writeByte(REG_CHARGECURRENT_WORD, regValue & 0xFF);                 //write low byte
    charger_writeByte(REG_CHARGECURRENT_WORD + 1, (regValue >> 8) & 0xFF);      //write high byte

    charger_setSettings(OPTION_ENABLE_LOWPOWER);
}

static uint16_t charger_readVBUS()
{
    uint16_t vbus = 3200;

    charger_setSettings(OPTION_DISABLE_LOWPOWER);
    charger_configureADC(ADC_VBUS_ENABLE);

	vbus += (uint16_t)(charger_readByte(REG_ADCVBUS_HIGH)&0x7F) * 64; 

    charger_configureADC(ADC_VBUS_ENABLE);
    charger_setSettings(OPTION_ENABLE_LOWPOWER);

	return vbus;
}

/* For debugging */
void charger_readCurrent()
{
    charger_setSettings(OPTION_DISABLE_LOWPOWER);
    charger_configureADC(ADC_CURRENT_ENABLE);

	ESP_LOGI(TAG, "Battery discharging current = %d mA", (uint16_t)(charger_readByte(REG_ADCIBAT_WORD)&0x7F) * 256); //read Low byte
	ESP_LOGI(TAG, "Battery charging current = %d mA", (uint16_t)(charger_readByte(REG_ADCIBAT_WORD + 1)&0x7F) * 64); //read High byte

    charger_configureADC(ADC_CURRENT_DISABLE);
    charger_setSettings(OPTION_ENABLE_LOWPOWER);
}

/* For debugging */
void charger_readStatus()
{
	ESP_LOGI(TAG, "REG_CHARGER_STATUS_HIGH_BYTE now = 0x%X\n", 
	    charger_readByte(REG_CHARGER_STATUS_HIGH));
}

ScannerError_t charger_selfTest()
{
    charger_setSettings(OPTION_DISABLE_WATCHDOG);
    uint16_t vbus = charger_readVBUS();

	if(vbus >= 4500 && vbus <= 5200)
		return NoError;
	else
		return CHARGER_SELF_TEST_FAILED;
}

ChargerState_t charger_start()
{
    charger_setSettings(OPTION_DISABLE_WATCHDOG);
    uint16_t vbus = charger_readVBUS();
    
    ESP_LOGI(TAG, "Charger started with input voltage = %d mV", vbus);

    if(vbus >= 6600)
    {
        charger_setCurrent(CURRENT_512mA);
        return ChargerFast;
    }
	else if(vbus < 6600 && vbus != 3200)
    {
        charger_setCurrent(CURRENT_192mA);
        return ChargerSlow;
    }
    else 
        return ChargerUnplugged;
}

ChargerState_t charger_stop()
{
    charger_setCurrent(CURRENT_ZERO);
    ESP_LOGI(TAG, "Charger stopped");
    
    return ChargerUnplugged;
}