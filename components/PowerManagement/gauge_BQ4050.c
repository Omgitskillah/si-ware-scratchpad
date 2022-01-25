#include "driver/i2c.h"
#include "gauge_BQ4050.h"
#include "centralManager_priv.h"


/* Local Defines */
#define TAG "PowerGauge"
#define BQ4050_SENSOR_ADDR      0x16
#define GOLDEN_FILE_SIZE        8192
#define FLASH_BLOCK_SIZE        32
#define GOLDEN_IMG_BLOCK_CNT    GOLDEN_FILE_SIZE / FLASH_BLOCK_SIZE
#define BQ4050_TIMOUT_MS        1000
#define MAC_BLOCK_COMMAND       0x44
#define MAX_WRITE_RETRIES       100

/* Local variables */
static GaugeWriteCommand_t init_command_set[] = 
{
    {.U.data = 0x01,   .address = Flash_DAconfig,           .size = 1, .successfully_written = 0},   //Enable 2 cells
    {.U.data = 0x0038, .address = Flash_MfgInit,            .size = 2, .successfully_written = 0},   //FET_EN = 1, GUAGE_EN = 1
    {.U.data = 0xFF,   .address = Flash_ProtectionA,        .size = 1, .successfully_written = 0},   //Enabled Protections A
    {.U.data = 0x3F,   .address = Flash_ProtectionB,        .size = 1, .successfully_written = 0},   //Enabled Protections B
    {.U.data = 0xD5,   .address = Flash_ProtectionC,        .size = 1, .successfully_written = 0},   //Enabled Protections C
    {.U.data = 0x0F,   .address = Flash_ProtectionD,        .size = 1, .successfully_written = 0},   //Enabled Protections D
    {.U.data = 0x0320, .address = Flash_LowTempCurrentLow,  .size = 2, .successfully_written = 0},   //LowTempCurrentLow = 800 mA
    {.U.data = 0x0320, .address = Flash_LowTempCurrentMed,  .size = 2, .successfully_written = 0},   //LowTempCurrentMed = 800 mA
    {.U.data = 0x0320, .address = Flash_LowTempCurrentHigh, .size = 2, .successfully_written = 0},   //LowTempCurrentHigh = 800 mA
    {.U.data = 0x06,   .address = Flash_TempEn,             .size = 1, .successfully_written = 0},   //Enable temperature
    {.U.data = 0x09F6, .address = Flash_CUVThreshold,       .size = 2, .successfully_written = 0},   //CUVThreshold = 2550 mV
    {.U.data = 0x0258, .address = Flash_OCThreshold,        .size = 2, .successfully_written = 0},   //OCThreshold = 600 mAh
    {.U.data = 0xC0,   .address = Flash_PreChargeCurrent,   .size = 1, .successfully_written = 0},   //PreChargeCurrent = 192 mA
    {.U.data = 0x0B54, .address = Flash_EDV0add,            .size = 2, .successfully_written = 0},   //EDV0 = 2900 mv
    {.U.data = 0x0BB8, .address = Flash_EDV1add,            .size = 2, .successfully_written = 0},   //EDV1 = 3000 mv
    {.U.data = 0x0BD7, .address = Flash_EDV2add,            .size = 2, .successfully_written = 0},   //EDV2 = 3031 mv
    {.U.data = 0x1102, .address = Flash_CEDVGaugeConfig,    .size = 2, .successfully_written = 0},   //CEDV Gauging Configuration : SME0 = 1, FCC_LIMIT = 1
    {.U.data = 0xD,    .address = Flash_CEDVsmoothConfig,   .size = 1, .successfully_written = 0},   //CEDVsmoothConfig : SMEN = 1, SMEXT = 1, SMOOTHEOC_EN = 1
    {.U.data = 0x41,   .address = Flash_FET_options,        .size = 1, .successfully_written = 0},   //FET Options register
};

uint8_t bq4050_golden_img[] = GOLDEN_REGISTER_FILE;
uint8_t bq4050_scratchpad[32];

/* function prototypes */
static esp_err_t gauge_MACWrite(uint16_t command, uint8_t *data_in, uint8_t size); 
static esp_err_t gauge_MACRead(uint16_t command, uint8_t *data_out);


static esp_err_t gauge_MACWrite(uint16_t command, uint8_t *data_in, uint8_t size) 
{
    // this could be clean up using a statem machine
    esp_err_t err = ESP_FAIL;

    if( ( NULL == data_in) && ( 0 != size) )
    {
        ESP_LOGW(TAG, "Invalid use of gauge_MACWrite()");
    }

    i2c_cmd_handle_t packet = i2c_cmd_link_create();

    if( NULL == packet )
    {
        ESP_LOGW(TAG, "failed to create I2C link");
    }
    else
    {
        err = i2c_master_start(packet);

        if( ESP_OK != err )
        {
            // exit I2C if not successfully started
            ESP_LOGW(TAG, "Error %u, failed to start I2C", err);
        }
        else
        {
            uint8_t write_address = BQ4050_SENSOR_ADDR | I2C_MASTER_WRITE;
            uint8_t command_byte_count = sizeof(command); // this will always be 2
            uint8_t total_byte_count = size + command_byte_count;
            // send write address 
            ERROR_CHECK( TAG, i2c_master_write_byte( packet, write_address, true) );
            // send MAC address
            ERROR_CHECK( TAG, i2c_master_write_byte(packet, MAC_BLOCK_COMMAND, true) );
            // send number of bites to write to the mac
            ERROR_CHECK( TAG, i2c_master_write_byte(packet, total_byte_count, true) );
            // send command
            ERROR_CHECK( TAG, i2c_master_write_byte(packet, (uint8_t)command, true) );
            ERROR_CHECK( TAG, i2c_master_write_byte(packet, (uint8_t)(command >> 8), true) );
            // send data
            for( uint8_t i = 0; i < size; i++ )
            {
                ERROR_CHECK( TAG, i2c_master_write_byte(packet, data_in[i], true) );
            }

            err = i2c_master_stop( packet );
            if( ESP_OK != err )
            {
                // could not send stop
                ESP_LOGW(TAG, "Error %u, failed to stop I2C", err);
            }
            else
            {
                // empty the queue
                err = i2c_master_cmd_begin( PM_I2C_NUM, packet, pdMS_TO_TICKS(BQ4050_TIMOUT_MS) );
                if( ESP_OK != err )
                {
                    // could not send 
                    ESP_LOGW(TAG, "Error %u, i2c_master_cmd_begin, cmd LSB: %02x, cmd MSB: %02x, size: %u", err, (uint8_t)command, (uint8_t)(command >> 8), size );
                }
            }
        }
        
        // delete the I2C instance
        i2c_cmd_link_delete(packet);
    }

    return err;
}

static esp_err_t gauge_MACRead(uint16_t command, uint8_t *data_out) 
{
    // this could be clean up using a statem machine
    esp_err_t err = ESP_FAIL;

    err = gauge_MACWrite( command, NULL, 0);

    if( ESP_OK != err )
    {
        ESP_LOGW(TAG, "failed to write command before the read");
    }
    else
    {
        i2c_cmd_handle_t packet = i2c_cmd_link_create();

        if( NULL == packet )
        {
            ESP_LOGW(TAG, "failed to create I2C link");
        }
        else
        {
            uint8_t write_address = BQ4050_SENSOR_ADDR | I2C_MASTER_WRITE;
            uint8_t read_address = BQ4050_SENSOR_ADDR | I2C_MASTER_READ;

            err = i2c_master_start(packet);

            if( ESP_OK != err )
            {
                // exit I2C if not successfully started
                ESP_LOGW(TAG, "Error %u, failed to start I2C", err);
            }
            else
            {
                uint8_t bytes_count = 0;
                // write chip write address
                ERROR_CHECK( TAG, i2c_master_write_byte(packet, write_address, true) );
                // write MAC address
                ERROR_CHECK( TAG, i2c_master_write_byte(packet, MAC_BLOCK_COMMAND, true) );
                // send restart
                ERROR_CHECK( TAG, i2c_master_start(packet) );
                // send chip read address
                ERROR_CHECK( TAG, i2c_master_write_byte(packet, read_address, true) );
                // read the bytes to expect
                ERROR_CHECK( TAG, i2c_master_read_byte(packet, &bytes_count, I2C_MASTER_ACK) ); 
                
                /** should check that this length 
                 * matches expected length of the command
                 * and throw a missmatch error and terminate
                 * to avoid a buffer overflow
                 **/

                // start I2C exchange
                ERROR_CHECK( TAG, i2c_master_cmd_begin(PM_I2C_NUM, packet, pdMS_TO_TICKS(BQ4050_TIMOUT_MS)) );
                // delete current i2c instance
                i2c_cmd_link_delete(packet);

                if( 0 == bytes_count )
                {
                    ESP_LOGW(TAG, "No bytes to read");
                }
                else
                {
                    // start a new I2C instance
                    i2c_cmd_handle_t packet = i2c_cmd_link_create();

                    if( NULL == packet )
                    {
                        ESP_LOGW(TAG, "failed to create I2C link");
                    }
                    else
                    {
                        // drop the command returned, might stand to benefit to forward this to the function caller
                        ERROR_CHECK( TAG, i2c_master_read_byte(packet, data_out, I2C_MASTER_ACK) );
                        bytes_count--;
                        ERROR_CHECK( TAG, i2c_master_read_byte(packet, data_out, I2C_MASTER_ACK) );
                        bytes_count--;

                        while( bytes_count < 1 )
                        {
                            // read and ack all the bytes
                            ERROR_CHECK( TAG, i2c_master_read_byte(packet, (data_out++), I2C_MASTER_ACK) );
                        }

                        // Nack the last byte
                        ERROR_CHECK( TAG, i2c_master_read_byte(packet, (data_out++), I2C_MASTER_NACK) );
                        // send i2c stop
                        ERROR_CHECK( TAG, i2c_master_stop(packet) );
                        // start the i2c reads
                        ERROR_CHECK( TAG, i2c_master_cmd_begin(PM_I2C_NUM, packet, pdMS_TO_TICKS(BQ4050_TIMOUT_MS)) );
                        // delete i2c instance
                        i2c_cmd_link_delete(packet);
                    }
                }
            }

        }
    }

    return err;
}

static uint16_t gauge_readWord(uint8_t command) {
    uint16_t data_out;

    i2c_cmd_handle_t packet = i2c_cmd_link_create();
    ERROR_CHECK(TAG, i2c_master_start(packet));
    ERROR_CHECK(TAG, i2c_master_write_byte(packet, (BQ4050_SENSOR_ADDR) | I2C_MASTER_WRITE, true));
    ERROR_CHECK(TAG, i2c_master_write_byte(packet, command, true));

    ERROR_CHECK(TAG, i2c_master_start(packet));
    ERROR_CHECK(TAG, i2c_master_write_byte(packet, (BQ4050_SENSOR_ADDR) | I2C_MASTER_READ, true));
    ERROR_CHECK(TAG, i2c_master_read_byte(packet, ((uint8_t *) &data_out + 0), I2C_MASTER_ACK));
    ERROR_CHECK(TAG, i2c_master_read_byte(packet, ((uint8_t *) &data_out + 1), I2C_MASTER_NACK));
    ERROR_CHECK(TAG, i2c_master_stop(packet));

    vTaskDelay(pdMS_TO_TICKS(20));
    ERROR_CHECK(TAG, i2c_master_cmd_begin(PM_I2C_NUM, packet, pdMS_TO_TICKS(BQ4050_TIMOUT_MS)));
    i2c_cmd_link_delete(packet);

    return data_out;
}

void gauge_readBatteryInfo(GaugeInfo_t *gaugeInfo) {
    gaugeInfo->battVoltage = gauge_readWord(SBS_VOLTAGE);
    gaugeInfo->cellVoltage1 = gauge_readWord(SBS_CELLVOLTAGE1);
    gaugeInfo->cellVoltage2 = gauge_readWord(SBS_CELLVOLTAGE2);
    gaugeInfo->timeToEmpty = gauge_readWord(SBS_AVERAGETIMETOEMPTY);
    gaugeInfo->timeToFull = gauge_readWord(SBS_AVERAGETIMETOFULL);
    gaugeInfo->chargingCurrent = gauge_readWord(SBS_CHARGINGCURRENT);

    gaugeInfo->fullCapacity = gauge_readWord(SBS_FULLCHARGECAPACITY);
    gaugeInfo->capacity = gauge_readWord(SBS_REMAININGCAPACITY);
//    gaugeInfo->batteryPercentage = (uint8_t) lround(fmin(fmax((gaugeInfo->fullCapacity != 0) ? (uint8_t) (gaugeInfo->capacity * 100.0 /fmax(gaugeInfo->fullCapacity,BATTERY_FULL_CAPACITY)) : 0, 0),100));
    gaugeInfo->batteryPercentage = gauge_readWord(SBS_RELATIVESTATEOFCHARGE);
    ESP_LOGI(TAG,"Relative State of charge is %d",gaugeInfo->batteryPercentage);

    //convert 100<->25 % on the original scale to 100<->0 % on the new reported scale
    int newCap = round((gaugeInfo->batteryPercentage - 25) * 100.0 / 75.0);
    if(newCap < 0)
        gaugeInfo->batteryPercentage = 0;
    else
        gaugeInfo->batteryPercentage = newCap;

    uint16_t temp = gauge_readWord(SBS_CURRENT);
    gaugeInfo->current = *((int16_t *) &temp);    //convert unsigned value to signed value

    gaugeInfo->temperature = (gauge_readWord(SBS_TEMPERATURE) / 10.0) - 273.15;

    uint32_t buff = 0;

    gaugeInfo->fC = (gauge_readWord(SBS_BATTERYSTATUS) & 0x20) >> 5;

    gauge_MACRead(MAC_CHARGINGSTATUS, (uint8_t *) &buff);
    gaugeInfo->VCT = (buff & 0x80) >> 7;

    gauge_MACRead(MAC_OPERATIONSTATUS, (uint8_t *) &gaugeInfo->operationStatus);
    gaugeInfo->xchg = (gaugeInfo->operationStatus & 0x4000) >> 14;
    gaugeInfo->xdischg = (gaugeInfo->operationStatus & 0x2000) >> 13;

    gauge_MACRead(MAC_SAFETYSTATUS, (uint8_t *) &gaugeInfo->safetyStatus);
    gauge_MACRead(MAC_PFSTATUS, (uint8_t *) &gaugeInfo->pfStatus);
    gauge_MACRead(MAC_GAUGINGSTATUS, (uint8_t *) &gaugeInfo->gaugingStatus);
    gauge_MACRead(MAC_MANUFACTURINGSTATUS, (uint8_t *) &gaugeInfo->mfgStatus);
    gauge_MACRead(MAC_SAFETYALERT,(uint8_t*)&gaugeInfo->safetyAlert);

    gauge_MACRead(Flash_Cycle_Count, bq4050_scratchpad);
    gaugeInfo->cycleCount = (uint16_t)bq4050_scratchpad[0] | ((uint16_t)bq4050_scratchpad[1] << 8);
}
void gauge_read_life_time_data(life_time_data_t* life_time_data){
    uint8_t *data_ptr = (uint8_t*)life_time_data;
    ESP_LOGI(TAG,"lifeTimeData size is %d",sizeof(life_time_data_t));
    gauge_MACRead(MAC_LIFETIMEDATABLOCK1,data_ptr);
    for (int i = 0,data_idx = 0; i < 5; ++i) {
        data_idx += gauge_MACRead(MAC_LIFETIMEDATABLOCK1 + i,&data_ptr[data_idx]);
        ESP_LOGI(TAG,"Read %d bytes",data_idx);
    }

}
void gauge_print_life_time_data(life_time_data_t * life_time_data){
    ESP_LOGI(TAG, "________________Life Time Data ________________________");
    ESP_LOGI(TAG,"cell_1_max_voltage = 0x%x",life_time_data -> cell_1_max_voltage);
    ESP_LOGI(TAG,"cell_2_max_voltage = 0x%x",life_time_data -> cell_2_max_voltage);
    ESP_LOGI(TAG,"cell_3_max_voltage = 0x%x",life_time_data -> cell_3_max_voltage);
    ESP_LOGI(TAG,"cell_4_max_voltage = 0x%x",life_time_data -> cell_4_max_voltage);
    ESP_LOGI(TAG,"cell_1_min_voltage = 0x%x",life_time_data -> cell_1_min_voltage);
    ESP_LOGI(TAG,"cell_2_min_voltage = 0x%x",life_time_data -> cell_2_min_voltage);
    ESP_LOGI(TAG,"cell_3_min_voltage = 0x%x",life_time_data -> cell_3_min_voltage);
    ESP_LOGI(TAG,"cell_4_min_voltage = 0x%x",life_time_data -> cell_4_min_voltage);
    ESP_LOGI(TAG,"max_delta_cell_voltage = 0x%x",life_time_data -> max_delta_cell_voltage);
    ESP_LOGI(TAG,"max_charge_current = 0x%x",life_time_data -> max_charge_current);
    ESP_LOGI(TAG,"max_discharge_current = 0x%x",life_time_data -> max_discharge_current);
    ESP_LOGI(TAG,"max_avg_dsg_current = 0x%x",life_time_data -> max_avg_dsg_current);
    ESP_LOGI(TAG,"max_avg_dsg_power = 0x%x",life_time_data -> max_avg_dsg_power);
    ESP_LOGI(TAG,"max_temp_cell = 0x%x",life_time_data ->max_temp_cell);
    ESP_LOGI(TAG,"min_temp_cell = 0x%x",life_time_data ->min_temp_cell);
    ESP_LOGI(TAG,"max_delta_cell_temp = 0x%x",life_time_data ->max_delta_cell_temp);
    ESP_LOGI(TAG,"max_temp_int_sensor = 0x%x",life_time_data ->max_temp_int_sensor);
    ESP_LOGI(TAG,"min_temp_int_sensor = 0x%x",life_time_data ->min_temp_int_sensor);
    ESP_LOGI(TAG,"max_temp_fet = 0x%x",life_time_data ->max_temp_fet);
    ESP_LOGI(TAG,"num_of_shutdowns = 0x%x",life_time_data ->num_of_shutdowns);
    ESP_LOGI(TAG,"num_of_partial_resets = 0x%x",life_time_data ->num_of_partial_resets);
    ESP_LOGI(TAG,"num_of_full_resets = 0x%x",life_time_data ->num_of_full_resets);
    ESP_LOGI(TAG,"num_of_wdt_resets = 0x%x",life_time_data ->num_of_wdt_resets);
    ESP_LOGI(TAG,"cb_time_cell_1 = 0x%x",life_time_data ->cb_time_cell_1);
    ESP_LOGI(TAG,"cb_time_cell_2 = 0x%x",life_time_data ->cb_time_cell_2);
    ESP_LOGI(TAG,"cb_time_cell_3 = 0x%x",life_time_data ->cb_time_cell_3);
    ESP_LOGI(TAG,"cb_time_cell_4 = 0x%x",life_time_data ->cb_time_cell_4);
    ESP_LOGI(TAG,"total_fw_runtime = 0x%x",life_time_data -> total_fw_runtime);
    ESP_LOGI(TAG,"time_spent_in_ut = 0x%x",life_time_data -> time_spent_in_ut);
    ESP_LOGI(TAG,"time_spent_in_lt = 0x%x",life_time_data -> time_spent_in_lt);
    ESP_LOGI(TAG,"Time_Spent_in_STL = 0x%x",life_time_data -> Time_Spent_in_STL);
    ESP_LOGI(TAG,"time_spent_in_rt = 0x%x",life_time_data -> time_spent_in_rt);
    ESP_LOGI(TAG,"Time_Spent_in_STH = 0x%x",life_time_data -> Time_Spent_in_STH);
    ESP_LOGI(TAG,"time_spent_in_ht = 0x%x",life_time_data -> time_spent_in_ht);
    ESP_LOGI(TAG,"time_spent_in_ot = 0x%x",life_time_data -> time_spent_in_ot);
    ESP_LOGI(TAG,"num_of_cov_events = 0x%x",life_time_data -> num_of_cov_events);
    ESP_LOGI(TAG,"last_cov_event = 0x%x",life_time_data -> last_cov_event);
    ESP_LOGI(TAG,"num_of_cuv_events = 0x%x",life_time_data -> num_of_cuv_events);
    ESP_LOGI(TAG,"last_cuv_event = 0x%x",life_time_data -> last_cuv_event);
    ESP_LOGI(TAG,"num_of_ocd1_events = 0x%x",life_time_data -> num_of_ocd1_events);
    ESP_LOGI(TAG,"last_ocd1_event = 0x%x",life_time_data -> last_ocd1_event);
    ESP_LOGI(TAG,"num_of_ocd2_events = 0x%x",life_time_data -> num_of_ocd2_events);
    ESP_LOGI(TAG,"last_ocd2_event = 0x%x",life_time_data -> last_ocd2_event);
    ESP_LOGI(TAG,"num_of_occ1_events = 0x%x",life_time_data -> num_of_occ1_events);
    ESP_LOGI(TAG,"last_occ1_event = 0x%x",life_time_data -> last_occ1_event);
    ESP_LOGI(TAG,"num_of_occ2_events = 0x%x",life_time_data -> num_of_occ2_events);
    ESP_LOGI(TAG,"last_occ2_event = 0x%x",life_time_data -> last_occ2_event);
    ESP_LOGI(TAG,"num_of_aold_events = 0x%x",life_time_data -> num_of_aold_events);
    ESP_LOGI(TAG,"last_aold_event = 0x%x",life_time_data -> last_aold_event);
    ESP_LOGI(TAG,"num_of_ascd_events = 0x%x",life_time_data -> num_of_ascd_events);
    ESP_LOGI(TAG,"last_ascd_event = 0x%x",life_time_data -> last_ascd_event);
    ESP_LOGI(TAG,"num_of_ascc_events = 0x%x",life_time_data -> num_of_ascc_events);
    ESP_LOGI(TAG,"last_ascc_event = 0x%x",life_time_data -> last_ascc_event);
    ESP_LOGI(TAG,"num_of_otc_events = 0x%x",life_time_data -> num_of_otc_events);
    ESP_LOGI(TAG,"last_otc_event = 0x%x",life_time_data -> last_otc_event);
    ESP_LOGI(TAG,"num_of_otd_events = 0x%x",life_time_data -> num_of_otd_events);
    ESP_LOGI(TAG,"last_otd_event = 0x%x",life_time_data -> last_otd_event);
    ESP_LOGI(TAG,"num_of_otf_events = 0x%x",life_time_data -> num_of_otf_events);
    ESP_LOGI(TAG,"last_otf_event = 0x%x",life_time_data -> last_otf_event);
    ESP_LOGI(TAG,"num_valid_charge_term = 0x%x",life_time_data -> num_valid_charge_term);
    ESP_LOGI(TAG,"last_valid_charge_term = 0x%x",life_time_data -> last_valid_charge_term);
    ESP_LOGI(TAG, "________________________________________");

}

ScannerError_t gauge_selfTest() {
    uint32_t mfgStatus;
    gauge_MACRead(MAC_MANUFACTURINGSTATUS, (uint8_t *) &mfgStatus);

    if ((mfgStatus & 0x10) != 0)
        return NoError;
    else
        return GAUGE_SELF_TEST_FAILED;
}

void gauge_printBatteryInfo(GaugeInfo_t *gaugeInfo) {
    ESP_LOGI(TAG, "________________________________________");
    ESP_LOGI(TAG, "battVoltage = %d", gaugeInfo->battVoltage);
    ESP_LOGI(TAG, "Current = %d", gaugeInfo->current);
    ESP_LOGI(TAG, "chargingCurrent = %d", gaugeInfo->chargingCurrent);
    ESP_LOGI(TAG, "capacity = %d", gaugeInfo->capacity);
    ESP_LOGI(TAG, "fullCapacity = %d", gaugeInfo->fullCapacity);
    ESP_LOGI(TAG, "Battery Capacity Percent = %d%%", gaugeInfo->batteryPercentage);
    ESP_LOGI(TAG, "temperature = %d", gaugeInfo->temperature);
    ESP_LOGI(TAG, "cellVoltage1 = %d", gaugeInfo->cellVoltage1);
    ESP_LOGI(TAG, "cellVoltage2 = %d", gaugeInfo->cellVoltage2);
    ESP_LOGI(TAG, "timeToEmpty = %d", gaugeInfo->timeToEmpty);
    ESP_LOGI(TAG, "timeToFull = %d", gaugeInfo->timeToFull);
    ESP_LOGI(TAG, "fC = %d", gaugeInfo->fC);
    ESP_LOGI(TAG, "VCT = %d", gaugeInfo->VCT);
    ESP_LOGI(TAG, "operationStatus = 0x%X", gaugeInfo->operationStatus);
    ESP_LOGI(TAG, "xchg = %d", gaugeInfo->xchg);
    ESP_LOGI(TAG, "xdischg = %d", gaugeInfo->xdischg);
    ESP_LOGI(TAG, "safetyStatus = 0x%0X", gaugeInfo->safetyStatus);
    ESP_LOGI(TAG, "safetyAlert = 0x%0X", gaugeInfo->safetyAlert);
    ESP_LOGI(TAG, "pfStatus = 0x%0X", gaugeInfo->pfStatus);
    ESP_LOGI(TAG, "gaugingStatus = 0x%0X", gaugeInfo->gaugingStatus);
    ESP_LOGI(TAG, "mfgStatus = 0x%0X", gaugeInfo->mfgStatus);
    ESP_LOGI(TAG,"CycleCount = 0x%0x",gaugeInfo->cycleCount);
    ESP_LOGI(TAG,"Size = %d",sizeof(GaugeInfo_t));
    ESP_LOGI(TAG, "----------------------------------------");
}

void gauge_deviceReset() {
    if( ESP_OK == gauge_MACWrite( MAC_DEVICERESET, NULL, 0 ) )
    {
        ESP_LOGI(TAG, "RESET GAUGE SUCCESS");
    }
    else
    {
        ESP_LOGI(TAG, "RESET GAUGE FAILED");
    }
    vTaskDelay(1000 / portTICK_RATE_MS);
}

void gauge_disable() {
    uint8_t data[] = { 0x00, 0x00 };
    if( ESP_OK == gauge_MACWrite( Flash_MfgInit, data, sizeof(data) ) )
    {
        ESP_LOGI(TAG, "DISABLE GAUGE SUCCESS");
    }
    else
    {
        ESP_LOGI(TAG, "DISABLE GAUGE FAILED");
    }
}

void gauge_enable(){
    uint8_t data[] = { 0x18, 0x00 };
    if( ESP_OK == gauge_MACWrite( Flash_MfgInit, data, sizeof(data) ) )
    {
        ESP_LOGI(TAG, "ENABLE GAUGE SUCCESS");
    }
    else
    {
        ESP_LOGI(TAG, "ENABLE GAUGE FAILED");
    }
}

void gauge_init() {

    uint8_t num_operations = sizeof(init_command_set) / sizeof(init_command_set[0]);

    uint8_t i = 0;
    uint8_t local_data_buffer[2];

    while( i < num_operations )
    {
        uint16_t current_cmd = init_command_set[i].address;
        uint16_t current_cmd_size = init_command_set[i].size;
        local_data_buffer[0] = init_command_set[i].U.data;
        local_data_buffer[1] = init_command_set[i].U.data >> 8;

        if( ESP_OK != gauge_MACWrite( current_cmd, local_data_buffer, current_cmd_size ) )
        {
            ESP_LOGI(TAG, "failed to write cmd: %04x, size: %u", current_cmd, current_cmd_size );
        }
    }
}

void gauge_writeGoldenFile( void ) {
    /**
     * According to the datasheet we can write to the flash in 32 byte blocks, starting from address 0x4000
     * That means we need 8192/32 = 256 write commands, instead of allocating 256 * 8 bytes we can use the memory just next to the golden file
     * since it's 64k and we need only the first 8k, so there is enough memory
     */
    
    ESP_LOGI(TAG, "Buring Golden Image");

    uint16_t current_address = Flash_Data_Access;
    uint32_t golden_img_index = 0;

    for( uint32_t i = 0; i < GOLDEN_IMG_BLOCK_CNT; i++ )
    {
        gauge_MACWrite( current_address, &bq4050_golden_img[golden_img_index], FLASH_BLOCK_SIZE );
        current_address += FLASH_BLOCK_SIZE;
        golden_img_index += FLASH_BLOCK_SIZE;
    }

}