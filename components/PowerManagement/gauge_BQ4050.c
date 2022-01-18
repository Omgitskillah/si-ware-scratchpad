#include "driver/i2c.h"

#include "gauge_BQ4050.h"

#define TAG "PowerGauge"

static esp_err_t gauge_MACWrite(uint16_t command, uint8_t *data_in, uint8_t size) {
    esp_err_t err = ESP_OK;
    i2c_cmd_handle_t packet = i2c_cmd_link_create();
    ERROR_CHECK(TAG, err |= i2c_master_start(packet));
    ERROR_CHECK(TAG, err |= i2c_master_write_byte(packet, (BQ4050_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true));
    ERROR_CHECK(TAG, err |= i2c_master_write_byte(packet, MAC_BLOCK_COMMAND, true));
    ERROR_CHECK(TAG, err |= i2c_master_write_byte(packet, size + 2, true));
    ERROR_CHECK(TAG, err |= i2c_master_write_byte(packet, *((uint8_t *) &command + 0), true));
    ERROR_CHECK(TAG, err |= i2c_master_write_byte(packet, *((uint8_t *) &command + 1), true));
    if (data_in != NULL) {
        ERROR_CHECK(TAG, err |= i2c_master_write(packet, data_in, size, true));
    }
    ERROR_CHECK(TAG, err |= i2c_master_stop(packet));
    if (ESP_OK != err) {
        ESP_LOGW(TAG, "Error %u before I2C Command Begin", err);
        return err;
    }
    ERROR_CHECK(TAG, err = i2c_master_cmd_begin(PM_I2C_NUM, packet, pdMS_TO_TICKS(10000)));
    if (ESP_OK != err) {
        ESP_LOGE(TAG, "Error @ I2C address %x", command);
    }
//    vTaskDelay(pdMS_TO_TICKS(20));
    i2c_cmd_link_delete(packet);
    return err;
}

static esp_err_t gauge_writeCommands(GaugeWriteCommand_t *commands, int num_commands, bool use_data_ptr) {
    uint8_t unable_to_write_ctr = 0;
//    uint16_t able_to_write_ctr;
    uint8_t trial_ctr = 0;
    do {
        unable_to_write_ctr = 0;
//        able_to_write_ctr = 0;
        for (int i = 0; i < num_commands; ++i) {
            if (!commands[i].successfully_written) {
                uint8_t *data_to_send = (use_data_ptr ? commands[i].U.data_ptr : (uint8_t *) &commands[i].U.data);
//                ESP_LOGI(TAG,"About To write Command %x, with size %d, and first byte is %x and it's %d yet written",commands[i].address,commands[i].size,
//                data_to_send == NULL ? 0x00:data_to_send[0],commands[i].successfully_written);
                commands[i].successfully_written = (ESP_OK == gauge_MACWrite(commands[i].address, data_to_send,
                                                                             commands[i].size));
                unable_to_write_ctr += !commands[i].successfully_written;
//                able_to_write_ctr += commands[i].successfully_written;
//                ESP_LOGI(TAG, "Remaining Items to write is %d, Written Items is %d", unable_to_write_ctr,able_to_write_ctr);
            } else {
//                ESP_LOGI(TAG,"Already Written this block");
            }
        }
//        ESP_LOGI(TAG, "Remaining Items to write is %d, Written Items is %d", unable_to_write_ctr,able_to_write_ctr);

    } while ((unable_to_write_ctr != 0) && (++trial_ctr < MAX_WRITE_RETRIES));
    if (trial_ctr >= MAX_WRITE_RETRIES) {
        ESP_LOGW(TAG, "Failed after %d Trials", MAX_WRITE_RETRIES);
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "Finished Writing Successfully");
        return ESP_OK;
    }
}

static uint8_t gauge_MACRead(uint16_t command, uint8_t *data_out) {
    uint8_t len = 0, dummy = 0;

    gauge_MACWrite(command, NULL, 0);

    i2c_cmd_handle_t packet = i2c_cmd_link_create();
    ERROR_CHECK(TAG, i2c_master_start(packet));
    ERROR_CHECK(TAG, i2c_master_write_byte(packet, BQ4050_SENSOR_ADDR << 1 | I2C_MASTER_WRITE, true));
    ERROR_CHECK(TAG, i2c_master_write_byte(packet, MAC_BLOCK_COMMAND, true));
    ERROR_CHECK(TAG, i2c_master_start(packet));
    ERROR_CHECK(TAG, i2c_master_write_byte(packet, BQ4050_SENSOR_ADDR << 1 | I2C_MASTER_READ, true));
    ERROR_CHECK(TAG, i2c_master_read_byte(packet, &len, I2C_MASTER_ACK));

    ERROR_CHECK(TAG, i2c_master_cmd_begin(PM_I2C_NUM, packet, 10000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(packet);
    if(len == 0){
        return 0;
    }
    packet = i2c_cmd_link_create();
    ERROR_CHECK(TAG, i2c_master_read_byte(packet, &dummy, I2C_MASTER_ACK)); //read command byte0
    ERROR_CHECK(TAG, i2c_master_read_byte(packet, &dummy, I2C_MASTER_ACK)); //read command byte1
    ERROR_CHECK(TAG,i2c_master_read(packet,data_out,len-3,I2C_MASTER_ACK));

//    for (uint8_t i = 0; i < len - 3; i++)
//    ERROR_CHECK(TAG, i2c_master_read_byte(packet, &data_out[i], I2C_MASTER_ACK));

    ERROR_CHECK(TAG, i2c_master_read_byte(packet, &data_out[len - 3], I2C_MASTER_NACK));
    ERROR_CHECK(TAG, i2c_master_stop(packet));

    ERROR_CHECK(TAG, i2c_master_cmd_begin(PM_I2C_NUM, packet, 10000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(packet);
    return len -2;
}
static void gauge_flashRead(uint16_t starting_address, uint8_t *data_out,uint8_t length){
    //The flash read always returns a 32 byte of data from the flash, we don't need them
    //so we only read the first length bytes
    uint8_t len = 0, dummy = 0;

    gauge_MACWrite(starting_address, NULL, 0);

    i2c_cmd_handle_t packet = i2c_cmd_link_create();
    ERROR_CHECK(TAG, i2c_master_start(packet));
    ERROR_CHECK(TAG, i2c_master_write_byte(packet, BQ4050_SENSOR_ADDR << 1 | I2C_MASTER_WRITE, true));
    ERROR_CHECK(TAG, i2c_master_write_byte(packet, MAC_BLOCK_COMMAND, true));
    ERROR_CHECK(TAG, i2c_master_start(packet));
    ERROR_CHECK(TAG, i2c_master_write_byte(packet, BQ4050_SENSOR_ADDR << 1 | I2C_MASTER_READ, true));
    ERROR_CHECK(TAG, i2c_master_read_byte(packet, &len, I2C_MASTER_ACK));

    ERROR_CHECK(TAG, i2c_master_cmd_begin(PM_I2C_NUM, packet, 10000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(packet);
//    ESP_LOGI(TAG,"Flash Read Length = %d",len); //should be 34
    packet = i2c_cmd_link_create();
    ERROR_CHECK(TAG, i2c_master_read_byte(packet, &dummy, I2C_MASTER_ACK)); //read command byte0
    ERROR_CHECK(TAG, i2c_master_read_byte(packet, &dummy, I2C_MASTER_ACK)); //read command byte1
    ERROR_CHECK(TAG,len-2 >= length ? 0 : ESP_FAIL);    //assertion that the length required is less than 32
    if(len-2 > length){
        ESP_LOGI(TAG,"Read length from gauge is %d, wanted length is %d",len,length);
        return;
    }
    ERROR_CHECK(TAG,i2c_master_read(packet,data_out,length,I2C_MASTER_ACK));
    for (uint8_t i = 0; i < len - length - 3; i++){
        ERROR_CHECK(TAG, i2c_master_read_byte(packet, &dummy, I2C_MASTER_ACK));

    }  //read the remaining data
    //last byte should be read with NACK
    ERROR_CHECK(TAG, i2c_master_read_byte(packet, &dummy, I2C_MASTER_NACK));
    ERROR_CHECK(TAG, i2c_master_stop(packet));

    ERROR_CHECK(TAG, i2c_master_cmd_begin(PM_I2C_NUM, packet, 10000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(packet);
}

static uint16_t gauge_readWord(uint8_t command) {
    uint16_t data_out;

    i2c_cmd_handle_t packet = i2c_cmd_link_create();
    ERROR_CHECK(TAG, i2c_master_start(packet));
    ERROR_CHECK(TAG, i2c_master_write_byte(packet, (BQ4050_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true));
    ERROR_CHECK(TAG, i2c_master_write_byte(packet, command, true));

    ERROR_CHECK(TAG, i2c_master_start(packet));
    ERROR_CHECK(TAG, i2c_master_write_byte(packet, (BQ4050_SENSOR_ADDR << 1) | I2C_MASTER_READ, true));
    ERROR_CHECK(TAG, i2c_master_read_byte(packet, ((uint8_t *) &data_out + 0), I2C_MASTER_ACK));
    ERROR_CHECK(TAG, i2c_master_read_byte(packet, ((uint8_t *) &data_out + 1), I2C_MASTER_NACK));
    ERROR_CHECK(TAG, i2c_master_stop(packet));

    ERROR_CHECK(TAG, i2c_master_cmd_begin(PM_I2C_NUM, packet, 10000 / portTICK_RATE_MS));
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
    gauge_flashRead(Flash_Cycle_Count,(uint8_t*)&gaugeInfo->cycleCount,2);
//    ESP_LOGI(TAG,"Data Flash Cycle count %d",gaugeInfo->cycleCount);
}
void gauge_read_life_time_data(life_time_data_t* life_time_data){
    uint8_t *data_ptr = (uint8_t*)life_time_data;
    ESP_LOGI(TAG,"lifeTimeData size is %d",sizeof(life_time_data_t));
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
    ESP_LOGI(TAG, "pfStatus = 0x%0X", gaugeInfo->pfStatus);
    ESP_LOGI(TAG, "gaugingStatus = 0x%0X", gaugeInfo->gaugingStatus);
    ESP_LOGI(TAG, "mfgStatus = 0x%0X", gaugeInfo->mfgStatus);
    ESP_LOGI(TAG,"CycleCount = 0x%0x",gaugeInfo->cycleCount);
    ESP_LOGI(TAG,"Size = %d",sizeof(GaugeInfo_t));
    ESP_LOGI(TAG, "----------------------------------------");
}

void gauge_deviceReset() {
//    gauge_MACWrite(MAC_DEVICERESET, NULL, 0);
    GaugeWriteCommand_t writeCommand = {.address = MAC_DEVICERESET, .U.data_ptr = NULL, .size = 0, .successfully_written = 0};
    ESP_LOGI(TAG, "Starting Gauge Reset");
    gauge_writeCommands(&writeCommand, 1, 1);
    vTaskDelay(1000 / portTICK_RATE_MS);
}

void gauge_disable() {
    ESP_LOGI(TAG, "Starting Gauge Disable");
//    uint32_t data = 0x0005;
//    gauge_MACWrite(Flash_MfgInit, (uint8_t *) &data, 2); //disable gauge
    GaugeWriteCommand_t writeCommand = {.address = Flash_MfgInit, .U.data = 0x0000, .size = 2, .successfully_written = 0}; //disable gauge
    ESP_ERROR_CHECK(gauge_writeCommands(&writeCommand,1,false));
//    data = 0x05;
//    gauge_MACRead(Flash_MfgInit, (uint8_t *) &data);
//    ESP_LOGI(TAG, "MFG_Init Register data is %d", data);
}

void gauge_init() {
    GaugeWriteCommand_t writeCommands[] = {
            {.U.data = 0x01, .address = Flash_DAconfig, .size = 1, .successfully_written = 0},   //Enable 2 cells
            {.U.data = 0x0038, .address = Flash_MfgInit, .size = 2, .successfully_written = 0},   //FET_EN = 1, GUAGE_EN = 1
            {.U.data = 0xFF, .address = Flash_ProtectionA, .size = 1, .successfully_written = 0},   //Enabled Protections A
            {.U.data = 0x3F, .address = Flash_ProtectionB, .size = 1, .successfully_written = 0},   //Enabled Protections B
            {.U.data = 0xD5, .address = Flash_ProtectionC, .size = 1, .successfully_written = 0},   //Enabled Protections C
            {.U.data = 0x0F, .address = Flash_ProtectionD, .size = 1, .successfully_written = 0},   //Enabled Protections D
            {.U.data = 0x0320, .address = Flash_LowTempCurrentLow, .size = 2, .successfully_written = 0},   //LowTempCurrentLow = 800 mA
            {.U.data = 0x0320, .address = Flash_LowTempCurrentMed, .size = 2, .successfully_written = 0},   //LowTempCurrentMed = 800 mA
            {.U.data = 0x0320, .address = Flash_LowTempCurrentHigh, .size = 2, .successfully_written = 0},   //LowTempCurrentHigh = 800 mA
            {.U.data = 0x06, .address = Flash_TempEn, .size = 1, .successfully_written = 0},   //Enable temperature
            {.U.data = 0x09F6, .address = Flash_CUVThreshold, .size = 2, .successfully_written = 0},   //CUVThreshold = 2550 mV
            {.U.data = 0x0258, .address = Flash_OCThreshold, .size = 2, .successfully_written = 0},   //OCThreshold = 600 mAh
            {.U.data = 0xC0, .address = Flash_PreChargeCurrent, .size = 1, .successfully_written = 0},   //PreChargeCurrent = 192 mA
            {.U.data = 0x0B54, .address = Flash_EDV0add, .size = 2, .successfully_written = 0},   //EDV0 = 2900 mv
            {.U.data = 0x0BB8, .address = Flash_EDV1add, .size = 2, .successfully_written = 0},   //EDV1 = 3000 mv
            {.U.data = 0x0BD7, .address = Flash_EDV2add, .size = 2, .successfully_written = 0},   //EDV2 = 3031 mv
            {.U.data = 0x1102, .address = Flash_CEDVGaugeConfig, .size = 2, .successfully_written = 0},   //CEDV Gauging Configuration : SME0 = 1, FCC_LIMIT = 1
            {.U.data = 0xD, .address = Flash_CEDVsmoothConfig, .size = 1, .successfully_written = 0},   //CEDVsmoothConfig : SMEN = 1, SMEXT = 1, SMOOTHEOC_EN = 1
            {.U.data = 0x41, .address = Flash_FET_options, .size = 1, .successfully_written = 0},   //FET Options register
    };
    uint8_t num_operations = sizeof(writeCommands) / sizeof(writeCommands[0]);
    ESP_LOGI(TAG, "Starting Gauge Init");
    ESP_ERROR_CHECK(gauge_writeCommands(writeCommands, num_operations, 0));
//    uint8_t unable_to_write_ctr = 0;
//    do {
//        unable_to_write_ctr = 0;
//        for (int i = 0; i < num_operations; ++i) {
//            if (!writeCommands[i].successfully_written) {
//                writeCommands[i].successfully_written = (ESP_OK == gauge_MACWrite(writeCommands[i].address,(uint8_t *) &writeCommands[i].data,writeCommands[i].size));
//                unable_to_write_ctr += !writeCommands[i].successfully_written;
//            }
//        }
//        ESP_LOGI(TAG,"Remaining Items to write for gauge_init is %d",unable_to_write_ctr);
//    } while (unable_to_write_ctr != 0);

//    uint16_t data = 0x01;
//    gauge_MACWrite(Flash_DAconfig, (uint8_t *) &data, 1); //Enable 2 cells
//
//    data = 0x0018;
//    gauge_MACWrite(Flash_MfgInit, (uint8_t *) &data, 2); //FET_EN = 1, GUAGE_EN = 1
//
//    data = 0xFF;
//    gauge_MACWrite(Flash_ProtectionA, (uint8_t *) &data, 1); //Enabled Protections A
//
//    data = 0x3F;
//    gauge_MACWrite(Flash_ProtectionB, (uint8_t *) &data, 1); //Enabled Protections B
//
//    data = 0xD5;
//    gauge_MACWrite(Flash_ProtectionC, (uint8_t *) &data, 1); //Enabled Protections C
//
//    data = 0x0F;
//    gauge_MACWrite(Flash_ProtectionD, (uint8_t *) &data, 1); //Enabled Protections D
//
//    data = 0x06;
//    gauge_MACWrite(Flash_TempEn, (uint8_t *) &data, 1); //Enable temperature
//
//    data = 0x09F6;
//    gauge_MACWrite(Flash_CUVThreshold, (uint8_t *) &data, 2); //CUVThreshold = 2550 mV
//
//    data = 0xC0;
//    gauge_MACWrite(Flash_PreChargeCurrent, (uint8_t *) &data, 1); //PreChargeCurrent = 192 mA
//
//    data = 0x1100;
//    gauge_MACWrite(Flash_CEDVGaugeConfig, (uint8_t *) &data,
//                   2);    //CEDV Gauging Configuration : SME0 = 1, FCC_LIMIT = 1
//
//    data = 0xD;
//    gauge_MACWrite(Flash_CEDVsmoothConfig, (uint8_t *) &data,
//                   1);  //CEDVsmoothConfig : SMEN = 1, SMEXT = 1, SMOOTHEOC_EN = 1
//    data = 0x41;
//    gauge_MACWrite(Flash_FET_options, (uint8_t *) &data, 1); //FET Options register
}

void gauge_enable(){
    GaugeWriteCommand_t enable_command = {.U.data = 0x0018, .address = Flash_MfgInit, .size = 2, .successfully_written = 0};   //FET_EN = 1, GUAGE_EN = 1
    ESP_ERROR_CHECK(gauge_writeCommands(&enable_command,1,false));
}

void gauge_writeGoldenFile(uint8_t *goldenFile) {
    /**
     * According to the datasheet we can write to the flash in 32 byte blocks, starting from address 0x4000
     * That means we need 8192/32 = 256 write commands, instead of allocating 256 * 8 bytes we can use the memory just next to the golden file
     * since it's 64k and we need only the first 8k, so there is enough memory
     */
//    bool is_sector_written[NUM_FLASH_WRITES] = {0};
//    uint8_t unable_to_write_ctr = 0;
    ESP_LOGI(TAG, "Starting to write Golden File");
    GaugeWriteCommand_t *writeCommands = (GaugeWriteCommand_t *) &goldenFile[GOLDEN_FILE_SIZE];
    for (int i = 0; i < NUM_FLASH_WRITES; ++i) {
        uint16_t offset = i * FLASH_BLOCK_SIZE;
        writeCommands[i].address = Flash_Data_Access + offset;
        writeCommands[i].U.data_ptr = &goldenFile[offset];
        writeCommands[i].successfully_written = 0;
        writeCommands[i].size = FLASH_BLOCK_SIZE;
        ESP_LOGI(TAG, "Perparing Golden file commands, address %x, first byte %x, size %d", writeCommands[i].address,
                 writeCommands[i].U.data_ptr[0], writeCommands[i].size);
    }
    ESP_ERROR_CHECK(gauge_writeCommands(writeCommands, NUM_FLASH_WRITES, 1));
//
//    do {
//        unable_to_write_ctr = 0;
//        for (int i = 0; i < NUM_FLASH_WRITES; ++i) {
//            int offset = i * FLASH_BLOCK_SIZE;
//
//            if (!is_sector_written[i]) {
//                is_sector_written[i] = (ESP_OK == gauge_MACWrite(Flash_Data_Access + offset, &goldenFile[offset],
//                                                                 FLASH_BLOCK_SIZE));
//                unable_to_write_ctr += is_sector_written[i] == 0 ? 1 : 0;
//            }
//        }
//        ESP_LOGI(TAG, "Golden File write Ctr for this iteration is %d", unable_to_write_ctr);
//
//    } while (unable_to_write_ctr != 0);
}