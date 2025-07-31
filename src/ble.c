#include "ble.h"


BMP bmp_data;
LSM lsm_data;
STCC4 stcc4_data;
LOGGING logging;
PHYPHOX_EVENT event_data;
//uint8_t OPERATING_MODE = MODE_BTHOME;
uint8_t OPERATING_MODE = MODE_PHYPHOX;

static struct bt_gatt_attr *attr_lsm_acc;
static struct bt_gatt_attr *attr_lsm_gyr;

DATALOGGING LOG;

float global_timestamp = NAN;

static const struct bt_le_adv_param adv_param_normal = {
	.options = BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME | BT_LE_ADV_OPT_FORCE_NAME_IN_AD,
	.interval_min = 3200,
	.interval_max = 3360,
};

static const struct bt_le_conn_param conn_paramter = {
	.interval_min = 12,
	.interval_max = 15,
	.latency = 0,
	.timeout = 10
};

void update_phy(struct bt_conn *conn) {

	int err;
	struct bt_conn_le_phy_param phy_param = {
		.options = BT_CONN_LE_PHY_OPT_NONE,
		.pref_tx_phy = BT_GAP_LE_PHY_2M,
		.pref_rx_phy = BT_GAP_LE_PHY_2M
	};
	err = bt_conn_le_phy_update(conn, &phy_param);
}

static ssize_t read_u16(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	uint8_t *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(value));
}

static ssize_t config_submits(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset,uint8_t flags)
{
	uint8_t *value = attr->user_data;
	if (offset + len > sizeof(config_data)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	memcpy(value + offset, buf, len);
	
	if(attr->uuid == &bmp_cnfg.uuid){
		submit_config_bmp();
	}
	if(attr->uuid == &hdc_cnfg.uuid){
		submit_config_hdc();
	}
	if(attr->uuid == &lsm_cnfg.uuid){
		adjust_lsm_configuration();
	}
	if(attr->uuid == &stcc4_cnfg.uuid){
		submit_config_stcc4();
	}
	if(attr->uuid == &event_uuid.uuid){
		phyphox_event_received();
	}
	return len;
};

BT_GATT_SERVICE_DEFINE(phyphox_gatt, 
	BT_GATT_PRIMARY_SERVICE(&data_service_uuid),
	//BMP384 
	BT_GATT_CHARACTERISTIC(&bmp_uuid,					
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &bmp_data.array[0]),
	BT_GATT_CCC(ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
 	BT_GATT_CHARACTERISTIC(&bmp_cnfg,					
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_WRITE,
			       NULL, config_submits, &bmp_data.config[0]),
	BT_GATT_CCC(ccc_cfg_changed,	//notification handler
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
//lsm323 - GYROSCOPE
	BT_GATT_CHARACTERISTIC(&lsm_gyr_uuid,					
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &lsm_data.gyr_array[0]),
	BT_GATT_CCC(ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
BT_GATT_CHARACTERISTIC(&lsm_acc_uuid,					
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &lsm_data.acc_array[0]),
	BT_GATT_CCC(ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),		
    BT_GATT_CHARACTERISTIC(&lsm_cnfg,					
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_WRITE,
			       NULL, config_submits, &lsm_data.config[0]),
				   
	BT_GATT_CCC(ccc_cfg_changed,	//notification handler
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),	
	//HDC
	BT_GATT_CHARACTERISTIC(&hdc_uuid,					
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &hdc_data.array[0]),
	BT_GATT_CCC(ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&hdc_cnfg,					
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_WRITE,
			       NULL, config_submits, &hdc_data.config[0]),
	BT_GATT_CCC(ccc_cfg_changed,	//notification handler
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	//STCC4
	BT_GATT_CHARACTERISTIC(&stcc4_uuid,					
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &stcc4_data.array[0]),
	BT_GATT_CCC(ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&stcc4_cnfg,					
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_WRITE,
			       NULL, config_submits, &stcc4_data.config[0]),
	BT_GATT_CCC(ccc_cfg_changed,	//notification handler
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	//EVENT SERVICE			
	BT_GATT_PRIMARY_SERVICE(&event_service_uuid),
	BT_GATT_CHARACTERISTIC(&event_uuid,					
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_WRITE,
			       NULL, config_submits, &event_data.config[0]),
	BT_GATT_CCC(ccc_cfg_changed,	//notification handler
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)				
);

#define SERVICE_UUID            0xfcd2
static uint8_t service_data[] = { 
	BT_UUID_16_ENCODE(SERVICE_UUID),
	0x40,
	0x02,	// Temperature
	0x00,	// Low byte
	0x00,   // High byte
	0x03,	// Humidity
	0xbf,	// 50.55%
	0x13,
};

// PRIMARY ADVERTISING DATA
static const struct bt_data ad_bthome[] = {
	/*
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)), //battery service
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, 
		BT_UUID_128_ENCODE(0xcddf1001, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a))
	*/
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
	//BT_DATA_BYTES(BT_DATA_UUID128_ALL,BT_UUID_128_ENCODE(0xcddf1001, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a)),
	BT_DATA(BT_DATA_SVC_DATA16, service_data, ARRAY_SIZE(service_data))
};

static const struct bt_data ad[] = {
	/*
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)), //battery service
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, 
		BT_UUID_128_ENCODE(0xcddf1001, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a))
	*/
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

// SCAN RESPONSE DATA
static const struct bt_data sd[] = {
BT_DATA_BYTES(BT_DATA_UUID128_ALL,
		      0x84, 0xaa, 0x60, 0x74, 0x52, 0x8a, 0x8b, 0x86,
		      0xd3, 0x4c, 0xb7, 0x1d, 0x1d, 0xdc, 0x53, 0x8d),
//BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, sizeof(DEVICE_NAME) - 1),
};
extern void update_advertising(uint8_t t_low, uint8_t t_high, uint8_t h_low, uint8_t h_high){
	service_data[4] = t_low;
	service_data[5] = t_high;
	service_data[7] = h_low;
	service_data[8] = h_high;
	bt_le_adv_update_data(ad_bthome, ARRAY_SIZE(ad_bthome), sd, ARRAY_SIZE(sd));
}
extern void basic_advertising(){
	bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
}
static void bt_ready(void)
{
	int err;
	BLE_PARAMETER_UPDATED = false;
	printk("Bluetooth initialized\n\r");

	uint16_t serialNumber[1];
	memcpy(&serialNumber[0], (uint8_t *)0x10001080, 2);

	if(serialNumber[0]==0x00 || serialNumber[0]==0xffff){
		serialNumber[0]=0;
	}
	printk("number: %i \r\n",serialNumber[0]);

	char name[20];
	sprintf(name, "phyfob %d\n", serialNumber[0]);	
	bt_set_name(name);


	attr_lsm_acc = bt_gatt_find_by_uuid(NULL,0,&lsm_acc_uuid);
	attr_lsm_gyr = bt_gatt_find_by_uuid(NULL,0,&lsm_gyr_uuid);
	err = bt_le_adv_start(&adv_param_normal, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err) {
		printk("Advertising failed to start (err %d)\n\r", err);
		return;
	}


	printk("Advertising successfully started\n\r");
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	OPERATING_MODE = MODE_PHYPHOX;
	basic_advertising();
	printk("Device with index %i trying to connect...\n\r",bt_conn_index(conn));
	
	bt_conn_le_param_update(conn,&conn_paramter);
	update_phy(conn);
	if (err) {
		printk("Connection failed (err 0x%02x)\n\r", err);
	} else {
		printk("Connected\n\r");
	}
}

void enter_logging(){
	//enable hdc, stcc4
	logging.enable = true;
	hdc_logging(true);
	stcc4_logging(true);

}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n\r", reason);
	if(bmp_data.logging==false){
		bmp_data.live = false;
		sleep_bmp(true);
	}else{
		bmp_data.live = false;
		bmp_loggingmode();
	}
	BLE_PARAMETER_UPDATED = false;
	sleep_hdc(true);
	enable_lsm(false);
	sleep_stcc4(true);

	//OPERATING_MODE = MODE_BTHOME;
	OPERATING_MODE = MODE_PHYPHOX;
	bthome_mode();
}
static void le_param_updated(struct bt_conn *conn, uint16_t interval,
			     uint16_t latency, uint16_t timeout){
	printk("Connection parameters updated.\n\r"
	       " interval: %d, latency: %d, timeout: %d\n\r",
	       interval, latency, timeout);
		   BLE_PARAMETER_UPDATED = true;
}
static void le_phy_updated(struct bt_conn *conn,struct bt_conn_le_phy_info *param){
	if(DEBUG){
		printk("LE PHY updated\n\r");
	}
}
static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param){
	if(DEBUG){
		printk("Connection parameters update request received.\n\r");
		printk("Minimum interval: %d, Maximum interval: %d\n\r",
	       param->interval_min, param->interval_max);
		printk("Latency: %d, Timeout: %d\n\r", param->latency, param->timeout);
	}	
	return true;
}
static void le_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	if(DEBUG){
		printk("Data length updated: %s max tx %u (%u us) max rx %u (%u us)\n\r",
           addr, info->tx_max_len, info->tx_max_time, info->rx_max_len,
           info->rx_max_time);
		   }
}
static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_req = le_param_req,
	.le_param_updated = le_param_updated,
	.le_phy_updated = le_phy_updated,
	.le_data_len_updated = le_data_len_updated,
};

void init_ble(){
	bt_enable(NULL);
	bt_ready();
	bt_conn_cb_register(&conn_callbacks);
	memset(LOG.data,NULL,176*LOG_MULTIPLIER);
	LOG.write_to_position = 0;
	LOG.average_sum = 0;
	LOG.average_n= 0;
	LOG.last_save =0;
};

extern void set_coincell_level(uint8_t val){
	bt_bas_set_battery_level(val);
}

extern void send_data(uint8_t ID, float* DATA,uint8_t LEN){

	if(ID == SENSOR_BMP581_ID){
		bt_gatt_notify_uuid(NULL, &bmp_uuid.uuid,&phyphox_gatt.attrs[0],DATA,LEN);
		return;
	}
	if(ID == SENSOR_LSM6DSR_ACC_ID){
		bt_gatt_notify(NULL,attr_lsm_acc,DATA,LEN);
		return;
	}
	if(ID == SENSOR_LSM6DSR_GYR_ID){
		bt_gatt_notify(NULL,attr_lsm_gyr,DATA,LEN);
		return;
	}
	if(ID == SENSOR_HDC_ID){
		bt_gatt_notify_uuid(NULL, &hdc_uuid.uuid,&phyphox_gatt.attrs[0],DATA,LEN);
		return;
	}
	if(ID == SENSOR_STCC4_ID){
		bt_gatt_notify_uuid(NULL, &stcc4_uuid.uuid,&phyphox_gatt.attrs[0],DATA,LEN);
		return;
	}
};