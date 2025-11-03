
#include "gpio.h" // Needed for set_board_led
#include "mesh.h"
#include "sensors.h"

bool publish = false;
uint16_t reply_addr;
uint8_t reply_net_idx;
uint8_t reply_app_idx;
static struct bt_mesh_model models[];
uint8_t onoff_tid = 0;
uint8_t onoff[] = {
	0,
	1};

//------------------------------------------------------------------------
//health server led control (attention)
//-------------------------------------
//attention leds taht will switch led while provisioning
static void attention_on(const struct bt_mesh_model *mod)
{
    printk("attention_on()\n");
    set_board_led(true);
}

static void attention_off(const struct bt_mesh_model *mod)
{
    printk("attention_off()\n");
    set_board_led(false);
}
//health server callback
static const struct bt_mesh_health_srv_cb health_srv_callback = {
    .attn_on = attention_on,
    .attn_off = attention_off,
};
//health server struct (callback is set in here)
static struct bt_mesh_health_srv health_srv = {
    .cb = &health_srv_callback,
};
//------------------------------------------------------------------------------------------------------------------------
//BT sensor mesh sesnor gets
//-------
//temp sensor, places the temp value from hs3003 sensor into bt mesh sensor when get request is sent
static int temp_get(struct bt_mesh_sensor_srv *srv,
                    struct bt_mesh_sensor *sensor,
                    struct bt_mesh_msg_ctx *ctx,
                    struct bt_mesh_sensor_value *rsp)
                    {
						//using environmental sensor format
                        int err;
                        struct sensor_value temperature;
                        /*

                        .val1 --> int32_t interger part of value

                        .val2 --> int32_t fractional part of the value  (in one-millionth parts, 0.5 = 500000, *1 000 000)
						
                        */
                        //take measurement and place values into global variables
						hs3003_measurement();
                        //global variables are double convert them too int32 and put them into sensor_value struct
                        temperature.val1 = (int32_t)hs3003_temperature;
                        double fractional_part = (hs3003_temperature-(double)(temperature.val1))*1000000;
                        temperature.val2 = (int32_t)fractional_part;
                        printk("val1 temp:%d \n val2 temp%d\n",temperature.val1,temperature.val2);

                        err = bt_mesh_sensor_value_from_sensor_value(
                            sensor->type->channels[0].format, &temperature, rsp);
                        if (err && err != -ERANGE) {
                            printk("Error encoding temperature sensor data (%d)\n", err);
                            return err;
                        }
                        return 0;
                    }
static int hum_get(struct bt_mesh_sensor_srv *srv,
                    struct bt_mesh_sensor *sensor,
                    struct bt_mesh_msg_ctx *ctx,
                    struct bt_mesh_sensor_value *rsp)
					{
						//using percentage sensor format
						int err;
                        struct sensor_value humidity;
						hs3003_measurement();
						humidity.val1 = (int32_t)(hs3003_humidity);//to round up and cast
						humidity.val2 = (int32_t)0;
						printk("val1 temp:%d \n val2 temp%d\n",humidity.val1,humidity.val2);
						err = bt_mesh_sensor_value_from_sensor_value(
                            sensor->type->channels[0].format, &humidity, rsp);
                        if (err && err != -ERANGE) {
                            printk("Error encoding humidity sensor data (%d)\n", err);
                            return err;
                        }
                        return 0;
					}
//------------------------------------------------------------------------------------------------------------------------
//BT  sensor mesh
//------- -------
//
//part of temp sensor structure
// static const struct bt_mesh_sensor_descriptor hs3003_temperature_description =
// {
// 	.tolerance = {
// 		.negative = BT_MESH_SENSOR_TOLERANCE_ENCODE(4),
// 		.positive = BT_MESH_SENSOR_TOLERANCE_ENCODE(4),
// 	},
// 	.sampling_type = BT_MESH_SENSOR_SAMPLING_INSTANTANEOUS,//sample is current instantaneous temperature
// };
//temp sensor structure
static struct bt_mesh_sensor hs3003_temp = 
{
.type = &bt_mesh_sensor_present_amb_temp,
.get = temp_get,//in sensors section
// .descriptor = &hs3003_temperature_description,
};
static struct bt_mesh_sensor hs3003_hum =
{
.type = &bt_mesh_sensor_present_amb_rel_humidity,
.get = hum_get,
};
//array of mesh sensors for elemnt 0
//array of sensors and sensor settings(not used here)
static struct bt_mesh_sensor *const ambient_temperature[] =
{
&hs3003_temp,//temperature sensor for mesh server
};
static struct bt_mesh_sensor *const ambient_humidity[] =
{
&hs3003_hum,
};
//sensor mesh initiation
//initialises array of sensors
static struct bt_mesh_sensor_srv ambient_hum_srv = BT_MESH_SENSOR_SRV_INIT(ambient_humidity,ARRAY_SIZE(ambient_humidity));
static struct bt_mesh_sensor_srv ambient_temp_srv = BT_MESH_SENSOR_SRV_INIT(ambient_temperature,ARRAY_SIZE(ambient_temperature));
//define published message for health server as 0, no health messages being published only led state being changed
BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

// -------------------------------------------------------------------------------------------------------
// onoff server and general functions
// -----------
//processes messages
uint8_t onoff_state;
void generic_onoff_status(bool publish, uint8_t on_or_off)
{
	int err;
	struct bt_mesh_model *model = &models[3];
	if (publish && model->pub->addr == BT_MESH_ADDR_UNASSIGNED) 
		{
			// printk("No publish address associated with the generic on off server model -add one with a configuration app like nRF Mesh\n");
			return;
		}
	if (publish) 
		{
		struct net_buf_simple *msg = model->pub->msg;
		net_buf_simple_reset(msg);
		bt_mesh_model_msg_init(msg, ONOFF_STATUS);
		net_buf_simple_add_u8(msg, on_or_off);
		// printk("publishing on off status message\n");
		err = bt_mesh_model_publish(model);
		if (err)
			{
				printk("bt_mesh_model_publish err %d\n", err);
			}
		} 
	else 
	{
		uint8_t buflen = 7;
		NET_BUF_SIMPLE_DEFINE(msg, buflen);
		bt_mesh_model_msg_init(&msg, ONOFF_STATUS);
		net_buf_simple_add_u8(&msg, on_or_off);
		onoff_tid++;
		struct bt_mesh_msg_ctx ctx = {
		.net_idx = reply_net_idx,
		.app_idx = reply_app_idx,
		.addr = reply_addr,
		.send_ttl = BT_MESH_TTL_DEFAULT,
		};
		// printk("sending on off status message\n");
		if (bt_mesh_model_send(model, &ctx, &msg, NULL, NULL))
			{
				printk("Unable to send generic onoff status message\n");
			}
	}
}

static void set_onoff_state(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,struct net_buf_simple *buf, bool ack)
{
	uint8_t msg_onoff_state = net_buf_simple_pull_u8(buf);
	if (msg_onoff_state == onoff_state) 
	{
		// no state change so nothing to do
		return;
	}
	onoff_state = msg_onoff_state;
	uint8_t tid = net_buf_simple_pull_u8(buf);
	// printk("set onoff state: onoff=%u TID=%u\n", onoff_state, tid);
	if (onoff_state == 0)
	{
		set_board_led(false);
	}
	else
	{
		set_board_led(true);
	}
	/*
	* 3.7.7.2 Acknowledged Set
	*/
	if (ack) 
	{
		generic_onoff_status(false, onoff_state);
	}
	/*
	* If a server has a publish address, it is required to publish status on a state
	change
	* See Mesh Profile Specification 3.7.6.1.2
	*/
	if (model->pub->addr != BT_MESH_ADDR_UNASSIGNED) 
		{
			generic_onoff_status(true, onoff_state);
		}
}
BT_MESH_MODEL_PUB_DEFINE(generic_onoff_pub, NULL, 2 + 1);

static void generic_onoff_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,struct net_buf_simple *buf)
{
	// printk("gen_onoff_get\n");
	reply_addr = ctx->addr;
	reply_net_idx = ctx->net_idx;
	reply_app_idx = ctx->app_idx;
	generic_onoff_status(false, onoff_state);
}

static void generic_onoff_set(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,struct net_buf_simple *buf)
{
	// printk("gen_onoff_set\n");
	set_onoff_state(model, ctx, buf, true);
}

static void generic_onoff_set_unack(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	// printk("generic_onoff_set_unack\n");
	set_onoff_state(model, ctx, buf, false);
}

static const struct bt_mesh_model_op generic_onoff_op[] = 
{
		{ONOFF_GET, 0, generic_onoff_get},
		{ONOFF_SET, 2, generic_onoff_set},
		{ONOFF_SET_UNACK, 2, generic_onoff_set_unack},
		BT_MESH_MODEL_OP_END,
};
// -------------------------------------------------------------------------------------------------------
// onoff client functions
// -----------
//sends messges
BT_MESH_MODEL_PUB_DEFINE(gen_onoff_cli, NULL, 2);
static const struct bt_mesh_model_op generic_onoff_cli_op[] =
{
	{ONOFF_STATUS, 1, generic_onoff_status},
	BT_MESH_MODEL_OP_END,
};

int genericOnOffGet()
{
	printk("genericOnOffGet\n");
	int err;
	struct bt_mesh_model *model = &models[4];
	//check if pub address set
	if (model->pub->addr == BT_MESH_ADDR_UNASSIGNED) 
		{
			// printk("No publish address associated with the generic on off client model - add onewith a configuration app like nRF Mesh\n");
			return -1;
		}
	struct net_buf_simple *msg = model->pub->msg;
	bt_mesh_model_msg_init(msg,ONOFF_GET);
	printk("publishing get on off message\n");
	err = bt_mesh_model_publish(model);
	if (err) 
		{
			printk("bt_mesh_model_publish err %d\n", err);
		}
	return err;
}

int sendGenOnOffSet(uint8_t on_or_off, uint16_t message_type)
{
	int err;
	struct bt_mesh_model *model = &models[4];
	if (model->pub->addr == BT_MESH_ADDR_UNASSIGNED) 
		{
			printk("No publish address associated with the generic on off client model - add one with a configuration app like nRF Mesh\n");
			return -1;
		}
	struct net_buf_simple *msg = model->pub->msg;
	bt_mesh_model_msg_init(msg, message_type);
	net_buf_simple_add_u8(msg, on_or_off);
	net_buf_simple_add_u8(msg, onoff_tid);
	onoff_tid++;
	printk("publishing set on off state=0x%02x\n",on_or_off);
	err = bt_mesh_model_publish(model);
	if (err)
		{
			printk("bt_mesh_model_publish err %d\n", err);
		}
	return err;
}
void genericOnOffSetUnAck(uint8_t on_or_off)
{
	if (sendGenOnOffSet(on_or_off, ONOFF_SET_UNACK))
		{
			printk("Unable to send generic onoff set unack message\n");
		} 
	else 
		{
			printk("onoff set unack message %d sent\n",on_or_off);
		}
}
void genericOnOffSet(uint8_t on_or_off)
{
	if (sendGenOnOffSet(on_or_off, ONOFF_SET))
		{
			printk("Unable to send generic onoff set message\n");
		} 
	else 
		{
			printk("onoff set message %d sent\n",on_or_off);
		}
}
// -------------------------------------------------------------------------------------------------------
// Composition
// -----------
static struct bt_mesh_model models[] =
{
	BT_MESH_MODEL_CFG_SRV,//config server
	BT_MESH_MODEL_HEALTH_SRV(&health_srv,&health_pub),//health server
	BT_MESH_MODEL_SENSOR_SRV(&ambient_temp_srv),//sensor server
	// BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, onoff_client_op, &onoff_client_publish, 0),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, generic_onoff_op,&generic_onoff_pub, NULL),//onoff server (led)
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, generic_onoff_cli_op, &gen_onoff_cli,&onoff[0]),//onoff client (switch)
};
static struct bt_mesh_model models2[] =
{
	BT_MESH_MODEL_SENSOR_SRV(&ambient_hum_srv),
};
static struct bt_mesh_elem elements[] = 
{
    //BT_MESH_ELEM(index, array of models, vendor models)
    BT_MESH_ELEM(1, models,BT_MESH_MODEL_NONE),
	BT_MESH_ELEM(2,models2,BT_MESH_MODEL_NONE),
};
//node
static const struct bt_mesh_comp comp = 
{
    .cid = 0xFFFF,//company id use this during testing
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};
// -------------------------------------------------------------------------------------------------------
//provisioning
//------------
//outputs pins generated by stack
static int output_number(bt_mesh_output_action_t action, uint32_t number)
{
	printk("OOB Number: %u\n", number);
	return 0;
}
static void prov_complete(uint16_t net_idx, uint16_t addr)
{
	printk("Provisioning completed!\n");
}
static void prov_reset(void)
{
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}
static uint8_t dev_uuid[16];
//provisioning properties
static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
	.output_size = 4,
	.output_actions = BT_MESH_DISPLAY_NUMBER,
	.output_number = output_number,//function above
	.complete = prov_complete,//function above
	.reset = prov_reset,//function above
};
static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	err = bt_mesh_init(&prov, &comp);
	// err = bt_mesh_init(bt_mesh_dk_prov_init(), model_handler_init());
	if (err) {
		printk("Initializing mesh failed (err %d)\n", err);
		return;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
        printk("settings loaded\n");
	}
	/* This will be a no-op if settings_load() loaded provisioning info */
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);

	printk("Mesh initialized\n");
}
int initialise_mesh(void)
{
	int err = 0;
	if (IS_ENABLED(CONFIG_HWINFO)) {
		err = hwinfo_get_device_id(dev_uuid, sizeof(dev_uuid));
	}

	if (err < 0) {
		dev_uuid[0] = 0xdd;
		dev_uuid[1] = 0xdd;
	}
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}
	

    printk("Health Server initialized.\n");
	return 1;
    // Often, this function is mostly a placeholder and the real initialization 
    // happens when the primary model list is defined and registered.
}
//
//more onoff stuff
//