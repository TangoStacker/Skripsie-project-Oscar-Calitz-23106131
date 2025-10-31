#ifndef HealthConfig_H
#define HealthConfig_H
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/mesh/health_srv.h>
#include <zephyr/bluetooth/mesh/cfg_srv.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <bluetooth/mesh/models.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/mesh/proxy.h>
#include <zephyr/bluetooth/mesh/access.h>


//op codes for on/off client (from bluetooth assigned numbers)
#define ONOFF_GET	      BT_MESH_MODEL_OP_2(0x82, 0x01)
#define ONOFF_SET	      BT_MESH_MODEL_OP_2(0x82, 0x02)
#define ONOFF_SET_UNACK  BT_MESH_MODEL_OP_2(0x82, 0x03)
#define ONOFF_STATUS     BT_MESH_MODEL_OP_2(0x82, 0x04)


extern uint8_t onoff_tid;
extern uint8_t onoff[];

int initialise_mesh(void);
void genericOnOffSetUnAck(uint8_t on_or_off);
void genericOnOffSet(uint8_t on_or_off);
#endif