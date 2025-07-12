#include "lvgl/lvgl.h"
#include "fdcan.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "lv_conf.h"


lv_obj_t* inv1_labels;
lv_obj_t* inv2_labels;
lv_obj_t* battery_labels;

typedef enum {
	STATUSWORD_NOTREADY	= 0x01,
	STATUSWORD_SHUTDOWN = 0x02,
	STATUSWORD_PRECHARGE = 0x04,
	STATUSWORD_ENERGISED = 0x07,
	STATUSWORD_ENABLED = 0x08,
	STATUSWORD_FAULTREACTION = 0x0B,
	STATUSWORD_FAULTOFF = 0x0D
} statusword_t;


typedef struct
{
	float output_torque;
	int16_t motor_speed;
	int16_t battery_current;
	float available_forward_torque;
	float available_reverse_torque;
	statusword_t statusword;
	float capacitor_voltage;
}inv_t;

inv_t inv1 = {};
inv_t inv2 = {};

static uint16_t b_dlc = 0;      // Battery DLC

void update_inverter(lv_obj_t* label, inv_t* inv);
void update_battery(uint16_t dlc);

/* Create the dashboard UI structure */
void create_dash_board(void)
{
    lv_obj_t * container = lv_obj_create(lv_scr_act());
    lv_obj_set_size(container, 800, 480);
    lv_obj_center(container);
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(container,
        LV_FLEX_ALIGN_SPACE_BETWEEN,   // Main-axis: space-between → first child left, last child right
        LV_FLEX_ALIGN_CENTER,          // Cross-axis: center vertically
        LV_FLEX_ALIGN_CENTER);         // Track-axis (unused for simple rows)
    lv_obj_set_style_pad_gap(container, 0, 0);       // no extra gap needed
    //lv_obj_set_style_pad_row(container, 5, 0); // spacing

    inv1_labels = lv_label_create(container);
    lv_label_set_text(
		inv1_labels,
		"Inverter 1, Status: ---\n"
		"Output Torque:    ---\n"
		"Available Torque:    ---\n"
		"Available Reverse Torque:    ---\n"
		"Motor Speed:    ---\n"
		"Battery Current:    ---\n"
		"Capacitor Voltage:    ---"
	);
    lv_obj_set_style_text_font(inv1_labels, &lv_font_montserrat_30, 0);

    inv2_labels = lv_label_create(container);
    lv_label_set_text(
    		inv2_labels,
    		"Inverter 1, Status: ---\n"
    		"Output Torque:    ---\n"
    		"Available Torque:    ---\n"
    		"Available Reverse Torque:    ---\n"
    		"Motor Speed:    ---\n"
    		"Battery Current:    ---\n"
    		"Capacitor Voltage:    ---"
    	);
    lv_obj_set_style_text_font(inv2_labels, &lv_font_montserrat_30, 0);



    //battery_labels = lv_label_create(container);
    //lv_label_set_text(battery_labels, "Battery DLC: ---");
    //lv_obj_set_style_text_font(battery_labels, &lv_font_montserrat_30, 0);

}

static void GuiTask(void *pvParameters)
{
    (void)pvParameters;
    const TickType_t xDelay = pdMS_TO_TICKS(10);

    uint32_t counter = 0;

    for(;;) {
        lv_timer_handler();   // or lv_task_handler();

        if ((counter % 100) == 0)
        {            // since loop ticks every 10 ms
			//lv_label_set_text_fmt(inv1_labels, "%lu", counter/100);
        	update_inverter(inv1_labels, &inv1);
        	update_inverter(inv2_labels, &inv2);

        	//update_battery(b_dlc);
        }
		counter++;

        osDelay(xDelay);
    }
}

void CreateGuiTask(void)
{

    osThreadNew(GuiTask, NULL, &(osThreadAttr_t){
        .name = "gui",
        .priority = osPriorityNormal,
        .stack_size = 1024
    });

}

const char* inverter_statusword(statusword_t word)
{
	switch(word)
	{
					case STATUSWORD_NOTREADY: return "NOT READY";
					case STATUSWORD_SHUTDOWN: return "SHUTDOWN";
					case STATUSWORD_PRECHARGE: return "PRECHARGE";
					case STATUSWORD_ENERGISED: return "ENERGISED";
					case STATUSWORD_ENABLED: return "ENABLED";
					case STATUSWORD_FAULTREACTION: return "FAULT REACTION";
					case STATUSWORD_FAULTOFF: return "FAULT OFF";
					default: return "";
	}
}

void update_inverter(lv_obj_t* label, inv_t* inv)
{
	lv_label_set_text_fmt(label,
	        "Status: %s\n"
	        "Output Torque: %.2f\n"
	        "Av For Torque: %.2f\n"
	        "Av Rev Torque: %.2f\n"
	        "Motor Speed: %i\n"
	        "Bat Curr: %i\n"
	        "Cap Voltage: %.2f",
			inverter_statusword(inv->statusword), inv->output_torque, inv->available_forward_torque, inv->available_reverse_torque, inv->motor_speed, inv->battery_current, inv->capacitor_voltage);
}


void update_battery(uint16_t dlc)
{
	lv_label_set_text_fmt(battery_labels, "Battery DLC: %d", dlc);
}

void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

/* Receive new CAN messages */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	FDCAN_RxHeaderTypeDef rxHeader;
	uint8_t rxData[8];

	// Check if new message is in FIFO0
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0){
		return;
	}

	// Get Message
	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData);

	// data fields that we want to receive

	// Inverters
	if (rxHeader.IdType == FDCAN_EXTENDED_ID){
		switch(rxHeader.Identifier & 0x01FFFFFF){
			case 0x118ff71:
				// Inverter 1 HS1
				// Output Torque, Motor Speed, Battery Current (16 bit each)
			    inv1.output_torque = ((rxData[0]) | (rxData[1] << 8)) / 160;
			    inv1.motor_speed = (rxData[2]) | (rxData[3] << 8);
			    inv1.battery_current = (rxData[4]) | (rxData[5] << 8);

			    break;
			case 0x118ff72:
				// Inverter 2 HS1
				// Output Torque, Motor Speed, Battery Current (16 bit each)
			    inv2.output_torque = ((rxData[0]) | (rxData[1] << 8)) / 160;
			    inv2.motor_speed = (rxData[2]) | (rxData[3] << 8);
			    inv2.battery_current = (rxData[4]) | (rxData[5] << 8);

			    break;
			case 0x119ff71:
				// Inverter 1 HS2
				// Available Torque, Available Reverse Torque, Status Word
				inv1.available_forward_torque = ((rxData[0]) | (rxData[1] << 8))/160;
				inv1.available_reverse_torque = ((rxData[2]) | (rxData[3] << 8))/160;
				inv1.statusword = rxData[4];

				break;
			case 0x119ff72:
				// Inverter 2 HS2
				// Available Torque, Available Reverse Torque, Status Word
				inv2.available_forward_torque = ((rxData[0]) | (rxData[1] << 8))/160;
				inv2.available_reverse_torque = ((rxData[2]) | (rxData[3] << 8))/160;
				inv2.statusword = rxData[4];

				break;
			
			case 0x11aff71:
			    // Inverter 1 HS3
				// Measured capacitor voltage (measured battery voltage)
				inv1.capacitor_voltage = ((rxData[4]) | (rxData[5] << 8))/16;

				break;
			case 0x11aff72:
				// Inverter 2 HS3
				// Measured capacitor voltage (measured battery voltage)
				inv2.capacitor_voltage = ((rxData[4]) | (rxData[5] << 8))/16;

				break;
		}
	// Battery
	} else {
		if (rxHeader.Identifier == 0x6B1) {
			// battery DLC
			b_dlc = (rxData[0] << 8) | rxData[1];

		}
	}

}



