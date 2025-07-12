#include "lvgl/lvgl.h"
#include "fdcan.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "lv_conf.h"
#include <stdbool.h>

#include "our_logo_screenshot.h"

#define LOGO_TIME 350 //time to show logo before switching to pre-drive state

#define LVGS_GREEN "009632"
#define LVGS_YELLOW "c8c800"
#define LVGS_RED "b40000"
#define LVGS_BLACK "000000"

static uint32_t time_count = 0;

//persistent lv_objs for logo state
lv_obj_t* our_logo;

//persistent lv_objs for pre_drive state
lv_obj_t* pre_drive_grid;
lv_obj_t* pre_drive_labels[4];

//persistent lv_objs for drive state

//persistent lv_objs for diagnostic state
lv_obj_t* diagnostic_label;

typedef enum {
	LOGO = 0,
	PRE_DRIVE = 1,
	DRIVE = 2,
	DIAGNOSTIC = 3
} display_state_t;

typedef enum {
	STATUSWORD_NOTREADY	= 0x01,
	STATUSWORD_SHUTDOWN = 0x02,
	STATUSWORD_PRECHARGE = 0x04,
	STATUSWORD_ENERGISED = 0x07,
	STATUSWORD_ENABLED = 0x08,
	STATUSWORD_FAULTREACTION = 0x0B,
	STATUSWORD_FAULTOFF = 0x0D
} statusword_t;

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


typedef struct
{
	float output_torque;
	int16_t motor_speed;
	int16_t battery_current;
	float available_forward_torque;
	float available_reverse_torque;
	statusword_t statusword;
	float capacitor_voltage;
	bool active;
}inv_t;

typedef struct
{
	uint16_t pack_dcl;
	uint8_t temperature;
	float pack_voltage;
	uint8_t pack_soc;
	bool active;
}battery_t;

typedef struct
{
	float lv_voltage;
	uint16_t max_rpm;
	uint8_t current_limit;
	uint16_t max_torque;
	bool rtd;
	bool rtd_switch_state;
	uint8_t fault;
	bool active;
	uint32_t last_comm_time;
}vcu_t;

inv_t inv1 = {};
inv_t inv2 = {};

battery_t battery = {};

vcu_t vcu = {};

static display_state_t current_display_state = LOGO;
static display_state_t commanded_display_state = LOGO;

void update_inverter(lv_obj_t* label, inv_t* inv);
void update_battery(lv_obj_t* label, battery_t* battery);
void update_vcu(lv_obj_t* label, vcu_t* vcu);

void initialize_display_state(display_state_t display_state);
	void initialize_display_state_logo(void);
	void initialize_display_state_pre_drive(void);
	void initialize_display_state_drive(void);
	void initialize_display_state_diagnostic(void);

void clear_display_state(display_state_t display_state);
	void clear_display_state_logo(void);
	void clear_display_state_pre_drive(void);
	void clear_display_state_drive(void);
	void clear_display_state_diagnostic(void);

void update_display_state(display_state_t display_state);
	void update_display_state_logo(void);
	void update_display_state_pre_drive(void);
	void update_display_state_drive(void);
	void update_display_state_diagnostic(void);

/* Create the dashboard UI structure */
void initialize_dashboard(void)
{
	initialize_display_state(LOGO);
}

static void GuiTask(void *pvParameters)
{
    (void)pvParameters;
    const TickType_t xDelay = pdMS_TO_TICKS(10);

    for(;;) {
        lv_timer_handler();   // or lv_task_handler();

        if(time_count < LOGO_TIME)
        {
        	commanded_display_state = LOGO;
        }
        else if(vcu.fault != 0 || !vcu.active)
        {
        	commanded_display_state = DIAGNOSTIC;
        }
        else{
        	commanded_display_state = vcu.rtd ? DRIVE : PRE_DRIVE;
        }
        static bool init_screen = false;
        if(init_screen)
        {
        	init_screen = false;
        	initialize_display_state(commanded_display_state);
        }
        else if(commanded_display_state != current_display_state)
        {
        	clear_display_state(current_display_state);
        	init_screen = true;
        	current_display_state = commanded_display_state;
        }

        if ((time_count % 75) == 0)
        {            // since loop ticks every 10 ms
			update_display_state(current_display_state);
        }

        time_count++;

		vcu.active = (vcu.last_comm_time - time_count) <= 50;

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

			case 0x19107101:
				//vcu.max_torque;
				break;

			case 0x19117101:
				break;
		}
	// Battery and VCU
	} else {
		switch(rxHeader.Identifier){
			case 0x6B1:
				battery.pack_dcl = ((uint16_t)rxData[0] << 8) | rxData[1];
				battery.temperature = rxData[4];

				break;
			case 0x6B0:
				battery.pack_soc = rxData[4];
				battery.pack_voltage = (((uint16_t)rxData[2] << 8) | rxData[3]) * 0.1f;

				break;

			case 0x7A4:
				vcu.lv_voltage = (rxData[0] + ((uint16_t)rxData[1] << 8)) * 12.58f / 2561;
				vcu.current_limit = rxData[4];

				vcu.rtd_switch_state = (rxData[6] >> 4) & 1;
				vcu.rtd = (rxData[6] >> 3) & 1;
				battery.active = (rxData[6] >> 2) & 1;
				inv1.active = (rxData[6] >> 1) & 1;
				inv2.active = (rxData[6] >> 0) & 1;

				vcu.last_comm_time = time_count;

				vcu.fault = rxData[7];

				break;
		}
	}
}

void initialize_display_state(display_state_t display_state)
{
	switch(display_state){
		case LOGO:
			initialize_display_state_logo();
			break;
		case PRE_DRIVE:
			initialize_display_state_pre_drive();
			break;
		case DRIVE:
			initialize_display_state_drive();
			break;
		case DIAGNOSTIC:
			initialize_display_state_diagnostic();
			break;
	}
}

void clear_display_state(display_state_t display_state)
{
	switch(display_state){
		case LOGO:
			clear_display_state_logo();
			break;
		case PRE_DRIVE:
			clear_display_state_pre_drive();
			break;
		case DRIVE:
			clear_display_state_drive();
			break;
		case DIAGNOSTIC:
			clear_display_state_diagnostic();
			break;
	}
}

void update_display_state(display_state_t display_state)
{
	switch(display_state){
			case LOGO:
				update_display_state_logo();
				break;
			case PRE_DRIVE:
				update_display_state_pre_drive();
				break;
			case DRIVE:
				update_display_state_drive();
				break;
			case DIAGNOSTIC:
				update_display_state_diagnostic();
				break;
		}
}

void initialize_display_state_logo(void)
{
	our_logo = lv_img_create(lv_scr_act());   // Create image object
	lv_img_set_src(our_logo, &our_logo_screenshot);                // Set image source
	lv_img_set_zoom(our_logo, 450);
	lv_obj_align(our_logo, LV_ALIGN_CENTER, 0, 0);      // Align to center (optional)
}

void clear_display_state_logo(void)
{
	lv_obj_del(our_logo);
}

void update_display_state_logo(void){/*do nothing*/}

void initialize_display_state_pre_drive(void)
{
	// Create a container for the grid
	pre_drive_grid = lv_obj_create(lv_scr_act());

	lv_obj_t* grid = pre_drive_grid; // make use of the additional variable so I don't have to change chatgpt's code
	lv_obj_set_size(grid, 800, 420);
	lv_obj_center(grid); // Optional: center the grid on the screen

	// Enable grid layout
	static lv_coord_t col_dsc[] = {400, 400, LV_GRID_TEMPLATE_LAST};
	static lv_coord_t row_dsc[] = {210, 210, LV_GRID_TEMPLATE_LAST};

	lv_obj_set_layout(grid, LV_LAYOUT_GRID);
	lv_obj_set_grid_dsc_array(grid, col_dsc, row_dsc);

	// Helper function to create a label in a grid cell
	for (int row = 0; row < 2; row++) {
	    for (int col = 0; col < 2; col++) {
	        lv_obj_t *label = lv_label_create(grid);
	        lv_label_set_recolor(label,true);
	        pre_drive_labels[2*row + col] = label;

	        // Set label to occupy one cell
	        lv_obj_set_grid_cell(label, LV_GRID_ALIGN_CENTER, col, 1,
	                                       LV_GRID_ALIGN_CENTER, row, 1);
	    }
	}

}

void clear_display_state_pre_drive(void)
{
	lv_obj_del(pre_drive_grid);
}

void update_display_state_pre_drive()
{
	const char* lv_battery_color_state = vcu.lv_voltage >= 12.7 ? LVGS_GREEN : LVGS_YELLOW;
	const char* rtd_color_state = vcu.rtd_switch_state && time_count % 150 >= 75 ? LVGS_RED : LVGS_BLACK;
	const char* rtd_state_string = vcu.rtd_switch_state ? "ON" : "OFF";

	lv_label_set_text_fmt(pre_drive_labels[0],
			"LV Batt: #%s %.1f%V#\n"
			"RTD Switch: #%s %s#",
			lv_battery_color_state,vcu.lv_voltage,rtd_color_state,rtd_state_string);

	lv_label_set_text_fmt(pre_drive_labels[2],
			"HV Battery Pack\n"
			"Temperature: %d C\n"
			"SOC: %d%%\n"
			"Pack Voltage: %.2f"
			"Pack DCL: %d A",
			battery.temperature,battery.pack_voltage,battery.pack_soc,battery.pack_dcl);

	lv_label_set_text_fmt(pre_drive_labels[1],
			"Inverters\n"
			"Inv1 Status: %s\n"
			"Inv1 Cap Voltage: %.2f\n\n"
			"Inv2 Status: %s\n"
			"Inv2 Cap Voltage: %.2f",
			inverter_statusword(inv1.statusword),inv1.capacitor_voltage,inverter_statusword(inv2.statusword),inv2.capacitor_voltage);

	lv_label_set_text_fmt(pre_drive_labels[3],
			"VCU Config\n"
			"Max Torque: %d N*m\n"
			"Max Inverter Current: %d A\n"
			"Max RPM: %d",
			vcu.max_torque,vcu.current_limit,vcu.max_rpm);

}

void initialize_display_state_drive(void)
{

}

void clear_display_state_drive(void)
{

}

void update_display_state_drive()
{

}

void initialize_display_state_diagnostic(void)
{
	diagnostic_label = lv_label_create(lv_scr_act());
	lv_obj_set_size(diagnostic_label,800,420);
	lv_obj_center(diagnostic_label);
}

void clear_display_state_diagnostic(void)
{
	lv_obj_del(diagnostic_label);
}

void update_display_state_diagnostic()
{
	lv_label_set_text_fmt(diagnostic_label,
			"VCU Fault: %d",
			vcu.fault);
}
