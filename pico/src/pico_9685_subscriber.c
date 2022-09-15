#include <stdio.h>
#include "pico/stdlib.h"
#include "../sdk/pico-sdk/src/rp2_common/hardware_i2c/include/hardware/i2c.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/multi_array_dimension.h>
#include <std_msgs/msg/multi_array_layout.h>

#include <rmw_microros/rmw_microros.h>
#include "pico_uart_transports.h"


const uint LED_PIN = 25;

rcl_subscription_t subscriber;
std_msgs__msg__Int16MultiArray msg;


bool direction = true;
float Millis = 1500;
int servoPin = 0;
float clockDiv = 64;
float wrap = 39062;
// void setMillis(int servoPin, float millis);
// void setServo(int servoPin, float startMillis);
void PCA9685_begin(void);
void PCA9685_writeRegister(uint8_t reg, uint8_t value);
void PCA9685_setPWM(uint8_t channel, int16_t PWM);
void PCA9685_setFrequency(uint16_t Frequency);
uint8_t PCA9685_readRegister(uint8_t reg);

// i2c_inst_t *i2c; 
uint _SDA=12; 
uint _SCL=13; 
static int _i2c_address=0x40;
uint8_t SERVO=0;

void subscription_callback(const void * msgin)
{  
 const std_msgs__msg__Int16MultiArray * msg = (const std_msgs__msg__Int16MultiArray *)msgin;
  gpio_put(LED_PIN, 0);
  sleep_ms(30);
  gpio_put(LED_PIN, 1);
  PCA9685_setPWM(SERVO,(int16_t)(msg->data.data[0]));
  
   
}

//============================================

int main()
{
	static int16_t memory[16]; 
 	msg.data.capacity = 16;
  	msg.data.data = memory;
  	msg.data.size = 16;

    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

	//create init options
    rclc_support_init(&support, 0, NULL, &allocator);

	//create node
    rclc_node_init_default(&node, "pico_subcscriber_to_pca9685_node", "", &support);
    
	// create subscriber
  	rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg,Int16MultiArray),
    "servo_cmd_topic");

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

    gpio_put(LED_PIN, 1);

	PCA9685_begin();

    
    while (true)
    {
		sleep_ms(100);
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}

//==============================================





















// #include "hardware/pwm.h"
// #include "hardware/clocks.h"





// //==================================================

// int main()
// {
//     PCA9685_begin();
//     while (true)
//     {
//         // PCA9685_setPWM(SERVO,(int16_t)(Millis/20000.f)*wrap);
//         // sleep_ms(1000);
//         // Millis = 500; 
//         // PCA9685_setPWM(SERVO,(int16_t)(Millis/20000.f)*wrap);
//         // sleep_ms(1000);
// 		Millis = 100;
// 		PCA9685_setPWM(SERVO,(int16_t)(Millis));
//         sleep_ms(1000);
//         Millis = 500; 
//         PCA9685_setPWM(SERVO,(int16_t)(Millis));
//         sleep_ms(1000);

//     }
// }

// //=========================================================

void PCA9685_begin(void){

    i2c_init(i2c_default, (uint)(100 * 1000));
	gpio_set_function(_SDA, GPIO_FUNC_I2C);
	gpio_set_function(_SCL, GPIO_FUNC_I2C);
	gpio_pull_up(_SDA);
	gpio_pull_up(_SCL);
	
	// configure the PCA9685 for driving servos
	PCA9685_writeRegister(0x00, 0b10100000);
	PCA9685_writeRegister(0x01, 0b00000100);
	PCA9685_setFrequency(50);
	
	// // centre all Servos initially
	// for (uint C = 0; C < 16; C++)
	// {
	// 	setPosition(C, 0);
	// }
	// return;
}

void PCA9685_setFrequency(uint16_t Frequency)
{
	int preScalerVal = (25000000 / (4096 * Frequency)) - 1;
    if (preScalerVal > 255) preScalerVal = 255;
    if (preScalerVal < 3) preScalerVal = 3;

	//need to be in sleep mode to set the pre-scaler
	uint8_t M1 = PCA9685_readRegister(0x00);
	PCA9685_writeRegister(0x00, ((M1 & ~0b10000000) | 0b00010000));
	PCA9685_writeRegister(0xFE, (uint8_t)preScalerVal);

	// restart
	PCA9685_writeRegister(0x00, ((M1 & ~0b00010000) | 0b10000000));
	sleep_us(500);		// <-- sleep functions not ideal, better to have a flag that clears after a period of time (or checks
						// to see if it should be cleared when its value is checked)
	return;
}

uint8_t PCA9685_readRegister(uint8_t reg)
{
	uint8_t D[1];
	
	
	D[0] = reg;
	
	i2c_write_blocking(i2c_default, _i2c_address, D, 1, true);	// write register
	i2c_read_blocking(i2c_default, _i2c_address, D, 1, false);		// read value
	
	return D[0];
}

void PCA9685_writeRegister(uint8_t reg, uint8_t value)
{
	uint8_t D[2];
	
	D[0] = reg;
	D[1] = value;
	
	i2c_write_blocking(i2c_default, _i2c_address, D, 2, false);	// write register then value
	
	return;
}

void PCA9685_setPWM(uint8_t channel, int16_t PWM)
{
	uint8_t D[5];
	
	uint16_t ChannelOffset = channel * 10;		// adds 0-160 to the counter values

	uint16_t ChannelOn = 0 + ChannelOffset;
	uint16_t ChannelOff = PWM + ChannelOffset;
	
	D[0] = 0x06 + (4 * channel);
	D[1] = (0x00FF & ChannelOn);
	D[2] = (0xFF00 & ChannelOn) >> 8;
	D[3] = (0x00FF & ChannelOff);
	D[4] = (0xFF00 & ChannelOff) >> 8;
	
	i2c_write_blocking(i2c_default, _i2c_address, D, 5, false);
	
	return;
}
































