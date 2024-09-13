#include <stdio.h>
#include <stdlib.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/vector3.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "pico/time.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

uint8_t ENC_IN_LEFT_A = 12;
uint8_t ENC_IN_RIGHT_A = 14;
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
uint8_t ENC_IN_LEFT_B = 13;
uint8_t ENC_IN_RIGHT_B = 15;
 
 // Motor control pins 
uint8_t PWM_PIN_LEFT =19 ;
uint8_t PWM_PIN_RIGHT = 21;
uint8_t DIR_LEFT = 18;
uint8_t DIR_RIGHT = 20;


const uint LED_PIN = 0;

rcl_publisher_t publisher;
rcl_publisher_t tick_count;
rcl_publisher_t accel_pub;
rcl_publisher_t left_tick_rate_pub;
rcl_publisher_t right_tick_rate_pub;
rcl_subscription_t left_motor_sub;
rcl_subscription_t right_motor_sub;

std_msgs__msg__Int32 msg;
geometry_msgs__msg__Vector3 tick_data;
std_msgs__msg__Float32 left_tick_rate;
std_msgs__msg__Float32 right_tick_rate;
sensor_msgs__msg__Imu accel_data;

bool Direction_right;
bool Direction_left;

int16_t right_tick_count = 0;
int16_t left_tick_count = 0;
int32_t pwm_l = 0;
int32_t pwm_r = 0;
const int16_t encoder_minimum = -32768;
const int16_t encoder_maximum = 32767;


static int addr = 0x68;


static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(i2c_default, addr, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}


void pwm_init_pin(uint8_t pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.f);
    pwm_init(slice_num, &config, true);
}


void enc_callback(uint gpio,uint32_t events){
    if(gpio == ENC_IN_RIGHT_A) {

        uint val1 = gpio_get(ENC_IN_RIGHT_B);
        if(val1 == 1) {
            Direction_right = false; // Reverse
        }
        else {
            Direction_right = true;
        }

        if (Direction_right) {   
            if (right_tick_count== encoder_maximum) {
            right_tick_count= encoder_minimum;
            }
            else {
            right_tick_count++;  
            }    
        }
        else {
            if (right_tick_count== encoder_minimum) {
            right_tick_count= encoder_maximum;
            }
            else {
            right_tick_count--;  
            }   
        }
    }


    if(gpio == ENC_IN_LEFT_A) {
        uint val = gpio_get(ENC_IN_LEFT_B);
        if(val == 0) {
            Direction_left = false; // Reverse
        }
        else {
            Direction_left = true; // Forward
        }
        
        if (Direction_left) {
            if (left_tick_count== encoder_maximum) {
            left_tick_count= encoder_minimum;
            }
            else {
            left_tick_count++;  
            }  
        }
        else {
            if (left_tick_count== encoder_minimum) {
            left_tick_count= encoder_maximum;
            }
            else {
            left_tick_count--;  
            }   
        }
    }
}

void motor_pwr_left()          // To Control the direction and power of  the left motor
{
  if(pwm_l>0)
  {           
    gpio_put(DIR_LEFT,1);
    gpio_put(LED_PIN,0);
  }
  else if(pwm_l<0)
  {
    gpio_put(DIR_LEFT,0);
    gpio_put(LED_PIN,1);
  }
  else
  {
    gpio_put(DIR_LEFT,0);
  }
  pwm_set_gpio_level(PWM_PIN_LEFT,abs(int(pwm_l)*255));
 }



void motor_pwr_right()          // To Control the direction and power of  the right motor
{
  if(pwm_r>0)
  {           
    gpio_put(DIR_RIGHT,1);
    gpio_put(LED_PIN,0);
  }
  else if(pwm_r<0)
  {
    gpio_put(DIR_RIGHT,0);
    gpio_put(LED_PIN,1);
  }
  else
  {
    gpio_put(DIR_RIGHT,0);
  }
  pwm_set_gpio_level(PWM_PIN_RIGHT,abs(int(pwm_r)*255));
}


void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    // rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    // // msg.data++;
}

void left_subscription_callback(const void * msgin)
{
  // Cast received message to used type
  const std_msgs__msg__Float32 * left_pwm = (const std_msgs__msg__Float32 *)msgin;
  pwm_l = (int32_t)left_pwm->data;
//   gpio_put(LED_PIN,1);

  // Process message
  printf("left_wheel: %d\n", left_pwm->data);
}

void right_subscription_callback(const void * msgin)
{
  // Cast received message to used type
  const std_msgs__msg__Float32 * right_pwm = (const std_msgs__msg__Float32 *)msgin;
  pwm_r = (int32_t)right_pwm->data;

  // Process message
  printf("right_wheel: %d\n", right_pwm->data);
  
}


int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);


    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    mpu6050_reset();

    gpio_init(ENC_IN_LEFT_B);
    gpio_init(ENC_IN_RIGHT_B);
    gpio_set_dir(ENC_IN_LEFT_B,0);
    gpio_set_dir(ENC_IN_RIGHT_B,0);

    gpio_init(ENC_IN_LEFT_A);
    gpio_init(ENC_IN_RIGHT_A);
    gpio_set_dir(ENC_IN_LEFT_A,0);
    gpio_set_dir(ENC_IN_RIGHT_A,0);
  
    // gpio_set_function(PWM_PIN_LEFT,GPIO_FUNC_PWM);
    // gpio_set_function(PWM_PIN_RIGHT,GPIO_FUNC_PWM);
    pwm_init_pin(PWM_PIN_LEFT);
    pwm_init_pin(PWM_PIN_RIGHT);
    gpio_init(DIR_LEFT);
    gpio_init(DIR_RIGHT);

    gpio_set_dir(DIR_LEFT, 1);
    gpio_set_dir(DIR_RIGHT, 1);


    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, 1);
    gpio_set_irq_enabled_with_callback(ENC_IN_LEFT_A, GPIO_IRQ_EDGE_RISE, true, &enc_callback);
    gpio_set_irq_enabled(ENC_IN_RIGHT_A, GPIO_IRQ_EDGE_RISE, true);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 1200;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &left_tick_rate_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "left_tickrate");

    rclc_publisher_init_default(
        &right_tick_rate_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "right_tickrate");

    rclc_publisher_init_default(
        &accel_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data");

    rclc_publisher_init_default(
        &tick_count,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "/tick_count");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(100),
        timer_callback);

    rclc_subscription_init_default(
        &left_motor_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
         "/left_wheel/control_effort");

    rclc_subscription_init_default(
        &right_motor_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/right_wheel/control_effort");

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    // rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(
        &executor, &left_motor_sub, &msg,
        &left_subscription_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(
        &executor, &right_motor_sub, &msg,
        &right_subscription_callback, ON_NEW_DATA);

    gpio_put(LED_PIN, 1);

    msg.data = 0;



    int16_t prev_left_tick_count = 0;
    int16_t prev_right_tick_count = 0;
    float interval = 500.0;
    uint32_t prev_time = to_ms_since_boot(get_absolute_time());
    int16_t acceleration[3], gyro[3], temp;
    rcl_ret_t rc;

    while (true){
        motor_pwr_left();
        motor_pwr_right();
        mpu6050_read_raw(acceleration, gyro, &temp);
        accel_data.linear_acceleration.x = (double)acceleration[0];
        accel_data.linear_acceleration.y = (double)acceleration[1];
        accel_data.linear_acceleration.z = (double)acceleration[2];

        accel_data.angular_velocity.x = (double)gyro[0];
        accel_data.angular_velocity.y = (double)gyro[1];
        accel_data.angular_velocity.z = (double)gyro[2];

        uint32_t curr_time = to_ms_since_boot(get_absolute_time());
        
        if((curr_time - prev_time)>interval){

            left_tick_rate.data = ((float)prev_left_tick_count - (float)left_tick_count)/interval*1000;
            right_tick_rate.data = ((float)prev_right_tick_count - (float)right_tick_count)/interval*1000;

            rc = rcl_publish(&left_tick_rate_pub,&left_tick_rate,NULL);
            rc = rcl_publish(&right_tick_rate_pub,&right_tick_rate,NULL);
            rc = rcl_publish(&accel_pub,&accel_data,NULL);
            
            prev_left_tick_count = left_tick_count;
            prev_right_tick_count = right_tick_count;
            prev_time = curr_time;

        }
        tick_data.x = left_tick_count;
        tick_data.y = right_tick_count; 
        rc = rcl_publish(&tick_count,&tick_data,NULL);
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
