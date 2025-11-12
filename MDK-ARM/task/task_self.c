#include "task_self.h"
 
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "task.h"
#include "FreeRTOSConfig.h"

#include "inv_mpu.h"
#include "mpu6050.h"
#include "filter.h"

#include "bsp_can.h"

/*
0:6020,id2,0x205+1,set_motor_voltage(0x1FF,0, motor_output[0], 0, 0, &hcan1);
1:3508,id1,0x200+1,set_motor_voltage( 0x200,motor_output[0],0,0,0,&hcan1);
2:3508,id2,0x200+2,set_motor_voltage( 0x200,0,motor_output[0],0,0,&hcan1);
3:3508,id3,0x200+3,set_motor_voltage( 0x200,0,0,motor_output[0],0,&hcan1);
4:6020,id4,0x205+3,set_motor_voltage(0x1FF,0, 0, 0, motor_output[0], &hcan2);
*/
//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_10);
//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_10);


//全局标志区
bool sbus_rx_flag=false;
uint8_t can_rx_flag=0;
bool mpu_rx_flag=false;
volatile ControlState_t g_robot_control_state = CONTROL_DISABLED;
uint8_t vofa_send_id= 2;
uint8_t pid_speed_mode= 1;    		//1为速度环，0为位置环 

//全局数据区
volatile float led_freq=1;
volatile uint16_t g_led_brightness = 0;
mpu6050_raw mpu_data_global;
moto_info_t motor_info_global[MOTOR_MAX_NUM];
ComplementaryFilter myComplementaryFilter[MOTOR_MAX_NUM];
												 
pid_pos pid_angle_pitch =   {.Kp = 0, .Ki = 0, .Kd = 0,.integral_max=0,
	.output_max = 0,.target=0,.now=0,.last_now=0,.integral=0,.output=0,.last_error=0};

pid_pos pid_speed_pitch =   {.Kp = 0, .Ki = 0, .Kd = 0,.integral_max=0,
	.output_max = 0,.target=0,.now=0,.last_now=0,.integral=0,.output=0,.last_error=0};									

pid_pos pid_angle_yaw =   {.Kp = 0, .Ki = 0, .Kd = 0,.integral_max=0,
	.output_max = 0,.target=0,.now=0,.last_now=0,.integral=0,.output=0,.last_error=0};

pid_pos pid_speed_yaw =   {.Kp = 0, .Ki = 0, .Kd = 0,.integral_max=25000,
	.output_max = 25000,.target=0,.now=0,.last_now=0,.integral=0,.output=0,.last_error=0};	

static int16_t motor_output[5] = {0}; // 用于存储PID计算结果

//任务实现区
void can1_rx(void const * argument){

    /* USER CODE BEGIN can1_rx */
    TickType_t xLastWakeTime = xTaskGetTickCount(); // 获取当前时间
    const TickType_t xFrequency = pdMS_TO_TICKS(1); 

    /* Infinite loop */
    //初始化互补滤波器
    myComplementaryFilter[motor_id_global] .alpha=0.98;
    myComplementaryFilter[motor_id_global].last_omega=0;
    myComplementaryFilter[motor_id_global].tau=0.0f;

    Filter_Init(&myComplementaryFilter[motor_id_global], 20.0f); // 20Hz截止频率

    uint8_t control_flag=0;//用来检测遥控器保护是否稳定,如果不稳定则用vofa观察1中是否会突然出现0
    for(;;)
    {
        // 1. 使用vTaskDelayUntil实现精准的周期性延时
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
		//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_10);
        if(can_rx_flag){
			//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_10);
            memcpy(&motor_info_global,&motor_info,sizeof(moto_info_t)*MOTOR_MAX_NUM);
            motor_info_global[0].motor_speed*=5;
            motor_info_global[4].motor_speed*=5;
			Filter_UpdateMotor(&myComplementaryFilter[0], motor_info_global[0].motor_speed);
            Filter_UpdateMotor(&myComplementaryFilter[4], motor_info_global[4].motor_speed);
            can_rx_flag=false;
			//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_10);
        }
		
        if(mpu_rx_flag){
            //互补滤波器融合陀螺仪和电机速度
			//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_10);
            //Filter_UpdateMPU(&myComplementaryFilter[0], mpu_data_global.gy);
            Filter_UpdateMPU(&myComplementaryFilter[4], mpu_data_global.gz);
            myComplementaryFilter[4].last_omega=mpu_data_global.gz;
            mpu_rx_flag=false;
			//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_10);
        }

        // 调用PID任务，不再需要传递rx_flag
        motor_output[motor_id_global] = 
        pid_speed_task(
            myComplementaryFilter[motor_id_global].last_omega,//将融合后的速度传递给电机信息结构体，方便PID调用
            motor_info_global[motor_id_global].motor_angle,
            &pid_angle_yaw,
            &pid_speed_yaw,
            motor_id_global
        );
        
        control_flag=0;	
        if(g_robot_control_state == CONTROL_ENABLED) {
            switch (motor_id_global)
            {
            case 0:
                set_motor_voltage(0x1FF,0, motor_output[0], 0, 0, &hcan1);
                break;
            case 4:
                set_motor_voltage(0x1FF,0, 0, 0, motor_output[4], &hcan2);
                break;    
            default:
                break;
            }          
            control_flag=1;	
        }   

        debug_send_uart1(2);
		//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_10);
    }
    /* USER CODE END can1_rx */
}

void sbus_receive(void const * argument)
{
    /* USER CODE BEGIN sbus_receive */
	TickType_t xLastWakeTime = xTaskGetTickCount(); // 获取当前时间
    const TickType_t xFrequency = pdMS_TO_TICKS(14); //每70hz检查遥控信号
	//SbusData_t my_sbus_data;
	uint8_t fail_count=0;
    uint8_t sbus_rx_buf_t[SBUS_FRAME_SIZE];
    /* Infinite loop */
    for(;;)
    {
        // 1. 使用vTaskDelayUntil实现精准的周期性延时
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        //HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_10);
        if(sbus_rx_flag){//如果中断收到信息
            memcpy(sbus_rx_buf_t,  sbus_rx_buf,SBUS_FRAME_SIZE);
            Get_DR16_Data(sbus_rx_buf_t);
            
            if(RC_CtrlData.remote.s1 == 3) {
                g_robot_control_state = CONTROL_ENABLED;
                LED_GREEN_ON(); // 激活时常亮，更直观
            } 
            else {
                g_robot_control_state = CONTROL_DISABLED;
                LED_GREEN_OFF(); 
            }

            if(RC_CtrlData.remote.s2 == 1) {
                pid_speed_mode = 1;
            } 
            else {
                pid_speed_mode = 0;
            }

            debug_send_uart1(4);
            sbus_rx_flag=false;
            fail_count=0;
        }
        else{
            fail_count++;
            if(fail_count>=2){
                g_robot_control_state = CONTROL_DISABLED;
                LED_GREEN_OFF();
                fail_count = 0;
            }
        }		

		debug_send_uart1(4);
		//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_10);
    }
    /* USER CODE END sbus_receive */
}
 
void led_breath(void const * argument)
{
  /* USER CODE BEGIN led_breath */
	static uint32_t breath_period_ms = led_timer; 
	uint16_t new_period_from_queue = 0;
	static uint8_t brightness = 0;
	static int8_t step = 4;
  /* Infinite loop */
  for(;;)
  {
        //f=1hz,则需要延时20*50ms=1s,f_max=50hz,延时20*1=20ms
        breath_period_ms =led_timer/led_freq;
        // 2. 更新亮度
        brightness += step;
        if (brightness >= 100) step = -5;
        if (brightness <= 0)   step = 5;
            
        // 3. 更新全局亮度变量 (供PWM中断使用)
        // 这个赋值是原子的，所以不需要互斥锁
            g_led_brightness = brightness; 
        
        // 4. 根据当前周期延时
	    osDelay(breath_period_ms);//100/5=20,20*50=1000
 
  }
  /* USER CODE END led_breath */
}

void mpu6050_read(void const * argument)
{
	/* USER CODE BEGIN mpu6050_read */
	/* Infinite loop */
    mpu6050_raw mpu_data;
	TickType_t xLastWakeTime = xTaskGetTickCount(); // 获取当前时间
    const TickType_t xFrequency = pdMS_TO_TICKS(5); 
    static float absolute_yaw=0;
    static float last_yaw=0;   
	for(;;)
	{
		// 1. 使用vTaskDelayUntil实现精准的周期性延时
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
		//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_10);
		//原始数据
		#if 0
            MPU_Get_Accelerometer(&ax,&ay,&az);
            MPU_Get_Gyroscope(&gx,&gy,&gz);
        #endif    
        mpu_dmp_get_data(&mpu_data);
		//MPU_Get_Gyroscope(&mpu_data.gx,&mpu_data.gy,&mpu_data.gz);
		//MPU_Get_Accelerometer(&mpu_data.gx,&mpu_data.gy,&mpu_data.gz);
		//绝对yaw角度处理
        float yaw_delta = mpu_data.yaw - last_yaw;
        if (yaw_delta > 180.0f)
        {
            yaw_delta -= 360.0f; 
        }
        else if (yaw_delta < -180.0f)
        {
            yaw_delta += 360.0f; 
        }
        absolute_yaw+=yaw_delta;
        last_yaw=mpu_data.yaw;
		
		//全局变量传递参数
		memcpy(&mpu_data_global,&mpu_data,sizeof(mpu6050_raw));
		mpu_data_global.yaw= absolute_yaw;
		mpu_rx_flag =true;  //提示数据更新

		//发送调试信息
		debug_send_uart1(1);
		//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_10);
	}
  /* USER CODE END mpu6050_read */
}

void debug_send_uart1(uint8_t t){
    uint8_t id=vofa_send_id;
	
    /*
    if(RC_CtrlData.remote.s2==3){
        id=3;
    }
    if(HAL_DMA_GetState(&hdma_usart1_tx)!=HAL_DMA_STATE_READY){
        return;
    }
    */  
	if(t!=id)
		return;
	
	switch (id)
    {
    case 1:
        #if mpu_send
            #if mpu_send_angle_gyro
				//vofa_send(1,(float)mpu_data_global.pitch); 
                //vofa_send(2,(float)mpu_data_global.pitch,(float)mpu_data_global.roll); 
				//vofa_send(3,(float)mpu_data_global.pitch,(float)mpu_data_global.roll,(float)mpu_data_global.yaw); 
                vofa_send(6,(float)mpu_data_global.pitch,(float)mpu_data_global.roll,(float)mpu_data_global.yaw
                ,(float)mpu_data_global.gx,(float)mpu_data_global.gy,(float)mpu_data_global.gz);
                 
            #else
                vofa_send(3,(float)mpu_data_global.gx,(float)mpu_data_global.gy,(float)mpu_data_global.gz);
            #endif 
			//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_10);				
         #endif
        break;
    case 2:
        #if pid_send
            if(pid_speed_mode)
                vofa_send(6,(float)pid_speed_yaw.target,(float)pid_speed_yaw.now,
                (float)(pid_speed_yaw.now - pid_speed_yaw.target),(float)pid_speed_yaw.output,
				(float)motor_info_global[motor_id_global].motor_speed,(float)mpu_data_global.gz	);
            else{
                vofa_send(6,(float)pid_angle_yaw.target,(float)pid_angle_yaw.now,(float)(pid_angle_yaw.now - pid_angle_yaw.target),(float)pid_angle_yaw.output);
            }       
			//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_10);
        #endif
        break;
    case 3:
        #if can_send_rx
		#endif
        break;
    case 4:
        #if sbus_send_chan
            vofa_send(3,(float)RC_CtrlData.remote.ch0,(float)RC_CtrlData.remote.s1,(float)RC_CtrlData.remote.s2);
			//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_10);
        #endif
        break;
    default:
        break;
    }
}