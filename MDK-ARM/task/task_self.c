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

//全局标志区
bool sbus_rx_flag=false;
uint8_t can_rx_flag=0;
bool mpu_rx_flag=false;
volatile ControlState_t g_robot_control_state = CONTROL_DISABLED;
volatile uint8_t remote_s2 = 0;
volatile uint8_t bopan_single_shoot_flag = 0;
volatile uint8_t bopan_continuous_shoot_flag = 0;
uint8_t  vofa_send_id= 2;
 
//全局数据区
volatile float led_freq=1;
volatile uint16_t g_led_brightness = 0;
volatile uint8_t ble_control_id=0,ble_control_id_global=0;
mpu6050_raw mpu_data_global;
moto_info_t motor_info_global[MOTOR_MAX_NUM];
ComplementaryFilter myComplementaryFilter[MOTOR_MAX_NUM];	
uint16_t bopan_delta_angle=1440;    //拨盘电机每次单发的角度增量，单位：编码器计数值

//--- left_whell and right whell and bopandianji 	
pid_pos pid_speed_left_whell =   {.Kp = 12, .Ki = 0, .Kd = 0,.integral_max=0, .k_f=0,
    .output_max = 16384,.target=0,.now=0,.last_now=0,.integral=0,.output=0,.last_error=0,.integral_threshold=0};

pid_pos pid_speed_right_whell =  {.Kp = 12, .Ki = 0, .Kd = 0,.integral_max=0, .k_f=0,
    .output_max = 16384,.target=0,.now=0,.last_now=0,.integral=0,.output=0,.last_error=0,.integral_threshold=0};

pid_pos pid_speed_bopandianji =  {.Kp = 7, .Ki = 0, .Kd = 0,.integral_max=0, .k_f=0,
    .output_max = 16384,.target=0,.now=0,.last_now=0,.integral=0,.output=0,.last_error=0,.integral_threshold=0};

pid_pos pid_angle_bopamdianji =  {.Kp = 10, .Ki = 0, .Kd = 2,.integral_max=0, .k_f=0,
    .output_max = 6000,.target=0,.now=0,.last_now=0,.integral=0,.output=0,.last_error=0,.integral_threshold=1};	
	
static int16_t motor_output[5] = {0}; // 用于存储PID计算结果

//任务实现区
void can1_rx(void const * argument){

    /* USER CODE BEGIN can1_rx */
    TickType_t xLastWakeTime = xTaskGetTickCount(); // 获取当前时间
    const TickType_t xFrequency = pdMS_TO_TICKS(1); 

    /* Infinite loop */
    //初始化互补滤波器

    static float local_gy=0,local_gz=0;

    for(;;)
    {
        // 1. 使用vTaskDelayUntil实现精准的周期性延时
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        if(can_rx_flag){
            memcpy(&motor_info_global,&motor_info,sizeof(moto_info_t)*MOTOR_MAX_NUM);
            can_rx_flag=false;
        }
		
        if(mpu_rx_flag){
            //互补滤波器融合陀螺仪和电机速度
            local_gz=mpu_data_global.gz;
            local_gy=mpu_data_global.gy;
            mpu_rx_flag=false;
        }

       //计算各电机输出

        motor_output[4] = //yaw motor
        pid_speed_angle_task(
            local_gz,
            motor_info_global[4].motor_angle,
            &pid_angle_yaw,
            &pid_speed_yaw,
            4
        );

        motor_output[0]= //pitch motor
        pid_speed_angle_task(
            local_gy,
            motor_info_global[0].motor_angle,
            &pid_angle_pitch,
            &pid_speed_pitch,
            0
        );

        motor_output[1]= //left whell
        pid_speed_task(
            motor_info_global[1].motor_speed,
            &pid_speed_left_whell,
            1
        );
		 
        motor_output[2]= //right whell
        pid_speed_task(
            motor_info_global[2].motor_speed,
            &pid_speed_right_whell,
            2
        );

        motor_output[3]= //bopandianji
        pid_speed_angle_task(
            motor_info_global[3].motor_speed,
            motor_info_global[3].motor_angle,
            &pid_angle_bopamdianji,
            &pid_speed_bopandianji,
            3
        );
		 
        // 输出电压到电机
        if(g_robot_control_state == CONTROL_ENABLED) {
            set_motor_voltage(0x1FF,0, -motor_output[0], 0, 0, &hcan1);//pitch motor
            //set_motor_voltage(0x1FF,0, 0, 0, motor_output[4], &hcan2);//yaw motor
            set_motor_voltage(0x200,motor_output[1],motor_output[2],motor_output[3],0,&hcan1);//left wheel
        }
		else{
			set_motor_voltage(0x200,0,0,0,0,&hcan1);//left wheel
		}
        
		//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_10);
    }
    /* USER CODE END can1_rx */
}

void debug_send_uart1(uint8_t t){

	switch (t)
    {
    case 1:
            vofa_send(6,(float)mpu_data_global.pitch,(float)mpu_data_global.roll,(float)mpu_data_global.yaw
            ,(float)mpu_data_global.gx,(float)mpu_data_global.gy,(float)mpu_data_global.gz);
    case 2:
		    ble_control_id_global=ble_control_id;
            switch (ble_control_id_global)
            {
            case 0:
                vofa_send(6,(float)pid_speed_yaw.target,(float)pid_speed_yaw.now,
                (float)(pid_speed_yaw.now - pid_speed_yaw.target),(float)pid_speed_yaw.output,
                (float)mpu_data_global.gz);
                break;
            case 1:
                vofa_send(6,(float)pid_angle_yaw.target,(float)pid_angle_yaw.now,
                (float)(pid_angle_yaw.now - pid_angle_yaw.target),(float)pid_angle_yaw.output,
                (float)mpu_data_global.gz);
                break;
            case 2:
                vofa_send(6,(float)pid_speed_pitch.target,(float)pid_speed_pitch.now,
                (float)(pid_speed_pitch.now - pid_speed_pitch.target),(float)pid_speed_pitch.output,
                (float)mpu_data_global.gy);
                break;
            case 3:
                vofa_send(6,(float)pid_angle_pitch.target,(float)pid_angle_pitch.now,
                (float)(pid_angle_pitch.now - pid_angle_pitch.target),(float)pid_angle_pitch.output,
                (float)mpu_data_global.gy);
                break;
            case 4:{
                vofa_send(6,(float)pid_speed_left_whell.target,(float)pid_speed_left_whell.now,
                (float)(pid_speed_left_whell.now - pid_speed_left_whell.target),(float)pid_speed_left_whell.output,
                (float)motor_info_global[1].motor_speed);
                switch (remote_s2)
                {
                    case 1:
                        pid_speed_left_whell.target=6000;
                        break;
                    case 3:
                        pid_speed_left_whell.target=0;
                        break;
                    default:
                        break;
                }
                break;
            }
            case 5:{
                vofa_send(6,(float)pid_speed_right_whell.target,(float)pid_speed_right_whell.now,
                (float)(pid_speed_right_whell.now - pid_speed_right_whell.target),(float)pid_speed_right_whell.output,
                (float)motor_info_global[2].motor_speed);
                switch (remote_s2)
                {
                    case 1:
                        pid_speed_right_whell.target=6000;
                        break;
                    case 3:
                        pid_speed_right_whell.target=0;
                        break;
                    default:
                        break;
                }
                break;
            }
            case 6:{
                vofa_send(6,(float)pid_speed_bopandianji.target,(float)pid_speed_bopandianji.now,
                (float)(pid_speed_bopandianji.now - pid_speed_bopandianji.target),(float)pid_speed_bopandianji.output,
                (float)motor_info_global[3].motor_speed);
                break;
            }
            case 7:{
                vofa_send(6,(float)pid_angle_bopamdianji.target,(float)pid_angle_bopamdianji.now,
                (float)(pid_angle_bopamdianji.now - pid_angle_bopamdianji.target),(float)pid_angle_bopamdianji.output,
                (float)motor_info_global[3].motor_angle);
                #if bopan_debug
                #else
                switch (remote_s2)
                {
                    case 1:
                        bopan_single_shoot_flag=1;
                        break;
                    case 3:
                        bopan_continuous_shoot_flag=0;
                        bopan_single_shoot_flag=0;
                        break;
                    case 2:
                        bopan_continuous_shoot_flag=1;
                        break;
                    default:
                        break;
                }
                #endif
                break;
            }
            default:
                break;
            }
        
        break;
    case 3:
        vofa_send(3,(float)RC_CtrlData.remote.ch0,(float)RC_CtrlData.remote.s1,(float)RC_CtrlData.remote.s2);
        break;
    case 4:
    vofa_send(5,(float)motor_info_global[0].motor_angle,(float)motor_info_global[1].motor_angle,
        (float)motor_info_global[2].motor_angle,(float)motor_info_global[3].motor_angle
        ,(float)motor_info_global[4].motor_angle);
    //由于只带了jlink,而且jlink不知道为什么会使vofa打不开串口,一开就闪退,只能用prinftf调试
    case 5:
    my_printf("LED freq: %.2f Hz, brightness: %d\r\n",led_freq,g_led_brightness);
        break;
    default:
        break;
    }
}

void bopan_motor_set(void const * argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(25); // 20Hz 任务周期

    // --- 状态变量 ---
    // 用于单发模式的 "边沿检测"
    static uint8_t last_single_shoot_flag = 0; 
    // 用于连发模式的 "状态翻转" (0代表设置100, 1代表设置0)
    static uint8_t continuous_phase = 0; 

    for(;;)
    {
        // 1. 使用vTaskDelayUntil实现精准的20Hz周期
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        ///debug时,先跳过单发连发逻辑
        #if bopan_debug
            continue;
        #endif
        ///
        // 2. 最高优先级：检查机器人是否使能。如果未使能，则强制目标为0并复位所有状态。
        if (g_robot_control_state != CONTROL_ENABLED) {
            //pid_angle_bopamdianji.target = pid_angle_bopamdianji.now; // 保持当前位置
            // 复位状态机，以便下次使能时能有正确的初始状态
            last_single_shoot_flag = 0;
            continuous_phase = 0;
            continue; // 直接进入下一次循环等待
        }

        // 控制逻辑 (优先级: 连发 > 单发 > 停止) ---
        // 3. 检查是否处于连发模式
        if (bopan_continuous_shoot_flag == 1) 
        {
            // 根据当前所处的阶段设置目标值
            if (continuous_phase == 0) {
                pid_angle_bopamdianji.target += bopan_delta_angle;
            } else {
                //pid_angle_bopamdianji.target = pid_angle_bopamdianji.now; // 保持当前位置
            }
            // 翻转状态，为下一次循环做准备
            continuous_phase = !continuous_phase;
            
            // 因为连发模式激活，所以重置单发检测的状态，防止模式切换时出现意外
            last_single_shoot_flag = 0;
        }
        // 4. 如果不处于连发模式，则检查单发模式
        else if (bopan_single_shoot_flag == 1 && last_single_shoot_flag == 0)
        {
            // 这是“上升沿”，是单发指令触发的唯一时刻
            pid_angle_bopamdianji.target += bopan_delta_angle;

            // 连发模式未激活，将其状态机复位
            continuous_phase = 0;
        }
        // 5. 如果以上模式均不满足，则为停止状态
        else 
        {
            //pid_angle_bopamdianji.target = pid_angle_bopamdianji.now; // 保持当前位置
            
            // 连发模式未激活，将其状态机复位
            continuous_phase = 0;
        }

        // 6. 在循环的最后，更新 "上一时刻" 的单发标志位状态，为下一次的边沿检测做准备
        last_single_shoot_flag = bopan_single_shoot_flag;
    }
}


//-------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------




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
            remote_s2 = RC_CtrlData.remote.s2;
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
        //mpu_dmp_get_data(&mpu_data);
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
        debug_send_uart1(5);

	}
  /* USER CODE END mpu6050_read */
}

