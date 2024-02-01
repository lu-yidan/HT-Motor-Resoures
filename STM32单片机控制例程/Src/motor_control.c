#include "motor_control.h"
#include "can_comm.h"
#include "math.h"

volatile uint32_t pre_PCtick = 0;       /**!< λ�ÿ���ǰһ��tick */
volatile uint32_t pre_VCtick = 0;       /**!< �ٶȿ���ǰһ��tick */
volatile uint32_t loop_counter = 0;
volatile float f_p = 0;

/* �ڷ��͵��λ����ǰ����Ҫ�ѵ�������п��Ʋ�������Ϊ0 */
static void ZeroPosition(void)
{
    CanComm_ControlCmd(CMD_MOTOR_MODE);
    HAL_Delay(100);
    CanComm_SendControlPara(0,0,0,0,0);
    HAL_Delay(100);
}


/* ����������� */
void MotorControl_Start(void)
{
    ZeroPosition();
    CanComm_ControlCmd(CMD_ZERO_POSITION);
    loop_counter = 0;
    pre_PCtick = HAL_GetTick();
    pre_VCtick = HAL_GetTick();
}

void MotorControl_Stop(void)
{
    CanComm_ControlCmd(CMD_RESET_MODE);
}

/**
  * @brief  ���λ�ÿ���
  * @retval false :�����һ�����ڵĿ���
            true��û�����һ�����ڵĿ���
  */
bool MotorControl_PositionHandler(void)
{
    float t;
    
    uint32_t tick = HAL_GetTick();
    
    if((tick - pre_PCtick) >= 10)      /**!< 10ms����һ�ο��� */
    {
        pre_PCtick = tick;
        loop_counter++;
        t = 0.01*loop_counter;      /**!< ��ʱ����ʱ�ӵδ�Ϊ10ms */
        
        if(t < 3)
        {
            f_p = -20.0f+20.0f*cos(t);
        }
        else if((t > 3)&&(t < 6))
        {
            f_p = -60.0f+20*cos(t-3);
        }
        
        if(t > 6)
        {
            CanComm_ControlCmd(CMD_RESET_MODE);     /**!< �������RESETģʽ */
            loop_counter = 0;                       /**!< ����count���� */
            return false;                           /**!< �������������ڵĿ��� */
        }
        
        CanComm_SendControlPara(f_p, 0, 5, 0.2, 0);
    }
    return true;
}


/**
  * @brief  �ٶȱջ�����
  * @retval false :�����һ�����ڵĿ���
            true��û�����һ�����ڵĿ���
  */
bool MotorControl_velocityHandler(void)
{
    uint32_t tick = HAL_GetTick();  
    
    if((tick - pre_VCtick) >= 10)
    {
        pre_VCtick = tick;
        
        CanComm_SendControlPara(0, 1, 0, 1, 0.2);
    }
    return true;
}

