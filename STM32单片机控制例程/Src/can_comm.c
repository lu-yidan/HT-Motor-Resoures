#include "can_comm.h"
#include "can.h"

#define CAN_SLAVE_ID        0x01

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))


volatile float CurVelocity = 0;

static void _CanFilter(void)
{
    CAN_FilterTypeDef   sCAN_Filter;
    
    sCAN_Filter.FilterBank = 0;                         /* ָ��������ʼ���Ĺ����� */  
    sCAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;     /* ����ģʽΪ����λģʽ */
    sCAN_Filter.FilterScale = CAN_FILTERSCALE_16BIT;    /* ָ���˲����Ĺ�ģ */
    sCAN_Filter.FilterIdHigh = 00;
    sCAN_Filter.FilterIdLow = 00;             
    sCAN_Filter.FilterMaskIdHigh = 00;
    sCAN_Filter.FilterMaskIdLow = 00;
    sCAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sCAN_Filter.FilterActivation = ENABLE;              /* ���û���ù����� */
    sCAN_Filter.SlaveStartFilterBank = 0;               /* ѡ�������ӹ������� */
    
    HAL_CAN_ConfigFilter(&hcan, &sCAN_Filter);
}

/**
  * @brief  CAN�ӿڳ�ʼ��
  * @param
  * @retval 
  */
void CanComm_Init(void)
{
    _CanFilter();
    HAL_CAN_Start(&hcan);               /* ����CANͨ�� */  
    HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);    /* ���������ж����� */
}

/**
  * @brief  Converts a float to an unsigned int, given range and number of bits
  * @param
  * @retval 
  */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    
    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
  * @brief  converts unsigned int to float, given range and number of bits
  * @param
  * @retval 
  */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/* ��buf�е�����ͨ��CAN�ӿڷ��ͳ�ȥ */
static void CanTransmit(uint8_t *buf, uint8_t len)
{
    CAN_TxHeaderTypeDef TxHead;             /**!< canͨ�ŷ���Э��ͷ */
    uint32_t canTxMailbox;
    
    if((buf != NULL) && (len != 0))
    {
        TxHead.StdId    = CAN_SLAVE_ID;     /* ָ����׼��ʶ������ֵ��0x00-0x7FF */
        TxHead.IDE      = CAN_ID_STD;       /* ָ����Ҫ������Ϣ�ı�ʶ������ */
        TxHead.RTR      = CAN_RTR_DATA;     /* ָ����Ϣ����֡���� */
        TxHead.DLC      = len;              /* ָ����Ҫ�����֡���� */
        
        if(HAL_CAN_AddTxMessage(&hcan, &TxHead, buf, (uint32_t *)&canTxMailbox) == HAL_OK )
        {
            
        }
    }
}

/**
  * @brief  Can���߷��Ϳ��Ʋ���
  * @param
  * @retval 
  */
void CanComm_SendControlPara(float f_p, float f_v, float f_kp, float f_kd, float f_t)
{
    uint16_t p, v, kp, kd, t;
    uint8_t buf[8];
    
    /* ��������Ĳ����ڶ���ķ�Χ�� */
    LIMIT_MIN_MAX(f_p,  P_MIN,  P_MAX);
    LIMIT_MIN_MAX(f_v,  V_MIN,  V_MAX);
    LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
    LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
    LIMIT_MIN_MAX(f_t,  T_MIN,  T_MAX);
    
    /* ����Э�飬��float��������ת�� */
    p = float_to_uint(f_p,      P_MIN,  P_MAX,  16);            
    v = float_to_uint(f_v,      V_MIN,  V_MAX,  12);
    kp = float_to_uint(f_kp,    KP_MIN, KP_MAX, 12);
    kd = float_to_uint(f_kd,    KD_MIN, KD_MAX, 12);
    t = float_to_uint(f_t,      T_MIN,  T_MAX,  12);
    
    /* ���ݴ���Э�飬������ת��ΪCAN���������ֶ� */
    buf[0] = p>>8;
    buf[1] = p&0xFF;
    buf[2] = v>>4;
    buf[3] = ((v&0xF)<<4)|(kp>>8);
    buf[4] = kp&0xFF;
    buf[5] = kd>>4;
    buf[6] = ((kd&0xF)<<4)|(t>>8);
    buf[7] = t&0xff;
    
    /* ͨ��CAN�ӿڰ�buf�е����ݷ��ͳ�ȥ */
    CanTransmit(buf, sizeof(buf));
}

void CanComm_ControlCmd(uint8_t cmd)
{
    uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
    switch(cmd)
    {
        case CMD_MOTOR_MODE:
            buf[7] = 0xFC;
            break;
        
        case CMD_RESET_MODE:
            buf[7] = 0xFD;
        break;
        
        case CMD_ZERO_POSITION:
            buf[7] = 0xFE;
        break;
        
        default:
        return; /* ֱ���˳����� */
    }
    CanTransmit(buf, sizeof(buf));
}


/**
  * @brief  CAN�ӿڽ�������
  * @param
  * @retval 
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint16_t tmp_value;
    
    CAN_RxHeaderTypeDef RxHead; /**!< canͨ��Э��ͷ */
    uint8_t data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHead, data);
    
    if(data[0] == CAN_SLAVE_ID)
    {
        tmp_value = (data[3]<<4)|(data[4]>>4);
        CurVelocity = uint_to_float(tmp_value, V_MIN, V_MAX, 12);
    }
}

float CanComm_GetCurVelocity(void)
{
    return CurVelocity;
}






