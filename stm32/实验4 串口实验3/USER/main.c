#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"

/************************************************
 ALIENTEK��ӢSTM32������ʵ��4
 ���� ʵ��
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/

int main(void)
{
    u16 t;
    u16 len;
    // u16 times = 0;
    u8 data[4] = {1, 2, 3, 4};
    delay_init();									//��ʱ������ʼ��
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
    uart_init(115200);								//���ڳ�ʼ��Ϊ115200
    LED_Init();										//LED�˿ڳ�ʼ��
    while (1)
    {
        if (USART_RX_STA & 0x8000)
        {
            len = USART_RX_STA & 0x3fff; //�õ��˴ν��յ������ݳ���
            //printf("\r\n�����͵���Ϣ����Ϊ:%d\r\n", len);
            //printf("\r\n�����͵���ϢΪ:\r\n\r\n");
            for (t = 0; t < len; t++)
            {
                printf("%c", USART_RX_BUF[t]);
                //USART_SendData(USART1, USART_RX_BUF[t]); //�򴮿�1��������
                //while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET); //�ȴ����ͽ���
            }
            //printf("\r\n\r\n"); //���뻻��
            USART_RX_STA = 0;
        }
        else
        {
            for (t = 0; t < 4; t++)
            {
                printf("%hhu", data[t]);
            }

            LED0 = !LED0; //��˸LED,��ʾϵͳ��������.
            delay_ms(1000);
        }

    }
}
