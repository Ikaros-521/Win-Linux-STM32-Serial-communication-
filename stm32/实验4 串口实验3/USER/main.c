#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"

/************************************************
 ALIENTEK精英STM32开发板实验4
 串口 实验
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司
 作者：正点原子 @ALIENTEK
************************************************/

int main(void)
{
    u16 t;
    u16 len;
    // u16 times = 0;
    u8 data[4] = {1, 2, 3, 4};
    delay_init();									//延时函数初始化
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
    uart_init(115200);								//串口初始化为115200
    LED_Init();										//LED端口初始化
    while (1)
    {
        if (USART_RX_STA & 0x8000)
        {
            len = USART_RX_STA & 0x3fff; //得到此次接收到的数据长度
            //printf("\r\n您发送的消息长度为:%d\r\n", len);
            //printf("\r\n您发送的消息为:\r\n\r\n");
            for (t = 0; t < len; t++)
            {
                printf("%c", USART_RX_BUF[t]);
                //USART_SendData(USART1, USART_RX_BUF[t]); //向串口1发送数据
                //while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET); //等待发送结束
            }
            //printf("\r\n\r\n"); //插入换行
            USART_RX_STA = 0;
        }
        else
        {
            for (t = 0; t < 4; t++)
            {
                printf("%hhu", data[t]);
            }

            LED0 = !LED0; //闪烁LED,提示系统正在运行.
            delay_ms(1000);
        }

    }
}
