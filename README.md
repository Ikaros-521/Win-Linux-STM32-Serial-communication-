## 前言
相关环境：win10、Linux（CentOS7）、正点原子STM32F103精英版、Keil5、VS Code、MinGW
语言：C语言
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210407142446490.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
## demo下载
码云：[传送门（点我）](https://gitee.com/ikaros-521/Win-Linux-STM32-Serial-communication)  GitHub：[传送门（点我）](https://github.com/Ikaros-521/Win-Linux-STM32-Serial-communication-)
## STM32
### 程序篇
简单展示
#### main.c

```c
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

```
#### usart.c

```c
#include "sys.h"
#include "usart.h"
//////////////////////////////////////////////////////////////////////////////////
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//串口1初始化
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/8/18
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持
//////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
    int handle;

};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
    x = x;
}
//重定义fputc函数
int fputc(int ch, FILE *f)
{
    while((USART1->SR&0X40)==0);//循环发送,直到发送完毕
    USART1->DR = (u8) ch;
    return ch;
}
#endif

/*使用microLib的方法*/
/*
int fputc(int ch, FILE *f)
{
USART_SendData(USART1, (uint8_t) ch);

while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}

   return ch;
}
int GetKey (void)  {

   while (!(USART1->SR & USART_FLAG_RXNE));

   return ((int)(USART1->DR & 0x1FF));
}
*/

#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记

void uart_init(u32 bound) {
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟

    //USART1_TX   GPIOA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9

    //USART1_RX	  GPIOA.10初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10

    //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

    //USART 初始化设置

    USART_InitStructure.USART_BaudRate = bound;//串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

    USART_Init(USART1, &USART_InitStructure); //初始化串口1
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
    USART_Cmd(USART1, ENABLE);                    //使能串口1

}

void USART1_IRQHandler(void)                	//串口1中断服务程序
{
    u8 Res;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
    OSIntEnter();
#endif
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
    {
        Res =USART_ReceiveData(USART1);	//读取接收到的数据

        if((USART_RX_STA&0x8000)==0)//接收未完成
        {
            if(USART_RX_STA&0x4000)//接收到了0x0d
            {
                if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
                else USART_RX_STA|=0x8000;	//接收完成了
            }
            else //还没收到0X0D
            {
                if(Res==0x0d)USART_RX_STA|=0x4000;
                else
                {
                    USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
                    USART_RX_STA++;
                    if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收
                }
            }
        }
    }
#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
    OSIntExit();
#endif
}
#endif


```
### 烧录测试
USB_232接电脑USB
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210407142850899.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
FlyMcu烧录工具
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210407142926726.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
串口调试工具 XCOM打开COM5
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210407143028449.png)
循环发送1234，32收到的数据返回，测试结束。
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210407143015553.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
## win10
程序 转自：https://zhidao.baidu.com/question/1770933168372746220.html
编码：UTF-8
### 1.c

```c
// 代码转自：https://zhidao.baidu.com/question/1770933168372746220.html 进行改写
#include <Windows.h>
#include <stdio.h>

int main(void)
{
    system("chcp 65001");
    char com[10] = {0};
    printf("请输入COM口，如COM1：");
    gets(com);
    HANDLE hCom = CreateFile(TEXT(com),                    //COM口
                             GENERIC_READ | GENERIC_WRITE, //允许读和写
                             0,                            //独占方式
                             NULL,
                             OPEN_EXISTING, //打开而不是创建
                             0,             //同步方式
                             NULL);
    if (hCom == (HANDLE)-1)
    {
        printf("打开COM失败!\n");
        return FALSE;
    }
    else
    {
        printf("COM打开成功！\n");
    }
    SetupComm(hCom, 1024, 1024); //输入缓冲区和输出缓冲区的大小都是1024
    COMMTIMEOUTS TimeOuts;
    //设定读超时
    TimeOuts.ReadIntervalTimeout = 1000;
    TimeOuts.ReadTotalTimeoutMultiplier = 500;
    TimeOuts.ReadTotalTimeoutConstant = 5000;
    //设定写超时
    TimeOuts.WriteTotalTimeoutMultiplier = 500;
    TimeOuts.WriteTotalTimeoutConstant = 2000;
    SetCommTimeouts(hCom, &TimeOuts); //设置超时
    DCB dcb;
    GetCommState(hCom, &dcb);
    dcb.BaudRate = 115200;       //波特率为9600
    dcb.ByteSize = 8;            //每个字节有8位
    dcb.Parity = NOPARITY;       //无奇偶校验位
    dcb.StopBits = ONE5STOPBITS; //两个停止位
    SetCommState(hCom, &dcb);
    DWORD rCount; //读取的字节数
    DWORD wCount; //读取的字节数
    BOOL bReadStat;
    BOOL bWriteStat;
    char buf[10] = "test\r\n";
    int num = 0;
    while (1)
    {
        PurgeComm(hCom, PURGE_TXCLEAR | PURGE_RXCLEAR); //清空缓冲区

        if (num == 3)
        {
            num = 0;

            bWriteStat = WriteFile(hCom,        //串口句柄
                                   buf,         //数据首地址
                                   strlen(buf), //要发送的数据字节数
                                   &wCount,     //DWORD*，用来接收返回成功发送的数据字节数
                                   NULL);       //NULL为同步发送，OVERLAPPED*为异步发送
            if (!bWriteStat)
            {
                printf("发串口失败!\n");
                return FALSE;
            }
            else
            {
                printf("发送数据:%s\n", buf);
                //printf("成功发送字节:%d\n", wCount);
            }
        }

        char str[1024] = {0};
        bReadStat = ReadFile(hCom, str, 4, &rCount, NULL);
        if (!bReadStat)
        {
            printf("读串口失败!\n");
            return FALSE;
        }
        else
        {
            str[4] = '\0';
            printf("收到数据:%s\n", str);
        }

        num++;
        Sleep(100);
    }

    CloseHandle(hCom);

    return 0;
}
```
### 运行效果
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210407143317107.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
## Linux
接法一样，USB接入工控机USB口。
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210407145913618.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
### 程序
uart.c 转自网络 找不到出处了尴尬。。。

```c
#include <stdio.h>     /*标准输入输出定义*/
#include <stdlib.h>    /*标准函数库定义*/
#include <unistd.h>    /*Unix标准函数定义*/
#include <sys/types.h> /**/
#include <sys/stat.h>  /**/
#include <fcntl.h>     /*文件控制定义*/
#include <termios.h>   /*PPSIX终端控制定义*/
#include <errno.h>     /*错误号定义*/

#define TRUE 1
#define FALSE 0

int speed_arr[] = {B38400, B19200, B9600, B4800, B2400, B1200, B300, B38400, B19200, B9600, B4800, B2400, B1200, B300,};
int name_arr[] = {38400, 19200, 9600, 4800, 2400, 1200, 300, 38400, 19200, 9600, 4800, 2400, 1200, 300,};

/**
*@brief   设置串口通信速率
*@param   fd     类型 int  打开串口的文件句柄
*@param   speed  类型 int  串口速度
*@return  void
*/
void set_speed(int fd, int speed)
{
    int i = 0;
    int status = 0;
    /*
    struct termios
    {
        unsigned short c_iflag;  //输入模式
        unsigned short c_oflag;  //输出模式
        unsigned short c_cflag;  //控制模式
        unsigned short c_lflag;  //本地模式
        unsigned char c_cc[NCC]; //控制 字符 数据
    }
    */
    struct termios opt;
    // 获取终端参数，成功返回零；失败返回非零，发生失败接口将设置errno错误标识 返回的结果保存在termios结构体
    tcgetattr(fd, &opt);
    for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++)
    {
        if (speed == name_arr[i])
        {
            /* 
            清空终端未完成的输入/输出请求及数据
            TCIFLUSH 　清空输入缓存
            TCOFLUSH 　清空输入缓存
            TCIOFLUSH  清空输入输出缓存 
            */
            tcflush(fd, TCIOFLUSH);
            // 设置输入输出波特率
            cfsetispeed(&opt, speed_arr[i]);
            cfsetospeed(&opt, speed_arr[i]);
            // 设置终端参数
            status = tcsetattr(fd, TCSANOW, &opt);
            if (status != 0)
                perror("tcsetattr fd1");
            return;
        }
        tcflush(fd, TCIOFLUSH);
    }
}

/**
*@brief   设置串口数据位，停止位和校验位
*@param   fd       类型  int  打开的串口文件句柄*
*@param   databits 类型  int  数据位   取值 为 7 或者8*
*@param   stopbits 类型  int  停止位   取值为 1 或者2*
*@param   parity   类型  int  校验类型 取值为N,E,O,,S
*/
int set_parity(int fd, int databits, int stopbits, int parity)
{
    struct termios options;
    // 获取终端参数，成功返回零；失败返回非零，发生失败接口将设置errno错误标识 返回的结果保存在termios结构体
    if (tcgetattr(fd, &options) != 0)
    {
        perror("SetupSerial 1");
        return (FALSE);
    }
    // 控制模式标志，指定终端硬件控制信息
    options.c_cflag &= ~CSIZE;
    // 设置数据位数
    switch (databits)
    {
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            // int fprintf(FILE *stream, const char *format, ...)
            fprintf(stderr, "Unsupported data size\n");
            return (FALSE);
    }
    // 设置校验位
    switch (parity)
    {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB; /* Clear parity enable */
            options.c_iflag &= ~INPCK;  /* Enable parity checking */
            break;
        case 'o':
        case 'O':
            options.c_cflag |= (PARODD | PARENB); /* 设置为奇校验*/
            options.c_iflag |= INPCK;             /* Disnable parity checking */
            break;
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;  /* Enable parity */
            options.c_cflag &= ~PARODD; /* 转换为偶校验*/
            options.c_iflag |= INPCK;   /* Disnable parity checking */
            break;
        case 'S':
        case 's': /*as no parity*/
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            fprintf(stderr, "Unsupported parity\n");
            return (FALSE);
    }
    // 设置停止位
    switch (stopbits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            fprintf(stderr, "Unsupported stop bits\n");
            return (FALSE);
    }
    // 设置输入奇偶校验选项
    if (parity != 'n')
        options.c_iflag |= INPCK;
    // 15秒
    options.c_cc[VTIME] = 150;
    options.c_cc[VMIN] = 0;

    // 清空终端未完成的输入/输出请求及数据
    tcflush(fd, TCIFLUSH);
    // 设置终端参数
    if (tcsetattr(fd, TCSANOW, &options) != 0)
    {
        perror("SetupSerial 3");
        return (FALSE);
    }
    return (TRUE);
}

// 打开串口
int open_dev(char *dev)
{
    // 读、写打开 | 如果pathname指的是终端设备，则不将此设备分配作为此进程的控制终端 | O_NONBLOCK 如果pathname指的是一个FIFO、一个块特殊文件或一个字符特殊文件，则此选择项为此文件的本次打开操作和后续的I/O操作设置非阻塞方式
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (-1 == fd)
    {
        perror("Can't Open Serial Port");
        return -1;
    }
    else
        return fd;
}

// 十六进制打印无符号char数组
void print_data(unsigned char *data, int len)
{
	int i = 0;

	for (i = 0; i < len; i++)
	{
		if (i && i % 16 == 0)
			printf("\n");
		printf("0x%02x ", data[i]);
	}
	printf("\n");
}

// 主函数
int main(int argc, char **argv)
{
    int fd = 0;
    int size = 0;
    unsigned char buf[1024] = {0};
    char *dev = "/dev/ttyUSB0";
    fd = open_dev(dev);
    if (fd > 0)
        set_speed(fd, 115200);
    else
    {
        printf("Can't Open Serial Port!\n");
        exit(0);
    }
    if (set_parity(fd, 8, 1, 'N') == FALSE)
    {
        printf("Set Parity Error\n");
        exit(1);
    }

    char text[7] = "6666\r\n";
    int num = 0;

    while (1)
    {
        if(num == 30)
        {
            num = 0;
            write(fd, text, 6);
            printf("send:%s\n", text);
        }

        while ((size = read(fd, buf, 512)) > 0)
        {
            buf[size] = '\0';
            printf("[uart] Len=%d, msg:%s\n", size, buf);
            print_data((unsigned char*)buf, size);
        }

        tcflush(fd, TCIFLUSH);

        num++;
        usleep(100000);
    }

    close(fd);
    exit(0);
}

```

### 效果图
![在这里插入图片描述](https://img-blog.csdnimg.cn/2021040715104918.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)

