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