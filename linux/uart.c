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
