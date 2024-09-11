/*************************************************************************
	> File Name: oled1.c
	> Author:
	> Mail:
	> Created Time: 2021年04月02日 星期五 10时53分55秒
 ************************************************************************/
/*
 * @Author: 陈俊鹏
 * @Date: 2016-06-27 06:23:25
 * @LastEditTime: 2021-04-02 22:58:24
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: 13/ssd0_i2c-master/oled.c6
 */
/*********************************************************************
修改者陈俊鹏
在Raspberry Pi 4B 上,连接了一个0.96寸上黄下蓝的OLED(128x64)上测试
*********************************************************************/

/*********************************************************************
这是一个基于SSD1306驱动程序的单色oled库
使用的是树莓派的i2c接口 接线方法:

GND-------GND----------任意0V
VCC-------3.3V---------任意3.3V
SDA-------SDA1---------3号引脚
SCK-------SCK1---------5号引脚
*********************************************************************/

#include "ssd1306_i2c.h"
#include <sys/ioctl.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/sysinfo.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
//#include <thread>
#define MAXBUFSIZE 1024

static int count = 0;
static struct itimerval oldtv;

void set_timer()
{
	struct itimerval itv;
	itv.it_interval.tv_sec = 0;
	itv.it_interval.tv_usec = 500000;
	itv.it_value.tv_sec = 0;
	itv.it_value.tv_usec = 500000;
	setitimer(ITIMER_REAL, &itv, &oldtv);
}

typedef  struct CPU_PACKED
{
    char name[20];
    unsigned int user;
    unsigned int nice;
    unsigned int system;
    unsigned int idle;
}CPU_OCCUPY;
//-----------------------mem--------------------
typedef  struct  MEM_PACKED
{
    char name[20];
    unsigned long total;
    char name2[20];
}MEM_OCCUPY1;

typedef struct MEM_PACK
{
    double total;
    double used_rate;
}MEM_PACK;

typedef struct PACKED
{
    char name[20];
    long total;
    char name2[20];
    long free;
}MEM_OCCUPY;
//------------------------------sysinfo-------------

/**
 * @description: CPU使用率
 * @param {CPU_OCCUPY} *o
 * @param {CPU_OCCUPY} *n
 * @return {*}实时CPU使用率
 */
int cal_cpuoccupy(CPU_OCCUPY *o, CPU_OCCUPY *n)
{
    unsigned long od, nd;
    unsigned long id, sd;
    int cpu_use = 0;

    od = (unsigned long) (o->user+ o->nice + o->system + o->idle);//第一次(用户+优先级+系统+空闲)的时间
    nd = (unsigned long) (n->user + n->nice + n->system + n->idle); //第二次

    id = (unsigned long) (n->user - o->user);
    sd = (unsigned long) (n->system - o->system);
    if((nd - od) != 0){
        cpu_use = (int)((sd+id)*100)/(nd-od);
    }else {
        cpu_use = 0;
    }
    return cpu_use;
}
/**
 * @description: 获取cpu信息
 * @param {CPU_OCCUPY} *cpust
 * @return {*}
 */
void get_cpuoccupy(CPU_OCCUPY *cpust)
{
    FILE *fd;
    int n;
    char buff[256];
    CPU_OCCUPY *cpu_occupy;
    cpu_occupy = cpust;

    fd = fopen("/proc/stat","r");
    fgets(buff,sizeof(buff),fd);
    sscanf(buff,"%s %u %u %u %u",cpu_occupy->name,&cpu_occupy->user,&cpu_occupy->nice, &cpu_occupy->system, &cpu_occupy->idle);
    fclose(fd);
}

/**
 * @description: 获取CPU占用总数和使用率
 * @param {*} 无
 * @return {*} mem.total占用总数,mem.used_rate使用率
 */
MEM_PACK get_memocupy()
{
    FILE *fd;
    int n;
    double mem_total,mem_used_rate;
    char buff[256];
    MEM_OCCUPY *m = (MEM_OCCUPY*)malloc(sizeof(MEM_OCCUPY));
    MEM_PACK p;

    fd = fopen("/proc/meminfo", "r");

    fgets(buff,sizeof (buff), fd);
    sscanf(buff, "%s %lu %s\n", m->name, &m->total, m->name2);
    mem_total = m->total;
    fgets(buff,sizeof (buff),fd);
    sscanf(buff, "%s %lu %s\n", m->name, &m->total,m->name2);
    mem_used_rate = (1 - m->total/mem_total)*100;
    mem_total = mem_total/(1024*1024);
    p.total = mem_total;
    p.used_rate = mem_used_rate;
    fclose(fd);
    free(m);
    return p;
}
/**
 * @description: 获取cpu温度
 * @param {*}  无
 * @return {*}  当前cpu温度值
 */
int get_cpu_temp()
{
    FILE *fd;
    int temp;
    char buff[256];

    fd = fopen("/sys/class/thermal/thermal_zone0/temp","r");
    fgets(buff,sizeof(buff),fd);
    sscanf(buff, "%d", &temp);

    fclose(fd);

    return temp/1000;
}

int get_last_reboot(char *cmd,char *output, int size)
{
    FILE *fp = NULL;
    fp = popen(cmd,"r");

    if(fp){
        fgets(output,size,fp);
        if(fgets(output,size,fp) != NULL){
            if(output[strlen(output) - 1] == '\n')
                output[strlen(output) - 1] = '\0';
        }
        pclose(fp);
    }
    return 0;
}

/**
 * @description: 获取IP地址
 * @param {*}
 * @return {*} ip地址
 */
char* GetLocalIp(void)
{
    int MAXINTERFACES=16;
    char *ip;
    int fd, intrface, retn = 0;
    struct ifreq buf[MAXINTERFACES];
    struct ifconf ifc;

    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) >= 0)
    {
        ifc.ifc_len = sizeof(buf);
        ifc.ifc_buf = (caddr_t)buf;
        if (!ioctl(fd, SIOCGIFCONF, (char *)&ifc))
        {
            intrface = ifc.ifc_len / sizeof(struct ifreq);

            while (intrface-- > 0)
            {
                if (!(ioctl (fd, SIOCGIFADDR, (char *) &buf[intrface])))
                {
                    ip=(inet_ntoa(((struct sockaddr_in*)(&buf[intrface].ifr_addr))->sin_addr));
                    break;
                }
            }
        }
        close (fd);
        return ip;
    }
}

/**
 * @description:  获取下载速度
 * @param {long int} *
 * @return {*}
 */
void getCurrentDownloadRates(long int * save_rate)
{
    char intface[] = "eth0:";  //这是网络接口名，根据主机配置
    //char intface[] = "wlan0:";
    FILE * net_dev_file;
    char buffer[1024];
    size_t bytes_read;
    char * match;

    if ((net_dev_file = fopen("/proc/net/dev", "r")) == NULL)
    {
        printf("open file /proc/net/dev/ error!\n");
        exit(EXIT_FAILURE);
    }

    int i = 0;
    while (i++ < 20)
    {
        if (fgets(buffer, sizeof(buffer), net_dev_file) != NULL)
        {
            if (strstr(buffer, intface) != NULL)
            {
                //printf("%d   %s\n",i,buffer);
                sscanf(buffer, "%s %ld", buffer, save_rate);
                break;
            }
        }
    }

    if (i == 20)
    {
        *save_rate = 0.01;
    }

    fclose(net_dev_file);     //关闭文件
    return;
}

/**
 * @description: 获取磁盘占用
 * @param {char} *
 * @return {*}
 */
void get_disk_occupy(char ** reused)
{
    char currentDirectoryPath[ MAXBUFSIZE ];

    getcwd(currentDirectoryPath, MAXBUFSIZE);

    //printf("当前目录：%s\n",currentDirectoryPath);
    char cmd[50] = "df ";
    strcat(cmd, currentDirectoryPath);

    //printf("%s\n",cmd);

    char buffer[MAXBUFSIZE];
    FILE* pipe = popen(cmd, "r");
    char fileSys[20];
    char blocks[20];
    char used[20];
    char free[20];
    char percent[10];
    char moment[20];

    if (!pipe)
    {
        return;
    }

    if (fgets(buffer, sizeof(buffer), pipe) != NULL)
    {
        sscanf(buffer, "%s %s %s %s %s %s", fileSys, blocks, used, free, percent, moment);
    }

    if (fgets(buffer, sizeof(buffer), pipe) != NULL)
    {
        sscanf(buffer, "%s %s %s %s %s %s", fileSys, blocks, used, free, percent, moment);
    }

    //printf("desk used:%s\n",percent);
    strcpy(*reused, percent);
    return;
}
CPU_OCCUPY cpu_stat1;
CPU_OCCUPY cpu_stat2;
time_t now;
struct tm *timenow;
char* IP_address="192.168.1.1";
int cpu;
int Byte=0;
char t[20] = "";
char *used = t;
char value[10]={'0','1','2','3','4','5','6','7','8','9'};
int temp=0;
MEM_PACK mem;
long int start_download_rates;      //保存开始时的流量计数
long int end_download_rates;        //保存结果时的流量计数

void Display(int m)
{
    //清屏
    ssd1306_clearDisplay();

    //显示时间
    ssd1306_drawString("    -  -       :  :");
    OLED_ShowNum(0,0,timenow->tm_year+1900,4);
    OLED_ShowNum(30,0,timenow->tm_mon,2);
    OLED_ShowNum(48,0,timenow->tm_mday,2);
    OLED_ShowNum(66+12,0,timenow->tm_hour,2);
    OLED_ShowNum(82+12,0,timenow->tm_min,2);
    OLED_ShowNum(100+12,0,timenow->tm_sec,2);
    ssd1306_drawString("\r\n");

    //显示IP
    ssd1306_drawString("IP=");
    ssd1306_drawString(IP_address);
    ssd1306_drawString("\r\n");

    //显示CPU温度
    ssd1306_drawString("CPU Temperature=   c");
    OLED_ShowNum(96,16,temp,2);
    ssd1306_drawString("\r\n");

    //显示CPU占用
    ssd1306_drawString("CPU usage=  %");
    OLED_ShowNum(61,24,cpu,2);
    ssd1306_drawString("\r\n");

    //显示运存占用
    ssd1306_drawString("mem use rate=  %");
    OLED_ShowNum(78,32,mem.used_rate,2);
    ssd1306_drawString("\r\n");

    //显示实时网速
    ssd1306_drawString("Download=       B/s");
    OLED_ShowNum(54,40,Byte,6);
    ssd1306_drawString("\r\n");

    //显示实时磁盘读写
    ssd1306_drawString("Disk used=");
    ssd1306_drawString(used);

    ssd1306_display();
}

void main() {
    //屏幕初始化
    ssd1306_begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS);
    //绑定定时器执行函数
    signal(SIGALRM, Display);
    //定时器启用中断  每0.5秒执行一次,就是2帧
	set_timer();
    while (1)
    {
        //获取时间
        time(&now);
        timenow = localtime(&now);

        //获取IP地址
        IP_address=GetLocalIp();

        //获取CPU温度
        temp = get_cpu_temp();

        //获取CPU占用
        get_cpuoccupy((CPU_OCCUPY*)&cpu_stat1);
        sleep(10);
        get_cpuoccupy((CPU_OCCUPY*)&cpu_stat2);
        cpu = cal_cpuoccupy((CPU_OCCUPY *)&cpu_stat1,(CPU_OCCUPY*)&cpu_stat2);

        //获取内存占用
        mem = get_memocupy();

        //获取实时流量
        getCurrentDownloadRates(&start_download_rates);    //获取当前流量，并保存在start_download_rates里
        sleep(3);     //休眠多少秒，这个值根据宏定义中的WAIT_SECOND的值来确定
        getCurrentDownloadRates(&end_download_rates);    //获取当前流量，并保存在end_download_rates里
        Byte=(end_download_rates - start_download_rates) / 3;


        //获取当前磁盘的使用率
        get_disk_occupy(&used);

    }
}