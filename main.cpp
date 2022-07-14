#include <unistd.h>
#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <signal.h>
#include <termios.h>
#include <sys/types.h>
#include <poll.h>
#include "my_easylogging.h"
int ttyfd;
int stdout_changed = 0;
int tty_changed = 0;
struct termios stdout_tio;
struct termios tty_tio;
using namespace std;

string get_sim_num();
int set_blue_alarm_led(int value);
string get_gps();

int main(int args, const char *argv[])
{
    // if (args != 2)
    // {
    //     fprintf(stderr, "Usage: ./sendat ttyUSB\n");
    //     exit(EXIT_FAILURE);
    // }
    easylogginginit(); //日志初始化
    // get_sim_num();
    // set_blue_alarm_led(1);
    get_gps();
    return 0;
}
/**
 * @brief Get the sim num object
 * 获取sim卡号 若格网关
 * @return string
 */
string get_sim_num()
{
    string sim_num;

    char readbuf[256];
    ttyfd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY);
    if (ttyfd < 0)
    {
        perror("open");
        exit(EXIT_FAILURE);
    }
    int ret = write(ttyfd, "AT+ICCID\r\n", 10);
    if (ret < 0)
    {
        perror("write");
        exit(EXIT_FAILURE);
    }
    usleep(100 * 1000);

    ret = read(ttyfd, readbuf, sizeof(readbuf));
    if (ret < 0 && errno != EAGAIN)
    {
        perror("read");
        exit(EXIT_FAILURE);
    }
    // LOG(INFO) << "ret = " << ret;

    LOG(INFO) << "readbuf =" << readbuf;

    for (int i = 0; i < ret; i++)
    {
        if (readbuf[i] >= 0x30 && readbuf[i] <= 0x39)
        {
            sim_num.push_back(readbuf[i]);
        }
    }
    LOG(INFO) << "sim_num = " << sim_num;
    LOG(INFO) << "sim_num.size  = " << sim_num.size();

    // if (ret > 0)
    //     write(ttyfd, "AT+ICCID\r\n", 10);
    // sim_num =to_string()
    return sim_num;
}
/*
+ICCID: 89860621330066521767

OK
*/

/**
 * @brief Set the red alarm led object
 * 蓝色表示 柴油发电机 水温高报警
 * 报警灯的黄色接到 网关DO2端 echo 1 > /sys/class/gpio/gpio53/value
 * @param  value           1 启动  0关闭
 * @return string
 */
int set_blue_alarm_led(int value)
{
    // string diesel_oil_level_sensor_2_status;
    // char readbuf[256];
    int ttyfd = open("/sys/class/gpio/gpio53/value", O_RDWR);
    if (ttyfd < 0)
    {
        return 0;
    }

    if (value > 0)
    {
        //开启报警灯
        int ret = write(ttyfd, "1", 1);
        if (ret < 0)
        {
            close(ttyfd);
            return 0;
        }
    }
    else
    {
        // 关闭报警灯
        int ret = write(ttyfd, "0", 1);
        if (ret < 0)
        {
            close(ttyfd);
            return 0;
        }
    }

    close(ttyfd);
    return 0;
}

string get_gps()
{
    string sim_num;
    int ttyfd;
    int stdout_changed = 0;
    int tty_changed = 0;
    struct termios stdout_tio;
    struct termios tty_tio;
    char readbuf[256];

    ttyfd = open("/dev/ttyUSB2", O_RDWR | O_NOCTTY | O_NDELAY);
    if (ttyfd < 0)
    {
        return "open error";
    }

    struct termios tio;
    int ret = tcgetattr(ttyfd, &tio);
    if (ret == -1)
    {
        perror("tcgetattr");
    }
    memcpy(&tty_tio, &tio, sizeof(struct termios));
    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_cflag = CS8 | CREAD | CLOCAL;
    tio.c_cflag &= (~CRTSCTS);
    tio.c_lflag = 0;
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 0;
    if (cfsetospeed(&tio, B115200) < 0 || cfsetispeed(&tio, B115200) < 0)
    {
        perror("cfseti/ospeed");
    }
    ret = tcsetattr(ttyfd, TCSANOW, &tio);
    if (ret == -1)
    {
        perror("tcsetattr");
    }
    tty_changed = 1;

    struct termios outio;
    ret = tcgetattr(STDIN_FILENO, &outio);
    if (ret == -1)
    {
        perror("tcgetattr");
    }
    memcpy(&stdout_tio, &outio, sizeof(struct termios));
    stdout_changed = 1;
    outio.c_lflag &= ~ECHO;
    ret = tcsetattr(STDIN_FILENO, TCSANOW, &outio);

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    // struct pollfd fds[] = {
    //     {STDIN_FILENO, POLLIN, 0},
    //     {ttyfd, POLLIN, 0}};

    ret = write(ttyfd, "ATE0\r\n", 6); //关闭回显
    if (ret < 0)
    {
        close(ttyfd);
        return "write error";
    }
    usleep(500 * 1000);
    ret = read(ttyfd, readbuf, sizeof(readbuf));
    if (ret < 0 && errno != EAGAIN)
    {
        close(ttyfd);
        return "read error";
    }
    LOG(INFO) << "ATE0 readbuf =" << readbuf;
    memset(readbuf, 0, sizeof(readbuf));

    ret = write(ttyfd, "at+qgps\r\n", 9); //打开GPS
    if (ret < 0)
    {
        close(ttyfd);
        return "write error";
    }
    usleep(500 * 1000);
    ret = read(ttyfd, readbuf, sizeof(readbuf));
    if (ret < 0 && errno != EAGAIN)
    {
        close(ttyfd);
        return "read error";
    }
    LOG(INFO) << "at+qgps readbuf =" << readbuf;
    memset(readbuf, 0, sizeof(readbuf));

    ret = write(ttyfd, "AT+QGPSCFG=\"gnssconfig\",0\r\n", 27); //配置1
    if (ret < 0)
    {
        close(ttyfd);
        return "write error";
    }
    usleep(500 * 1000);
    ret = read(ttyfd, readbuf, sizeof(readbuf));
    if (ret < 0 && errno != EAGAIN)
    {
        close(ttyfd);
        return "read error";
    }
    LOG(INFO) << "AT+QGPSCFG=\"gnssconfig\",0 readbuf =" << readbuf;
    memset(readbuf, 0, sizeof(readbuf));

    ret = write(ttyfd, "at+qgps=1\r\n", 11); //配置2
    if (ret < 0)
    {
        close(ttyfd);
        return "write error";
    }
    usleep(500 * 1000);
    ret = read(ttyfd, readbuf, sizeof(readbuf));
    if (ret < 0 && errno != EAGAIN)
    {
        close(ttyfd);
        return "read error";
    }
    LOG(INFO) << "at+qgps=1 readbuf =" << readbuf;
    memset(readbuf, 0, sizeof(readbuf));

    ret = write(ttyfd, "AT+QGPSLOC=0\r\n", 14); //配置3
    if (ret < 0)
    {
        close(ttyfd);
        return "write error";
    }
    usleep(500 * 1000);
    ret = read(ttyfd, readbuf, sizeof(readbuf));
    if (ret < 0 && errno != EAGAIN)
    {
        close(ttyfd);
        return "read error";
    }
    LOG(INFO) << "AT+QGPSLOC=0 readbuf =" << readbuf;
    memset(readbuf, 0, sizeof(readbuf));

    ret = write(ttyfd, "AT+QGPSLOC=1\r\n", 14); //配置3
    if (ret < 0)
    {
        close(ttyfd);
        return "write error";
    }
    usleep(500 * 1000);
    ret = read(ttyfd, readbuf, sizeof(readbuf));
    if (ret < 0 && errno != EAGAIN)
    {
        close(ttyfd);
        return "read error";
    }
    LOG(INFO) << "AT+QGPSLOC=1 readbuf =" << readbuf;
    memset(readbuf, 0, sizeof(readbuf));

    ret = write(ttyfd, "AT+QGPSLOC=0\r\n", 14); //配置3
    if (ret < 0)
    {
        close(ttyfd);
        return "write error";
    }
    usleep(500 * 1000);
    ret = read(ttyfd, readbuf, sizeof(readbuf));
    if (ret < 0 && errno != EAGAIN)
    {
        close(ttyfd);
        return "read error";
    }
    LOG(INFO) << "AT+QGPSLOC=0 readbuf =" << readbuf;
    memset(readbuf, 0, sizeof(readbuf));

    return sim_num;
}