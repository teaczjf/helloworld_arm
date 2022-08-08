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
#include "zini.h"
int ttyfd;
int stdout_changed = 0;
int tty_changed = 0;
struct termios stdout_tio;
struct termios tty_tio;
using namespace std;

string get_sim_num();
int set_blue_alarm_led(int value);
string get_gps();

double x_pi = 3.14159265358979324 * 3000.0 / 180.0;
double pi = 3.1415926535897932384626; //π
double a = 6378245.0;                 // 长半轴
double ee = 0.00669342162296594323;   // 扁率
double transformlat(double lng, double lat)
{
    double ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * sqrt(fabs(lng));
    ret += (20.0 * sin(6.0 * lng * pi) + 20.0 * sin(2.0 * lng * pi)) * 2.0 / 3.0;
    ret += (20.0 * sin(lat * pi) + 40.0 * sin(lat / 3.0 * pi)) * 2.0 / 3.0;
    ret += (160.0 * sin(lat / 12.0 * pi) + 320 * sin(lat * pi / 30.0)) * 2.0 / 3.0;
    return ret;
}

double transformlng(double lng, double lat)
{
    double ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * sqrt(fabs(lng));
    ret += (20.0 * sin(6.0 * lng * pi) + 20.0 * sin(2.0 * lng * pi)) * 2.0 / 3.0;
    ret += (20.0 * sin(lng * pi) + 40.0 * sin(lng / 3.0 * pi)) * 2.0 / 3.0;
    ret += (150.0 * sin(lng / 12.0 * pi) + 300.0 * sin(lng / 30.0 * pi)) * 2.0 / 3.0;
    return ret;
}

/* ********************************************************/
/**
 * @brief  split by space
 * 按空格(pattern)分割cli输入的命令 获取操作内容与参数
 * 字符串处理
 * @param[in]  cmd_str
 * @param[in]  pattern
 * @param[in]  param_list
 */
/* ********************************************************/
void split(const char *cmd_str, char pattern, vector<string> &param_list)
{
    char tempstr[100];
    memset(tempstr, 0, 100);

    int cmd_str_index = 0;
    int param_str_index = 0;
    for (cmd_str_index = 0; cmd_str_index < strlen(cmd_str); cmd_str_index++)
    {
        if (cmd_str[cmd_str_index] != pattern)
        {
            tempstr[param_str_index++] = cmd_str[cmd_str_index];
        }
        else
        {
            /// string is valid
            if (strlen(tempstr) > 0)
            {
                param_list.push_back(tempstr);
                memset(tempstr, 0, 100);
                param_str_index = 0;
            }
        }
    }

    /// last param
    if (cmd_str[cmd_str_index] != pattern)
    {
        param_list.push_back(tempstr);
    }
}

//结构定义,这个是配置文件的定义
typedef struct
{
    char ItemName[50];
    char ItemContent[500];
} CConfItem, *LPCConfItem;

#define error_str(s, ...) \
    std::cout << s << "，文件是: " << __FILE__ << ",行号是: " << __LINE__ << std::endl;

std::map<std::string, std::vector<LPCConfItem>> m_ConfigItemList; //存储配置信息的列表
bool Load(const char *pconfName);                                 //装载配置文件

bool Load(const char *pconfName)
{
    FILE *fp;

#if _WIN32
    int err = fopen_s(&fp, pconfName, "r");
    if (err != NULL)
    {
        error_str("读取配置文件失败") return false;
    }
#else
    fp = fopen(pconfName, "r");
    if (fp == NULL)
    {
        error_str("读取配置文件失败") return false;
    }
#endif

    char linebuf[501]; //每一行的配置文件读出来放在这里
    char m_str[100];   // Map的Ky

    std::vector<LPCConfItem> local_list; //存储配置信息的列表

    while (!feof(fp)) //检查文件是否结束 ，没有结束则条件成立
    {

        if (fgets(linebuf, 500, fp) == NULL) //从文件中读数据，每次读一行，一行最多不要超过500个字符
            continue;

        if (linebuf[0] == 0)
            continue;

        //处理注释
        if (*linebuf == ';' || *linebuf == ' ' || *linebuf == '#' || *linebuf == '\t' || *linebuf == '\n')
            continue;

    lblprocstring:

        //屁股后边若有换行，回车，空格等都截取掉
        if (strlen(linebuf) > 0)
        {
            if (linebuf[strlen(linebuf) - 1] == 10 || linebuf[strlen(linebuf) - 1] == 13 || linebuf[strlen(linebuf) - 1] == 32)
            {
                linebuf[strlen(linebuf) - 1] = 0;
                goto lblprocstring;
            }
        }
        //判断是否有找到[
        if (*linebuf == '[')
        {

            if (!local_list.empty())
            {
                m_ConfigItemList.insert(pair<std::string, std::vector<LPCConfItem>>(m_str, local_list)); //插入
                memset(m_str, 0, sizeof(m_str));
                local_list.clear();
            }
            char *ptemp = strchr(linebuf, ']'); //如果找到 '[' , 那么寻找 ']'
            if (ptemp == NULL)
            {
                error_str("解析失败，没有找到 ']'") return false;
            }
#if _WIN32
            strncpy_s(m_str, sizeof(std::string), linebuf + 1, (size_t)(ptemp - linebuf) - 1);
#else
            // strncpy(p_confitem->ItemName, linebuf, (size_t)(ptmp - linebuf)); //不带 '\0'
            strncpy(m_str, linebuf, (size_t)(ptemp - linebuf + 1));           //不带 '\0'
#endif
            continue;
        }

        //然后寻找=
        char *ptmp = strchr(linebuf, '=');
        if (ptmp != NULL)
        {

            LPCConfItem p_confitem = new CConfItem;   //注意前边类型带LP，后边new这里的类型不带
            memset(p_confitem, 0, sizeof(CConfItem)); //指针滞空
#if _WIN32
            strncpy_s(p_confitem->ItemName, sizeof(p_confitem->ItemName), linebuf, (size_t)(ptmp - linebuf)); //不带 '\0'
            strcpy_s(p_confitem->ItemContent, ptmp + 1);                                                      //带 '\0'
#else
            strncpy(p_confitem->ItemName, linebuf, (size_t)(ptmp - linebuf)); //不带 '\0'
            strcpy(p_confitem->ItemContent, ptmp + 1);
#endif
            local_list.push_back(p_confitem);
        }
    }

    for (auto &a : m_ConfigItemList)
    {

        cout << a.first << endl;
        for (auto &b : a.second)
        {
            cout << b->ItemName << endl;
            cout << b->ItemContent << endl;
        }
    }

    fclose(fp);
    return true;
}

int main(int args, const char *argv[])
{
    // if (args != 2)
    // {
    //     fprintf(stderr, "Usage: ./sendat ttyUSB\n");
    //     exit(EXIT_FAILURE);
    // }
    easylogginginit(); //日志初始化
                       // string Longitude_NUM = "12021.9479"; //经度 12021.9479E
                       // // string Longitude_fanwei = param_list[1].substr(9, 1); //经度12021.9479E

    // string latitude_NUM = "3018.3451"; //纬度  3018.3451N
    // // string latitude_fanwei = param_list[2].substr(10, 1); //纬度  3018.3451N
    // double lon_num = stod(Longitude_NUM) / 100;
    // double lat_num = stod(latitude_NUM) / 100;

    // vector<string> lon_param_list;
    // vector<string> lat_param_list;

    // // string str = readbuf;

    // split(to_string(lon_num).data(), '.', lon_param_list);
    // split(to_string(lat_num).data(), '.', lat_param_list);

    // double lon_xs = stod(lon_param_list[1]) / 60;
    // double lat_xs = stod(lat_param_list[1]) / 60;
    // // LOG(INFO) << "lon_xs " << to_string(lon_xs);
    // // LOG(INFO) << "lat_xs " << to_string(lat_xs);
    // while (lon_xs > 1)
    // {
    //     lon_xs = lon_xs / 10;
    // }
    // while (lat_xs > 1)
    // {
    //     lat_xs = lat_xs / 10;
    // }
    // // LOG(INFO) << "lon_xs " << to_string(lon_xs);
    // // LOG(INFO) << "lat_xs " << to_string(lat_xs);

    // lon_num = stod(lon_param_list[0]) + lon_xs;
    // lat_num = stod(lat_param_list[0]) + lat_xs;

    // // LOG(INFO) << "lon_num =" << to_string(lon_num);
    // // LOG(INFO) << "lat_num =" << to_string(lat_num);
    // // LOG(INFO) << "latitude_fanwei =" << latitude_fanwei;

    // // wgs84_to_gcj02
    // double lat = lat_num;
    // double lng = lon_num;
    // double dlat = transformlat((lng - 105.0), (lat - 35.0));
    // double dlng = transformlng((lng - 105.0), (lat - 35.0));
    // double radlat = lat / 180.0 * pi;
    // double magic = sin(radlat);
    // magic = 1 - ee * magic * magic;
    // double sqrtmagic = sqrt(magic);
    // dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * pi);
    // dlng = (dlng * 180.0) / (a / sqrtmagic * cos(radlat) * pi);
    // double mglat = lat + dlat;
    // double mglng = lng + dlng;
    // LOG(INFO) << "mglng =" << to_string(mglng);
    // LOG(INFO) << "mglat =" << to_string(mglat);

    // // gcj02_to_bd09
    // double bd_lon_num;
    // double bd_lat_num;
    // double z = sqrt(mglng * mglng + mglat * mglat) + 0.00002 * sin(mglat * x_pi);
    // double theta = atan2(mglat, mglng) + 0.000003 * cos(mglng * x_pi);
    // bd_lon_num = z * cos(theta) + 0.0065;
    // bd_lat_num = z * sin(theta) + 0.006;
    // LOG(INFO) << "=====================================";
    // LOG(INFO) << "bd_lon_num =" << to_string(bd_lon_num);
    // LOG(INFO) << "bd_lat_num =" << to_string(bd_lat_num);
    // while (1)
    // {
    // get_gps();
    // sleep(1);
    // LOG(INFO) << "helloworld";
    // }

    // Load("/home/lkt/gitlab/HelloWorld_ARM/123.ini");
    if (!ZIni::writeString("Sect1", "Key1_1", "hello", "test.ini") ||
        !ZIni::writeString("Sect1", "Key1_2", "world", "test.ini") ||
        !ZIni::writeInt("Sect2", "Key2_1", 1984, "test.ini") ||
        !ZIni::writeDouble("Sect3", "Key3_1", 3.14159, "test.ini") ||
        !ZIni::writeDouble("Sect3", "Key3_2", 2.71828, "test.ini"))
    {
        cout << "write failed!" << endl;
    }
    cout << ZIni::readString("Sect1", "Key1_1", "nothing", "test.ini") << endl;
    cout << ZIni::readString("Sect1", "Key1_2", "nothing", "test.ini") << endl;
    cout << ZIni::readString("Sect1", "Key1_3", "nothing", "test.ini") << endl;
    cout << ZIni::readInt("Sect2", "Key2_1", 0, "test.ini") << endl;
    cout << ZIni::readInt("Sect2", "Key2_2", 0, "test.ini") << endl;
    cout << ZIni::readDouble("Sect3", "Key3_1", 0.0, "test.ini") << endl;
    cout << ZIni::readDouble("Sect3", "Key3_2", 0.0, "test.ini") << endl;
    cout << ZIni::readDouble("Sect3", "Key3_3", 0.0, "test.ini") << endl;
    cout << ZIni::readString("Sect4", "Key4_1", "nothing", "test.ini") << endl;
    cout << ZIni::readInt("Sect4", "Key4_2", 0, "test.ini") << endl;
    cout << ZIni::readDouble("Sect4", "Key4_3", 0.0, "test.ini") << endl;
    return 0;

    // get_sim_num();
    // set_blue_alarm_led(1);
    // get_gps();
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
    int time = 100 * 1000;
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
    usleep(time);
    ret = read(ttyfd, readbuf, sizeof(readbuf));
    if (ret < 0 && errno != EAGAIN)
    {
        close(ttyfd);
        return "read error";
    }
    // LOG(INFO) << "ATE0 readbuf =" << readbuf;
    memset(readbuf, 0, sizeof(readbuf));

    ret = write(ttyfd, "at+qgps\r\n", 9); //打开GPS
    if (ret < 0)
    {
        close(ttyfd);
        return "write error";
    }
    usleep(time);
    ret = read(ttyfd, readbuf, sizeof(readbuf));
    if (ret < 0 && errno != EAGAIN)
    {
        close(ttyfd);
        return "read error";
    }
    // LOG(INFO) << "at+qgps readbuf =" << readbuf;
    memset(readbuf, 0, sizeof(readbuf));

    ret = write(ttyfd, "AT+QGPSCFG=\"gnssconfig\",0\r\n", 27); //配置1
    if (ret < 0)
    {
        close(ttyfd);
        return "write error";
    }
    usleep(time);
    ret = read(ttyfd, readbuf, sizeof(readbuf));
    if (ret < 0 && errno != EAGAIN)
    {
        close(ttyfd);
        return "read error";
    }
    // LOG(INFO) << "AT+QGPSCFG=\"gnssconfig\",0 readbuf =" << readbuf;
    memset(readbuf, 0, sizeof(readbuf));

    ret = write(ttyfd, "at+qgps=1\r\n", 11); //配置2
    if (ret < 0)
    {
        close(ttyfd);
        return "write error";
    }
    usleep(time);
    ret = read(ttyfd, readbuf, sizeof(readbuf));
    if (ret < 0 && errno != EAGAIN)
    {
        close(ttyfd);
        return "read error";
    }
    // LOG(INFO) << "at+qgps=1 readbuf =" << readbuf;
    memset(readbuf, 0, sizeof(readbuf));

    ret = write(ttyfd, "AT+QGPSLOC=0\r\n", 14); //配置3
    if (ret < 0)
    {
        close(ttyfd);
        return "write error";
    }
    usleep(time);
    ret = read(ttyfd, readbuf, sizeof(readbuf));
    if (ret < 0 && errno != EAGAIN)
    {
        close(ttyfd);
        return "read error";
    }
    // LOG(INFO) << "AT+QGPSLOC=0 readbuf =" << readbuf;
    memset(readbuf, 0, sizeof(readbuf));

    ret = write(ttyfd, "AT+QGPSLOC=1\r\n", 14); //配置3
    if (ret < 0)
    {
        close(ttyfd);
        return "write error";
    }
    usleep(time);
    ret = read(ttyfd, readbuf, sizeof(readbuf));
    if (ret < 0 && errno != EAGAIN)
    {
        close(ttyfd);
        return "read error";
    }
    // LOG(INFO) << "AT+QGPSLOC=1 readbuf =" << readbuf;
    memset(readbuf, 0, sizeof(readbuf));

    ret = write(ttyfd, "AT+QGPSLOC=0\r\n", 14); //配置3
    if (ret < 0)
    {
        close(ttyfd);
        return "write error";
    }
    usleep(time);
    ret = read(ttyfd, readbuf, sizeof(readbuf));
    if (ret < 0 && errno != EAGAIN)
    {
        close(ttyfd);
        return "read error";
    }
    // LOG(INFO) << "AT+QGPSLOC=0 readbuf =" << readbuf;
    close(ttyfd);

    vector<string> param_list;
    // string str = readbuf;
    split(readbuf, ',', param_list);
    if (param_list.size() < 2)
    {
        // LOG(INFO) << "no gps sig";
        string ret_str = " no signal";
        return ret_str;
    }
    else
    {
        // LOG(INFO) << "param_list[1] =" << param_list[1];
        // LOG(INFO) << "param_list[2] =" << param_list[2];
        string Longitude_NUM = param_list[1].substr(0, 9);    //纬度 3018.3451N
        string Longitude_fanwei = param_list[1].substr(9, 1); //纬度 3018.3451N

        string latitude_NUM = param_list[2].substr(0, 10);    //经度 12021.9479E
        string latitude_fanwei = param_list[2].substr(10, 1); //经度 12021.9479E

        double lon_num = stod(Longitude_NUM) / 100;
        double lat_num = stod(latitude_NUM) / 100;

        vector<string> lon_param_list;
        vector<string> lat_param_list;

        // string str = readbuf;

        split(to_string(lon_num).data(), '.', lon_param_list);
        split(to_string(lat_num).data(), '.', lat_param_list);

        double lon_xs = stod(lon_param_list[1]) / 60;
        double lat_xs = stod(lat_param_list[1]) / 60;
        // LOG(INFO) << "lon_xs " << to_string(lon_xs);
        // LOG(INFO) << "lat_xs " << to_string(lat_xs);
        while (lon_xs > 1)
        {
            lon_xs = lon_xs / 10;
        }
        while (lat_xs > 1)
        {
            lat_xs = lat_xs / 10;
        }
        // LOG(INFO) << "lon_xs " << to_string(lon_xs);
        // LOG(INFO) << "lat_xs " << to_string(lat_xs);

        lon_num = stod(lon_param_list[0]) + lon_xs;
        lat_num = stod(lat_param_list[0]) + lat_xs;

        // LOG(INFO) << "lon_num =" << to_string(lon_num);
        // LOG(INFO) << "lat_num =" << to_string(lat_num);
        // LOG(INFO) << "latitude_fanwei =" << latitude_fanwei;

        // wgs84_to_gcj02
        double lat = lat_num;
        double lng = lon_num;
        double dlat = transformlat((lng - 105.0), (lat - 35.0));
        double dlng = transformlng((lng - 105.0), (lat - 35.0));
        double radlat = lat / 180.0 * pi;
        double magic = sin(radlat);
        magic = 1 - ee * magic * magic;
        double sqrtmagic = sqrt(magic);
        dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * pi);
        dlng = (dlng * 180.0) / (a / sqrtmagic * cos(radlat) * pi);
        double mglat = lat + dlat;
        double mglng = lng + dlng;
        // LOG(INFO) << "mglng =" << to_string(mglng);
        // LOG(INFO) << "mglat =" << to_string(mglat);

        // gcj02_to_bd09
        double bd_lon_num;
        double bd_lat_num;
        double z = sqrt(mglng * mglng + mglat * mglat) + 0.00002 * sin(mglat * x_pi);
        double theta = atan2(mglat, mglng) + 0.000003 * cos(mglng * x_pi);
        bd_lon_num = z * cos(theta) + 0.0065;
        bd_lat_num = z * sin(theta) + 0.006;
        // LOG(INFO) << "=====================================";
        // LOG(INFO) << "bd_lon_num =" << to_string(bd_lon_num);
        // LOG(INFO) << "bd_lat_num =" << to_string(bd_lat_num);

        string addr = to_string(bd_lon_num) + " " + Longitude_fanwei + " " + to_string(bd_lat_num) + " " + latitude_fanwei;
        LOG(INFO) << addr;
        return addr;
    }
}

/*
        sleep(1);
        string slat = "3018.3451";
        string slng = "12021.9479";
        double lat;
        double lng;
        lat = stod(slat) / 100;
        lng = stod(slng) / 100;
        cout << to_string(lat) << endl;
        cout << to_string(lng) << endl;
        */