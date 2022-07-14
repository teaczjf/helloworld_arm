#include <stdio.h>
#include "test.h"
#include "test2.h"
#include <iostream>
#include "my_easylogging.h"
// #define HDDD
int main()
{
    easylogginginit(); //日志初始化
    log_test();        //日志自测函数   里面可以看细节的用法

    printf("Hello world\n\r");
    test1();
    test2();
#if 0
    while (1)
    {
        LOG(INFO) << "》》》》》》》》》》打印测试11111《《《《《《《《";
        // sleep(1);//秒
        usleep(1 * 1000); //微秒
        // printf("HDDD not def\n\r");
        // std::cout << "An exception occurred. Exception Nr. " << std::endl;
        LOG(INFO) << "》》》》》》》》》》打印测试《《《《《《《《";

        LOG(INFO) << "》》》》》》》》》》打印测试2222《《《《《《《《";
    }
#endif
    try
    {
        throw 20;
    }
    catch (int e)
    {
        std::cout << "An exception occurred. Exception Nr. " << e << '\n';
    }

#ifdef HDDD
    printf("HDDD def\n\r");
#else
    printf("HDDD not def\n\r");
    // std::cout << HDDD << std::endl;
#endif // #

    return 0;
}
