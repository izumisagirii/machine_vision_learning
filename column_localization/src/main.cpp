#include <signal.h>
#include "column_identify.h"
hitcrt::column_identify detector;
void stopProgram(int sig)
{
    detector.stop();
}
int main(int argc, char *argv[]) {
    detector.run();
    signal(SIGQUIT, stopProgram);
    signal(SIGTSTP, stopProgram);
    pause();
    // system("stty -icanon");//关闭缓冲区，输入字符无需回车直接接受
    // while (key != 'q') {
    //     key = getchar();
    //     usleep(10000);
    // }
    return 0;
}
