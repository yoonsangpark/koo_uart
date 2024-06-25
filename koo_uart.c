#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>

#define DEV_TTY "/dev/ttyS0"
#define BUF_SIZE 256

int main(int argc, char *argv[])
{
    int i;
    int fd;
    int ret;    
    char tx_buf[] = "ABCD\n";
    char rx_buf[BUF_SIZE] = "";
    struct termios options;

    /* open uart */
    fd = open(DEV_TTY, O_RDWR|O_NOCTTY);
    if (fd < 0) {
        printf("ERROR open %s ret=%d\n", DEV_TTY, fd);
        return -1;
    }
    /* configure uart */
    tcgetattr(fd, &options);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cc[VTIME] = 10; // read timeout 10*100ms
    options.c_cc[VMIN]  = 0;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(ICRNL | IXON);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    options.c_cflag |= (CLOCAL | CREAD);
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);

    //while (1) {

        printf(">> TX\n");
        /* write uart */
        ret = write(fd, tx_buf, sizeof(tx_buf));
        if (ret != sizeof(tx_buf))
            printf("ERROR write ret=%d\n", ret);

        printf(">> RX\n");
        /* read uart */
        while ((ret = read(fd, rx_buf, BUF_SIZE-1)) > 0) {
            printf("ret=%d\n", ret);
            //puts(rx_buf);
            for(i=0; i< ret; i++)
              printf("%X ", rx_buf[i]);

            printf("\n");
            memset(rx_buf, 0, ret);
        }        
    //}

    /* close uart */
    close(fd);

    return 0;
}
