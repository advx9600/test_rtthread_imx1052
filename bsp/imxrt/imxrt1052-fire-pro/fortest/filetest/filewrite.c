#include <rtthread.h>
#include <sys/socket.h> /* 使用BSD socket，需要包含socket.h头文件 */
#include <netdb.h>
#include <string.h>
#include <finsh.h>
#include <dfs_posix.h>

static int filewrite(int argc, char** argv)
{
    if (argc < 3)
    {
        rt_kprintf("Usage:fileread filename, size");
        return 0;
    }

    int fd = open(argv[1], O_WRONLY | O_CREAT);
		int totalsize = atoi(argv[2]);
    if (fd < 0)
    {
        printf("open file failed\n");
        return -1;
    }

    int writelen = 0;
    int starttime = rt_tick_get();

    while(writelen < totalsize)
    {
        char buf[1024];
        int i;

        for (i = 0; i < sizeof(buf); i++)
        {
            buf[i] = i;
        }

        int curlen = write(fd, buf, sizeof(buf));

        if (curlen != sizeof(buf))
        {
            printf("write file failed :%d\n", curlen);
            close(fd);
            return -1;
        }

        writelen += sizeof(buf);
    }

    close(fd);
		int time_cast = (rt_tick_get() - starttime);
    printf("size:%d,use %d.%03dS second\n", totalsize, time_cast/ RT_TICK_PER_SECOND ,time_cast % RT_TICK_PER_SECOND / ((RT_TICK_PER_SECOND * 1 + 999) / 1000));
}

MSH_CMD_EXPORT(filewrite,);
