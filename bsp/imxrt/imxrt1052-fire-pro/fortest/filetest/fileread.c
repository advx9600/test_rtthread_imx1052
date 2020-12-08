#include <rtthread.h>
#include <sys/socket.h> /* 使用BSD socket，需要包含socket.h头文件 */
#include <netdb.h>
#include <string.h>
#include <finsh.h>
#include <dfs_posix.h>

static int fileread(int argc, char** argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage:fileread filename, size");
        return 0;
    }

    int fd = open(argv[1], O_RDONLY);

    if (fd < 0)
    {
        printf("open file failed\n");
        return -1;
    }

    int writelen = 0;
    int starttime = rt_tick_get();
    int totalsize = 0;

    while(1)
    {
        char buf[1024];
        int i;

        int curlen = read(fd, buf, sizeof(buf));

        if (curlen > 0)
        {
            totalsize += curlen;
        }
        else if (curlen < 0)
        {
            printf("write file failed :%d\n", curlen);
            close(fd);
            return -1;
        }
        else if (curlen == 0)
        {
            break;
        }
    }

    close(fd);
    int time_cast = (rt_tick_get() - starttime);
    printf("size:%d,use %d.%03dS second\n", totalsize, time_cast / RT_TICK_PER_SECOND, time_cast % RT_TICK_PER_SECOND / ((RT_TICK_PER_SECOND * 1 + 999) / 1000));
}

MSH_CMD_EXPORT(fileread,);
