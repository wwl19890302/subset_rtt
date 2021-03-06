#ifndef PIPE_H__
#define PIPE_H__

/**
 * Pipe Device
 */
#include <rtthread.h>
#include <rtdevice.h>

#if defined(RT_USING_POSIX)

#ifndef RT_PIPE_BUFSZ
#define PIPE_BUFSZ    512
#else
#define PIPE_BUFSZ    RT_PIPE_BUFSZ
#endif

struct rt_pipe_device
{
    struct rt_device parent;

    /* ring buffer in pipe device */
    struct rt_ringbuffer *fifo;

    rt_uint8_t readers;
    rt_uint8_t writers;

    rt_wqueue_t reader_queue;
    rt_wqueue_t writer_queue;

    struct rt_mutex lock;
};
typedef struct rt_pipe_device rt_pipe_t;

rt_pipe_t *rt_pipe_create(const char *name);

#endif /* RT_USING_POSIX */
#endif /* PIPE_H__ */
