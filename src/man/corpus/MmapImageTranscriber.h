#ifndef MMAPIMAGE_TRANSCRIBER_H
#define MMAPIMAGE_TRANSCRIBER_H

#include "almemoryproxy.h"
#include "albroker.h"
#include "alptr.h"
#include "alloggerproxy.h"

#include "ThreadedImageTranscriber.h"
#include "synchro.h"
#include "linux/videodev2.h"
#include "bn/i2c/i2c-dev.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>
#include <malloc.h>

class MmapImageTranscriber : public ThreadedImageTranscriber {
 public:
    MmapImageTranscriber(boost::shared_ptr<Synchro> synchro,
                       boost::shared_ptr<Sensors> s,
                       AL::ALPtr<AL::ALBroker> broker);
    virtual ~MmapImageTranscriber();

 private:
    MmapImageTranscriber(const MmapImageTranscriber &other);
    void operator= (const MmapImageTranscriber &other);

 public:

    int start();
    void run();
    void stop();
    void releaseImage();

 private: // helper methods

    void process_image(const void * p);
    void start_capturing();
    void stop_capturing();
    int read_frame();
    void init_read(unsigned int i);
    void init_mmap();
    void init_userp(unsigned int buf_size);
    void init_device();
    void uninit_device();
    bool open_device();
    void close_device();
    bool open_i2c();
    void close_i2c();
    void init_cameras();
    void set_camera_bottom();
    void set_camera_top();
    void errno_exit(const char * s);
    int xioctl (int fd, int request, void * arg);

 private: // member variables
    // Interfaces/Proxies to robot

    // stream descriptor for video camera
    AL::ALPtr<AL::ALLoggerProxy> log;
    bool camera_active;

    struct buffer {
        void * start;
        size_t length;
    };

    char * dev_name;
    int fd;
    int i2cBus;
    buffer * buffers;
    unsigned int n_buffers;

    typedef enum {
        IO_METHOD_READ,
        IO_METHOD_MMAP,
        IO_METHOD_USERPTR,
    } io_method;

    static const io_method io = IO_METHOD_MMAP;

};

#endif
