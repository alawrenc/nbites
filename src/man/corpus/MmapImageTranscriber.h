#ifndef MMAPIMAGE_TRANSCRIBER_H
#define MMAPIMAGE_TRANSCRIBER_H

#include "almemoryproxy.h"
#include "albroker.h"
#include "alptr.h"
#include "alloggerproxy.h"

#include "ThreadedImageTranscriber.h"
#include "synchro.h"
#include <linux/videodev2.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>

class ALImageTranscriber : public ThreadedImageTranscriber {
 public:
    ALImageTranscriber(boost::shared_ptr<Synchro> synchro,
                       boost::shared_ptr<Sensors> s,
                       AL::ALPtr<AL::ALBroker> broker);
    virtual ~ALImageTranscriber();

 private:
    ALImageTranscriber(const ALImageTranscriber &other);
    void operator= (const ALImageTranscriber &other);

 public:

    int start();
    void run();
    void stop();
    void releaseImage();

 private: // helper methods
    bool registerCamera(AL::ALPtr<AL::ALBroker> broker);
    void initCameraSettings(int whichCam);
    void waitForImage();
    void testV4L2SetValues();

 private: // member variables
    // Interfaces/Proxies to robot

    int fd;
    AL::ALPtr<AL::ALLoggerProxy> log;

    std::string lem_name;

    bool camera_active;

    // Keep a local copy of the image because accessing the one from NaoQi is
    // from the kernel and thus very slow.
    unsigned char *image;
};

#endif
