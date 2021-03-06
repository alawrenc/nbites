
// This file is part of Man, a robotic perception, locomotion, and
// team strategy application created by the Northern Bites RoboCup
// team of Bowdoin College in Brunswick, Maine, for the Aldebaran
// Nao robot.
//
// Man is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Man is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser Public License for more details.
//
// You should have received a copy of the GNU General Public License
// and the GNU Lesser Public License along with Man.  If not, see
// <http://www.gnu.org/licenses/>.

#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstdlib>
#include <sys/stat.h>

using namespace std;

#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

#include <boost/lambda/lambda.hpp>
using namespace boost::lambda;

#include "Sensors.h"

#include "corpusconfig.h"
#include "NBMath.h"
#include "Kinematics.h"
using namespace Kinematics;

// static base image array, so we don't crash on image access if the setImage()
// method is never called
static uint16_t global_image[NAO_IMAGE_BYTE_SIZE];

//
// C++ Sensors class methods
//
int Sensors::saved_frames = 0;

Sensors::Sensors ()
    : bodyAngles(NUM_ACTUATORS), visionBodyAngles(NUM_ACTUATORS),
      motionBodyAngles(NUM_ACTUATORS),
      bodyAnglesError(NUM_ACTUATORS),
      bodyTemperatures(NUM_ACTUATORS,0.0f),
      leftFootFSR(0.0f, 0.0f, 0.0f, 0.0f),
      rightFootFSR(leftFootFSR),
      leftFootBumper(0.0f, 0.0f),
      rightFootBumper(0.0f, 0.0f),
      inertial(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f),
      ultraSoundDistanceLeft(0.0f), ultraSoundDistanceRight(0.0f),
      yImage(&global_image[0]), uvImage(&global_image[0]),
      colorImage(reinterpret_cast<uint8_t*>(&global_image[0])),
      naoImage(reinterpret_cast<uint8_t*>(&global_image[0])),
      supportFoot(LEFT_SUPPORT),
      unfilteredInertial(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f),
      chestButton(0.0f),batteryCharge(0.0f),batteryCurrent(0.0f),
      FRM_FOLDER("/home/nao/naoqi/frames"),
	  saving_frames_on(false)
{
    pthread_mutex_init(&angles_mutex, NULL);
    pthread_mutex_init(&vision_angles_mutex, NULL);
    pthread_mutex_init(&motion_angles_mutex, NULL);
    pthread_mutex_init(&temperatures_mutex, NULL);
    pthread_mutex_init(&errors_mutex, NULL);
    pthread_mutex_init(&fsr_mutex, NULL);
    pthread_mutex_init(&button_mutex, NULL);
    pthread_mutex_init(&inertial_mutex, NULL);
    pthread_mutex_init(&unfiltered_inertial_mutex, NULL);
    pthread_mutex_init(&ultra_sound_mutex, NULL);
    pthread_mutex_init(&support_foot_mutex, NULL);
    pthread_mutex_init(&battery_mutex, NULL);
#ifdef USE_SENSORS_IMAGE_LOCKING
    pthread_mutex_init(&image_mutex, NULL);
#endif

    // THIS IS AN OCTAL NUMBER, must start with 0
    mkdir(FRM_FOLDER.c_str(), 0755); // permissions: u+rwx, og+rx
}

Sensors::~Sensors ()
{
    pthread_mutex_destroy(&angles_mutex);
    pthread_mutex_destroy(&vision_angles_mutex);
    pthread_mutex_destroy(&motion_angles_mutex);
    pthread_mutex_destroy(&temperatures_mutex);
    pthread_mutex_destroy(&errors_mutex);
    pthread_mutex_destroy(&fsr_mutex);
    pthread_mutex_destroy(&button_mutex);
    pthread_mutex_destroy(&inertial_mutex);
    pthread_mutex_destroy(&unfiltered_inertial_mutex);
    pthread_mutex_destroy(&ultra_sound_mutex);
    pthread_mutex_destroy(&support_foot_mutex);
    pthread_mutex_destroy(&battery_mutex);
#ifdef USE_SENSORS_IMAGE_LOCKING
    pthread_mutex_destroy(&image_mutex);
#endif
}

const vector<float> Sensors::getBodyAngles () const
{
    pthread_mutex_lock (&angles_mutex);

    vector<float> vec(bodyAngles);

    pthread_mutex_unlock (&angles_mutex);

    return vec;
}

const vector<float> Sensors::getHeadAngles () const
{
    pthread_mutex_lock (&vision_angles_mutex);
    vector<float> vec(2);
    vec[0] = visionBodyAngles[0];
    vec[1] = visionBodyAngles[1];
    pthread_mutex_unlock (&vision_angles_mutex);

    return vec;
}

const vector<float> Sensors::getBodyAngles_degs () const
{
    pthread_mutex_lock (&angles_mutex);

    vector<float> vec(bodyAngles);

    pthread_mutex_unlock (&angles_mutex);

    // Convert the angles from radians to degrees
    std::for_each(vec.begin(), vec.end(), _1 = _1 * TO_DEG);

    return vec;
}

const vector<float> Sensors::getVisionBodyAngles() const
{
    pthread_mutex_lock (&vision_angles_mutex);

    vector<float> vec(visionBodyAngles);

    pthread_mutex_unlock (&vision_angles_mutex);

    return vec;
}

const vector<float> Sensors::getMotionBodyAngles_degs () const
{
    pthread_mutex_lock (&motion_angles_mutex);

    vector<float> vec(motionBodyAngles);

    pthread_mutex_unlock (&motion_angles_mutex);

    // Convert the angles from radians to degrees
    std::for_each(vec.begin(), vec.end(), _1 = _1 * TO_DEG);

    return vec;
}

const vector<float> Sensors::getMotionBodyAngles() const
{
    pthread_mutex_lock (&motion_angles_mutex);

    vector<float> vec(motionBodyAngles);

    pthread_mutex_unlock (&motion_angles_mutex);

    return vec;
}

const vector<float> Sensors::getBodyTemperatures() const
{
    pthread_mutex_lock (&temperatures_mutex);

    vector<float> vec(bodyTemperatures);

    pthread_mutex_unlock (&temperatures_mutex);

    return vec;
}

const float Sensors::getBodyAngle(const int index) const {
    pthread_mutex_lock (&angles_mutex);

    const float angle = bodyAngles[index];

    pthread_mutex_unlock (&angles_mutex);

    return angle;
}

const vector<float> Sensors::getBodyAngleErrors () const
{
    pthread_mutex_lock (&errors_mutex);

    vector<float> vec(bodyAnglesError);

    pthread_mutex_unlock (&errors_mutex);

    return vec;
}

const float Sensors::getBodyAngleError (int index) const
{
    pthread_mutex_lock (&errors_mutex);

    const float angleError = bodyAnglesError[index];

    pthread_mutex_unlock (&errors_mutex);

    return angleError;
}

const FSR Sensors::getLeftFootFSR () const
{
    pthread_mutex_lock (&fsr_mutex);

    const FSR left(leftFootFSR);

    pthread_mutex_unlock (&fsr_mutex);

    return left;
}

const FSR Sensors::getRightFootFSR () const
{
    pthread_mutex_lock (&fsr_mutex);

    const FSR right(rightFootFSR);

    pthread_mutex_unlock (&fsr_mutex);

    return right;
}

const FootBumper Sensors::getLeftFootBumper() const
{
    pthread_mutex_lock (&button_mutex);

    const FootBumper bumper = leftFootBumper;

    pthread_mutex_unlock (&button_mutex);

    return bumper;
}

const FootBumper Sensors::getRightFootBumper() const
{
    pthread_mutex_lock (&button_mutex);

    const FootBumper bumper = rightFootBumper;

    pthread_mutex_unlock (&button_mutex);

    return bumper;
}

const Inertial Sensors::getInertial () const
{
    pthread_mutex_lock (&inertial_mutex);

    const Inertial inert(inertial);

    pthread_mutex_unlock (&inertial_mutex);

    return inert;
}

const Inertial Sensors::getInertial_degs () const
{
    pthread_mutex_lock (&inertial_mutex);

    Inertial inert(inertial);

    pthread_mutex_unlock (&inertial_mutex);

    inert.angleX *= TO_DEG;
    inert.angleY *= TO_DEG;

    return inert;
}

const Inertial Sensors::getUnfilteredInertial () const
{
    pthread_mutex_lock (&unfiltered_inertial_mutex);

    const Inertial inert(unfilteredInertial);

    pthread_mutex_unlock (&unfiltered_inertial_mutex);

    return inert;
}

const float Sensors::getUltraSoundLeft () const
{
    pthread_mutex_lock (&ultra_sound_mutex);

    float dist = ultraSoundDistanceLeft;

    pthread_mutex_unlock (&ultra_sound_mutex);

    return dist;
}

const float Sensors::getUltraSoundRight () const
{
    pthread_mutex_lock (&ultra_sound_mutex);

    float dist = ultraSoundDistanceRight;

    pthread_mutex_unlock (&ultra_sound_mutex);

    return dist;
}


const float Sensors::getUltraSoundLeft_cm () const
{
    pthread_mutex_lock (&ultra_sound_mutex);

    float dist = ultraSoundDistanceLeft;

    pthread_mutex_unlock (&ultra_sound_mutex);

    return dist * M_TO_CM;
}

const float Sensors::getUltraSoundRight_cm () const
{
    pthread_mutex_lock (&ultra_sound_mutex);

    float dist = ultraSoundDistanceRight;

    pthread_mutex_unlock (&ultra_sound_mutex);

    return dist * M_TO_CM;
}

const SupportFoot Sensors::getSupportFoot () const
{
    pthread_mutex_lock (&support_foot_mutex);

    SupportFoot foot = supportFoot;

    pthread_mutex_unlock (&support_foot_mutex);

    return foot;
}

const float Sensors::getChestButton () const
{
    pthread_mutex_lock (&button_mutex);

    float button = chestButton;

    pthread_mutex_unlock (&button_mutex);

    return button;
}

const float Sensors::getBatteryCharge () const
{
    pthread_mutex_lock (&battery_mutex);

    const float charge = batteryCharge;

    pthread_mutex_unlock (&battery_mutex);

    return charge;
}
const float Sensors::getBatteryCurrent () const
{
    pthread_mutex_lock (&battery_mutex);

    const float current = batteryCurrent;

    pthread_mutex_unlock (&battery_mutex);

    return current;
}

const vector<float> Sensors::getAllSensors () const
{
    //All sensors sans unfiltered Inertials and Temperatures
    //and the chest button preses
    pthread_mutex_lock (&fsr_mutex);
    pthread_mutex_lock (&button_mutex);
    pthread_mutex_lock (&inertial_mutex);
    pthread_mutex_lock (&ultra_sound_mutex);
    pthread_mutex_lock (&support_foot_mutex);

    vector<float> allSensors;

    // write the FSR values
    allSensors += leftFootFSR.frontLeft, leftFootFSR.frontRight,
        leftFootFSR.rearLeft, leftFootFSR.rearRight,
        rightFootFSR.frontLeft, rightFootFSR.frontRight,
        rightFootFSR.rearLeft, rightFootFSR.rearRight;

    // write the foot bumper values
    allSensors += static_cast<float>(leftFootBumper.left),
        static_cast<float>(leftFootBumper.right),
        static_cast<float>(rightFootBumper.left),
        static_cast<float>(rightFootBumper.right);

    // write the accelerometers + gyros + filtered angleX and angleY
    allSensors += inertial.accX, inertial.accY, inertial.accZ,
        inertial.gyrX, inertial.gyrY,
        inertial.angleX, inertial.angleY;

    // write the ultrasound values
    allSensors += ultraSoundDistanceLeft;
    allSensors += ultraSoundDistanceRight;

    allSensors += supportFoot;

    pthread_mutex_unlock (&support_foot_mutex);
    pthread_mutex_unlock (&fsr_mutex);
    pthread_mutex_unlock (&button_mutex);
    pthread_mutex_unlock (&inertial_mutex);
    pthread_mutex_unlock (&ultra_sound_mutex);

    return allSensors;
}

void Sensors::setBodyAngles (const vector<float>& v)
{
    pthread_mutex_lock (&angles_mutex);

    bodyAngles = v;
    /*
      cout << "Body angles in sensors";
      for (int i = 0 ; i < 22; i++){
      cout <<  bodyAngles[i] << " ";

      }
      cout << endl;
    */
    pthread_mutex_unlock (&angles_mutex);
}

void Sensors::setVisionBodyAngles (const vector<float>& v)
{
    pthread_mutex_lock (&vision_angles_mutex);

    visionBodyAngles = v;

    pthread_mutex_unlock (&vision_angles_mutex);
}

void Sensors::setMotionBodyAngles (const vector<float>& v)
{
    pthread_mutex_lock (&motion_angles_mutex);

    motionBodyAngles = v;

    pthread_mutex_unlock (&motion_angles_mutex);
}

void Sensors::setBodyAngleErrors (const vector<float>& v)
{
    pthread_mutex_lock (&errors_mutex);

    bodyAnglesError = v;

    pthread_mutex_unlock (&errors_mutex);
}


void Sensors::setBodyTemperatures (const vector<float>& v)
{
    pthread_mutex_lock (&temperatures_mutex);

    bodyTemperatures = v;

    pthread_mutex_unlock (&temperatures_mutex);
}

void Sensors::setLeftFootFSR(const float frontLeft, const float frontRight,
                             const float rearLeft, const float rearRight)
{
    pthread_mutex_lock (&fsr_mutex);

    leftFootFSR = FSR(frontLeft, frontRight, rearLeft, rearRight);

    pthread_mutex_unlock (&fsr_mutex);
}

void Sensors::setRightFootFSR(const float frontLeft, const float frontRight,
                              const float rearLeft, const float rearRight)
{
    pthread_mutex_lock (&fsr_mutex);

    rightFootFSR = FSR(frontLeft, frontRight, rearLeft, rearRight);

    pthread_mutex_unlock (&fsr_mutex);
}

void Sensors::setFSR(const FSR &_leftFootFSR, const FSR &_rightFootFSR)
{
    pthread_mutex_lock (&fsr_mutex);

    leftFootFSR = _leftFootFSR;
    rightFootFSR = _rightFootFSR;

    pthread_mutex_unlock (&fsr_mutex);
}

void Sensors::setLeftFootBumper(const float left, const float right)
{
    pthread_mutex_lock (&button_mutex);

    leftFootBumper = FootBumper(left, right);

    pthread_mutex_unlock (&button_mutex);
}

void Sensors::setLeftFootBumper(const FootBumper& bumper)
{
    pthread_mutex_lock (&button_mutex);

    leftFootBumper = bumper;

    pthread_mutex_unlock (&button_mutex);
}

void Sensors::setRightFootBumper(const float left, const float right)
{
    pthread_mutex_lock (&button_mutex);

    rightFootBumper = FootBumper(left, right);

    pthread_mutex_unlock (&button_mutex);
}

void Sensors::setRightFootBumper(const FootBumper& bumper)
{
    pthread_mutex_lock (&button_mutex);

    rightFootBumper = bumper;

    pthread_mutex_unlock (&button_mutex);
}

void Sensors::setInertial(const float accX, const float accY, const float accZ,
                          const float gyrX, const float gyrY,
                          const float angleX, const float angleY)
{
    pthread_mutex_lock (&inertial_mutex);

    inertial = Inertial(accX, accY, accZ, gyrX, gyrY, angleX, angleY);

    pthread_mutex_unlock (&inertial_mutex);
}

void Sensors::setInertial (const Inertial &v)
{
    pthread_mutex_lock (&inertial_mutex);

    inertial = v;

    pthread_mutex_unlock (&inertial_mutex);
}

void Sensors::setUnfilteredInertial(const float accX, const float accY, const float accZ,
                                    const float gyrX, const float gyrY,
                                    const float angleX, const float angleY)
{
    pthread_mutex_lock (&unfiltered_inertial_mutex);

    unfilteredInertial = Inertial(accX, accY, accZ, gyrX, gyrY, angleX, angleY);

    pthread_mutex_unlock (&unfiltered_inertial_mutex);
}

void Sensors::setUnfilteredInertial (const Inertial &v)
{
    pthread_mutex_lock (&unfiltered_inertial_mutex);

    unfilteredInertial = v;

    pthread_mutex_unlock (&unfiltered_inertial_mutex);
}

void Sensors::setUltraSound (const float distLeft,
                             const float distRight)
{
    pthread_mutex_lock (&ultra_sound_mutex);

    ultraSoundDistanceLeft = distLeft;
    ultraSoundDistanceRight = distRight;

    pthread_mutex_unlock (&ultra_sound_mutex);
}

void Sensors::setSupportFoot (const SupportFoot _supportFoot)
{
    pthread_mutex_lock (&support_foot_mutex);

    supportFoot = _supportFoot;

    pthread_mutex_unlock (&support_foot_mutex);
}


/**
 * Sets the sensors which are updated on the motion frequency (every 20ms)
 */
void Sensors::setMotionSensors (const FSR &_leftFoot, const FSR &_rightFoot,
                                const float _chestButton,
                                const Inertial &_inertial,
                                const Inertial & _unfilteredInertial)
{
    pthread_mutex_lock(&button_mutex);
    pthread_mutex_lock(&fsr_mutex);
    pthread_mutex_lock(&inertial_mutex);
    pthread_mutex_lock(&unfiltered_inertial_mutex);

    leftFootFSR = _leftFoot;
    rightFootFSR = _rightFoot;
    chestButton = _chestButton;
    inertial = _inertial;
    unfilteredInertial = _unfilteredInertial;

    pthread_mutex_unlock(&unfiltered_inertial_mutex);
    pthread_mutex_unlock(&inertial_mutex);
    pthread_mutex_unlock(&fsr_mutex);
    pthread_mutex_unlock(&button_mutex);
}

/**
 * Sets the sensors which are updated on the vision frequency (every ?? ms)
 */
void Sensors::setVisionSensors (const FootBumper &_leftBumper,
                                const FootBumper &_rightBumper,
                                const float ultraSoundLeft,
                                const float ultraSoundRight,
                                const float bCharge, const float bCurrent)
{
    pthread_mutex_lock (&battery_mutex);
    pthread_mutex_lock (&button_mutex);
    pthread_mutex_lock(&ultra_sound_mutex);

    leftFootBumper = _leftBumper;
    rightFootBumper = _rightBumper;
    ultraSoundDistanceLeft = ultraSoundLeft;
    ultraSoundDistanceRight = ultraSoundRight;
    batteryCharge = bCharge;
    batteryCurrent = bCurrent;

    pthread_mutex_unlock(&ultra_sound_mutex);
    pthread_mutex_unlock (&button_mutex);
    pthread_mutex_unlock (&battery_mutex);

}

void Sensors::setAllSensors (vector<float> sensorValues) {
    //All sensors sans unfiltered Inertials and Temperatures
    //and the chest button preses
    pthread_mutex_lock (&fsr_mutex);
    pthread_mutex_lock (&button_mutex);
    pthread_mutex_lock (&inertial_mutex);
    pthread_mutex_lock (&ultra_sound_mutex);
    pthread_mutex_lock (&support_foot_mutex);

    // we have to be EXTRA careful about this order. If someone can think of
    // a better way to assign these so that it's checked at compile time
    // please do!
    leftFootFSR = FSR(sensorValues[0], sensorValues[1],
                      sensorValues[2], sensorValues[3]);
    rightFootFSR = FSR(sensorValues[4], sensorValues[5],
                       sensorValues[6], sensorValues[7]);

    leftFootBumper = FootBumper(sensorValues[8], sensorValues[9]);
    rightFootBumper = FootBumper(sensorValues[10], sensorValues[11]);

    inertial = Inertial(sensorValues[12], sensorValues[13], sensorValues[14],
                        sensorValues[15], sensorValues[16], // gyros
                        sensorValues[17], sensorValues[18]); // angleX/angleY

    ultraSoundDistanceLeft = sensorValues[19];
    ultraSoundDistanceRight = sensorValues[20];

    supportFoot = static_cast<SupportFoot>(
                                           static_cast<int>(sensorValues[21]));

    pthread_mutex_unlock (&support_foot_mutex);
    pthread_mutex_unlock (&ultra_sound_mutex);
    pthread_mutex_unlock (&inertial_mutex);
    pthread_mutex_unlock (&button_mutex);
    pthread_mutex_unlock (&fsr_mutex);
}


void Sensors::lockImage() const
{
#ifdef USE_SENSORS_IMAGE_LOCKING
    pthread_mutex_lock (&image_mutex);
#endif
}

void Sensors::releaseImage() const
{
#ifdef USE_SENSORS_IMAGE_LOCKING
    pthread_mutex_unlock (&image_mutex);
#endif
}

/**
 * Beware!!! I am toying with the possibility that a possible deadlock condition
 * exists here. If we somehow are able to lock body_angles, but unable to lock
 * vision_body_angles because someone else is using it, but that someone else
 * is also waiting on the body_angles mutex, we get a beautiful deadlock. I don't
 * think anyone else requires both mutexes locked at the same time. Beware
 * nonetheless.
 */
void Sensors::updateVisionAngles() {
    pthread_mutex_lock (&angles_mutex);
    pthread_mutex_lock (&vision_angles_mutex);

    visionBodyAngles = bodyAngles;

    pthread_mutex_unlock (&vision_angles_mutex);
    pthread_mutex_unlock (&angles_mutex);
}

void Sensors::lockVisionAngles() {
    pthread_mutex_lock (&vision_angles_mutex);
}

void Sensors::releaseVisionAngles() {
    pthread_mutex_unlock (&vision_angles_mutex);
}

// Get a pointer to the full size Nao image
const uint8_t* Sensors::getNaoImage () const
{
    return naoImage;
}

const uint16_t* Sensors::getYImage () const
{
    return yImage;
}

const uint16_t* Sensors::getImage () const
{
    return yImage;
}

const uint16_t* Sensors::getUVImage() const
{
    return uvImage;
}

const uint8_t* Sensors::getColorImage() const
{
    return colorImage;
}

void Sensors::setNaoImage(const uint8_t *img)
{
    naoImage = img;
}

void Sensors::setImage (const uint16_t *img)
{
    yImage = img;
    uvImage = img + AVERAGED_IMAGE_SIZE;
    colorImage = reinterpret_cast<const uint8_t*>(img + AVERAGED_IMAGE_SIZE*3);
}


void Sensors::resetSaveFrame()
{
    saved_frames = 0;
}

// The version for the frame format
static const int VERSION = 0;

void Sensors::startSavingFrames()
{
#ifdef SAVE_ALL_FRAMES
    if (!isSavingFrames())
    {
        saving_frames_on = true;
        cout << "****Started Saving Frames****" << endl;
    }
#endif
}

void Sensors::stopSavingFrames()
{
#ifdef SAVE_ALL_FRAMES
    if (isSavingFrames())
    {
        saving_frames_on = false;
        cout << "****Stopped Saving Frames****" << endl;
    }
#endif
}

bool Sensors::isSavingFrames() const
{
    return saving_frames_on;
}

// @TODO move this to Transcriber to write out from full size image...
void Sensors::saveFrame()
{
    int MAX_FRAMES = 5000;
    if (saved_frames > MAX_FRAMES)
        return;
    string EXT(".frm");
    string BASE("/");
    int NUMBER = saved_frames;
    stringstream FRAME_PATH;

    FRAME_PATH << FRM_FOLDER << BASE << NUMBER << EXT;
    fstream fout(FRAME_PATH.str().c_str(), fstream::out);

    // Lock and write image possibility of deadlock if something has
    // the image locked and is waiting for visionAngles to unlock -
    // not happening in our code atm
    lockVisionAngles();
    lockImage();

    // @TODO Write out entire 640x480 image
    fout.write(reinterpret_cast<const char*>(getNaoImage()),
               NAO_IMAGE_BYTE_SIZE);
    // write the version of the frame format at the end before joints/sensors
    fout << VERSION << " ";

    // Write joints
    for (vector<float>::const_iterator i = visionBodyAngles.begin();
         i < visionBodyAngles.end(); i++) {
        fout << *i << " ";
    }
    releaseImage();
    releaseVisionAngles();

    // Write sensors
    vector<float> sensor_data = getAllSensors();
    for (vector<float>::const_iterator i = sensor_data.begin();
         i != sensor_data.end(); i++) {
        fout << *i << " ";
    }

    fout.close();
    cout << "Saved frame #" << saved_frames++ << endl;
}

/**
 * Load a frame from a file and set the sensors and image data as
 * appropriate. Useful for running offline.
 */
void Sensors::loadFrame(string path)
{
    fstream fin(path.c_str() , fstream::in);
    if (fin.fail()){
        cout << "Frame load failed: " << path << endl;
        return ;
    }

    lockImage();
    // Load the image from the file, puts it straight into Sensors'
    // image buffer so it doesn't have to allocate its own buffer and
    // worry about deleting it
    uint16_t * img = const_cast<uint16_t*>(getImage());
    uint8_t * byte_img = new uint8_t[320 * 240 * 2];
    fin.read(reinterpret_cast<char *>(byte_img), 320 * 240 * 2);
    releaseImage();

    lockImage();

    // Translate the loaded image into the proper format.
    // @TODO: Convert images to new format.
    for (int i=0; i < 320*240; i++){
        img[i] = 0;
        img[i] = static_cast<uint16_t>(byte_img[i<<1]);
    }
    delete byte_img;

    releaseImage();
    float v;
    int version;
    string space;
    fin >> version;

    vector<float> vba;

    // Read in the body angles
    for (unsigned int i = 0; i < NUM_ACTUATORS; ++i) {
        fin >> v;
        vba += v;
    }
    setVisionBodyAngles(vba);

    // Read sensor values
    vector<float> sensor_data;

    for (int i = 0; i < NUM_SENSORS; ++i) {
        fin >> v;
        sensor_data += v;
    }
    setAllSensors(sensor_data);

    fin.close();
}
