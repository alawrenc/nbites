#include "RoboGuardian.h"

#include <iostream>
#include <cstdlib>

using namespace std;

#include "Kinematics.h"

#include "FreezeCommand.h"
#include "UnfreezeCommand.h"

//#define DEBUG_GUARDIAN_CLICKS
#define WIFI_CONNECTION NBITES
#define WIFI_RECONNECTS_MAX 5
//check for a connection once in 20 secs
#define CONNECTION_CHECK_RATE 20*RoboGuardian::GUARDIAN_FRAME_RATE

const int RoboGuardian::GUARDIAN_FRAME_RATE = MOTION_FRAME_RATE;
// 1 second * 1000 ms/s * 1000 us/ms
const int RoboGuardian::GUARDIAN_FRAME_LENGTH_uS = 1 * 1000 * 1000 /
    RoboGuardian::GUARDIAN_FRAME_RATE;

const int RoboGuardian::NO_CLICKS = -1;

static const string quiet = " -q ";
static const string sout = "aplay"+quiet;
static const string sdir = "/opt/naoqi/share/naoqi/wav/";
static const string nbsdir = "/home/nao/naoqi/etc/audio/";
static const string wav = ".wav";
static const string shutdown_wav = sdir + "shutdown" + wav;
static const string heat_wav = sdir + "heat" + wav;
static const string energy_wav = sdir + "energy" + wav;
static const string mynameis_wav = nbsdir + "mynameis" + wav;
static const string my_address_is_wav = sdir + "my_internet_address_is" + wav;
static const string stiffness_removed_wav = sdir + "emergency_stiffness"+wav;
static const string stiffness_enabled_wav = nbsdir + "stiffness_enabled"+wav;
static const string warning_wav = sdir + "warning"+wav;
static const string falling_wav = nbsdir +"falling"+wav;
static const string wifi_restart_wav = nbsdir +"wifi_restart"+wav;
static const string dot = ".";


static const boost::shared_ptr<FreezeCommand> REMOVE_GAINS =
    boost::shared_ptr<FreezeCommand>
    (new FreezeCommand());

static const boost::shared_ptr<UnfreezeCommand> ENABLE_GAINS =
    boost::shared_ptr<UnfreezeCommand>
    (new UnfreezeCommand());

//Non blocking!!
void RoboGuardian::playFile(string str)const{
    system((sout+str+" &").c_str()); // system returns an int. 
}



RoboGuardian::RoboGuardian(boost::shared_ptr<Synchro> _synchro,
                           boost::shared_ptr<Sensors> s)
    : Thread(_synchro,"RoboGuardian"), sensors(s),
      motion_interface(NULL),
      lastTemps(sensors->getBodyTemperatures()),
      lastBatteryCharge(sensors->getBatteryCharge()),
      chestButton(new ClickableButton(GUARDIAN_FRAME_RATE)),
      leftFootButton(new ClickableButton(GUARDIAN_FRAME_RATE)),
      rightFootButton(new ClickableButton(GUARDIAN_FRAME_RATE)),
      // buttonOnCounter(0),buttonOffCounter(0),
      //       lastButtonOnCounter(0),lastButtonOffCounter(0),
      //       buttonClicks(0),
      lastInertial(sensors->getInertial()), fallingFrames(0),
      notFallingFrames(0),fallenCounter(0),
      registeredFalling(false),registeredShutdown(false),
      falling(false),fallen(false),
      useFallProtection(false),
      wifiReconnectTimeout(0), lastHeatAudioWarning(0), lastHeatPrintWarning(0)
{
    pthread_mutex_init(&click_mutex,NULL);
    executeStartupAction();
}
RoboGuardian::~RoboGuardian(){
    pthread_mutex_destroy(&click_mutex);
}



void RoboGuardian::run(){
    Thread::running = true;
    Thread::trigger->on();

    struct timespec interval, remainder;
    interval.tv_sec = 0;
    interval.tv_nsec = static_cast<long long int> (GUARDIAN_FRAME_LENGTH_uS * 1000);
    int connectionCheckCount = 0;
    while(Thread::running){

        countButtonPushes();
        checkFalling();
        checkFallen();
        checkBatteryLevels();
        checkTemperatures();
        processFallingProtection();
        processChestButtonPushes();
        if (connectionCheckCount == CONNECTION_CHECK_RATE) {
            connectionCheckCount = 0;
            checkConnection();
        } else {
            connectionCheckCount++;
        }
        nanosleep(&interval, &remainder);
    }

    Thread::trigger->off();
}

void RoboGuardian::shutoffGains(){
    cout << "RoboGuardian::shutoffGains()" <<endl;
    if(motion_interface != NULL)
        motion_interface->sendFreezeCommand(REMOVE_GAINS);
    playFile(stiffness_removed_wav);
}

void RoboGuardian::enableGains(){
    cout << "RoboGuardian::enableGains()" <<endl;
    if(motion_interface != NULL)
        motion_interface->sendFreezeCommand(ENABLE_GAINS);
    playFile(stiffness_enabled_wav);
}

static const float FALL_SPEED_THRESH = 0.03f; //rads/20ms
static const float NOFALL_SPEED_THRESH = 0.01f; //rads/20ms
static const int FALLING_FRAMES_THRESH = 3;
static const int FALLING_RESET_FRAMES_THRESH = 10;
static const float FALLING_ANGLE_THRESH = M_PI_FLOAT/6.0f; //30 degrees
static const float FALLEN_ANGLE_THRESH = M_PI_FLOAT/2.5f; //72 degrees


//Check if the angle is unstable, (ie tending AWAY from zero)
bool isFalling(float angle_pos, float angle_vel){
    if (angle_pos < 0){
        //then we only care if the velocity is negative
        if(angle_vel < -FALL_SPEED_THRESH)
            return true;
    }else{
        if(angle_vel > FALL_SPEED_THRESH)
            return true;
    }
    return false;
}



void RoboGuardian::checkFallen(){
    const Inertial inertial  = sensors->getInertial();

    /***** Determine if the robot has FALLEN OVER *****/
    const bool fallen_now =
        std::abs(inertial.angleX) > FALLEN_ANGLE_THRESH ||
        std::abs(inertial.angleY) > FALLEN_ANGLE_THRESH;

    if(fallen_now)
        fallenCounter +=1;
    else
        fallenCounter = 0;

    static const int FALLEN_FRAMES_THRESH  = 2;
    if(fallenCounter >= FALLEN_FRAMES_THRESH ){
        fallen = true;
#ifdef DEBUG_GUARDIAN_FALLING
        cout << "Robot has fallen" <<endl;
#endif
    }


}


/**
 * Method to watch the robots rotation and detect falls
 * publishes the result of this computation to other classes.
 *
 * Also builds in the ability to shutoff gains when a fall is detected,
 * but this still has some problems. Mainly, it sometimes triggers a fall
 * when the robot is rotated back up again, but only when the robot is
 * 'over rotated' during the righting.  This feature can be enabled
 * by calling 'enableFallProtection'.
 * Also, currently the robot will print whenever it believes it is in
 * the process of falling. This will allow us to monitor how well this code
 * works.
 *
 */
void RoboGuardian::checkFalling(){
    const Inertial inertial  = sensors->getInertial();

    /***** Determine if the robot is in the process of FALLING ****/
    //Using just the magnitude:
    const float angleMag = std::sqrt(std::pow(inertial.angleX,2) +
                                     std::pow(inertial.angleY,2));
    const float lastAngleMag = std::sqrt(std::pow(lastInertial.angleX,2) +
                                         std::pow(lastInertial.angleY,2));

    const float angleSpeed = angleMag - lastAngleMag;
    const bool falling_critical_angle = angleMag > FALLING_ANGLE_THRESH;

    if(isFalling(angleMag,angleSpeed)){
        fallingFrames += 1;
        notFallingFrames = 0;
    }else if( angleSpeed < NOFALL_SPEED_THRESH){
        fallingFrames = 0;
        notFallingFrames +=1;
    }
    //     cout << "angleSpeed "<<angleSpeed << " and angleMag "<<angleMag<<endl
    //          << "  fallingFrames is " << fallingFrames
    //          << " and critical angle is "<< falling_critical_angle<< endl;

    //If the robot has been falling for a while, and the robot is inclined
    //already at a 45 degree angle, than we know we are falling
    if (fallingFrames > FALLING_FRAMES_THRESH && falling_critical_angle){
        //Execute protection? Just print for now and we'll see how this works
        falling = true;
    }else if(notFallingFrames > FALLING_FRAMES_THRESH){
        falling = false;
    }


    lastInertial  = inertial;
}


void RoboGuardian::processFallingProtection(){
    if(falling && !registeredFalling){
        registeredFalling = true;
        executeFallProtection();
    }else if(notFallingFrames > FALLING_RESET_FRAMES_THRESH){
        registeredFalling = false;
    }

    //     if(fallingFrames == FALLING_FRAMES_THRESH && falling_critical_angle){
    //         if(useFallProtection){
    //             shutoffGains();
    //             cout << "Disabling Gains!!! due to falling" <<endl;
    //         }
    // #define DEBUG_GUARDIAN_FALLING
    // #ifdef DEBUG_GUARDIAN_FALLING
    //         cout << Thread::name <<": OH NO! I think I'm falling" <<endl;
    //         playFile(falling_wav);
    // #endif
    //     }
}

// We print each time the battery looses ten percent of charge
//once the charge gets to %30 percent of capacity, we start to say "energy"
void RoboGuardian::checkBatteryLevels(){
    //Constants on 0-10 scale, sensor reading is on 0-1.0 scale
    static const float LOW_BATTERY_VALUE = 3.0f;
    static const float EMPTY_BATTERY_VALUE = 0.7f; //start nagging below 7%

    //sometimes we get weird values from the battery (like -9.8...)
    const float newBatteryCharge =  sensors->getBatteryCharge();
    if(newBatteryCharge < 0 || newBatteryCharge > 1.0){
#ifdef GUARDIAN_DEBUG_BATTERY
        cout<< Thread::name<<": Got bad battery charge of "<< newBatteryCharge
            <<endl;
#endif
        return;
    }

    if(newBatteryCharge != lastBatteryCharge){

        //covert to a 0 - 10 scale
        const float newLevel = floor(newBatteryCharge*10.0f);
        const float oldLevel = floor(lastBatteryCharge*10.0f);
        if(oldLevel != newLevel && oldLevel > newLevel &&
           oldLevel - newLevel <= 1.0f){
            cout << Thread::name << ": Battery charge is now at "
                 << 10.0f * newBatteryCharge
                 << " (was "<<oldLevel<<")"<<endl;

            if (newLevel <= LOW_BATTERY_VALUE){
                playFile(energy_wav);
            }else if(newLevel <= EMPTY_BATTERY_VALUE){
                playFile(energy_wav); playFile(energy_wav);
            }
        }
    }

    lastBatteryCharge = newBatteryCharge;
}

void RoboGuardian::checkTemperatures(){
    static const float HIGH_TEMP = 40.0f; //deg C
    static const float TEMP_THRESHOLD = 1.0f; //deg C
    static const float REALLY_HIGH_TEMP = 50.0f; //deg C

    vector<float> newTemps = sensors->getBodyTemperatures();

    bool sayWarning = false;
    for(unsigned int joint = 0; joint < Kinematics::NUM_JOINTS; joint++){
        const float tempDiff = newTemps[joint] - lastTemps[joint];
        if(newTemps[joint] >= HIGH_TEMP && tempDiff >= TEMP_THRESHOLD &&
           micro_time() - lastHeatPrintWarning > TIME_BETWEEN_HEAT_WARNINGS){
            lastHeatPrintWarning = micro_time();
            cout << Thread::name << "::" << "TEMP-WARNING: "
                 << Kinematics::JOINT_STRINGS[joint]
                 << " is at " << setprecision(1)
                 << newTemps[joint] <<" deg C"<< setprecision(6) << endl;
            if(newTemps[joint] >= REALLY_HIGH_TEMP){
                sayWarning = true;
            }
        }
    }
    if(sayWarning &&
       micro_time() - lastHeatAudioWarning > TIME_BETWEEN_HEAT_WARNINGS){
        playFile(heat_wav);
        lastHeatAudioWarning = micro_time();
    }
    lastTemps = newTemps;
}

void RoboGuardian::countButtonPushes(){
    //First, update the buttons with their new values
    const FootBumper left = sensors->getLeftFootBumper();
    const FootBumper right = sensors->getRightFootBumper();

    chestButton->updateFrame(sensors->getChestButton());
    leftFootButton->updateFrame(left.left || left.right);
    rightFootButton->updateFrame(right.left || right.right);

}

void RoboGuardian::processChestButtonPushes(){
    static const int SHUTDOWN_THRESH = 3;

    //Then, process our actions locally
    if(chestButton->getClickLength() == 0.0f ){
        registeredShutdown  = false;
    }else if(chestButton->getClickLength() > SHUTDOWN_THRESH &&
             !registeredShutdown){
        registeredShutdown = true;
        executeShutdownAction();
    }

    if(executeChestClickAction(chestButton->peekNumClicks())){
        chestButton->getAndClearNumClicks();
    }
}


bool RoboGuardian::executeChestClickAction(int nClicks){
#ifdef DEBUG_GUARDIAN_CLICKS
    cout << "Processing "<<nClicks<< " clicks"<<endl;
#endif

    //NOTE: Please upade wiki/NaoChestButtonInterface when this is changed!!!
    switch(nClicks){
    case NO_CLICKS:
        return false;
        break;
    case 2:
        shutoffGains();
        break;
    case 3:
        //Say IP Address
        speakIPAddress();
        break;
    case 5:
        enableGains();
        break;
    case 7:
		checkConnection();
        break;
    case 9:
        //Easter EGG!
        playFile(nbsdir+"easter_egg.wav");
        break;
    default:
        //nothing
        //cout << Thread::name <<" is leaving "<<nClicks<<" clicks for someone else"<<endl;
        return false;
        break;
    }
    return true;
}


void RoboGuardian::executeFallProtection(){
    if(useFallProtection){
        cout << Thread::name <<": OH NO! I'm falling. Removing stiffness." <<endl;
        shutoffGains();
    }
#ifdef DEBUG_GUARDIAN_FALLING
    playFile(falling_wav);
#endif
}

//     if(fallingFrames == FALLING_FRAMES_THRESH && falling_critical_angle){
//         if(useFallProtection){
//             shutoffGains();
//             cout << "Disabling Gains!!! due to falling" <<endl;
//         }
// #define DEBUG_GUARDIAN_FALLING
// #ifdef DEBUG_GUARDIAN_FALLING
//         cout << Thread::name <<": OH NO! I think I'm falling" <<endl;
//         playFile(falling_wav);
// #endif
//     }


void RoboGuardian::executeStartupAction() const{
    //Blank for now

}

void RoboGuardian::executeShutdownAction()const {
    cout << Thread::name<<" is shutting down the robot NOW!!!"<<endl;
    playFile(shutdown_wav);
    system("shutdown -h now &");
}

string RoboGuardian::getHostName()const {
    char name[40];
    name[0] ='\0';
    gethostname(name,39);
    return string(name);
}

const string RoboGuardian::discoverIP() const{
    // try ...|awk '{print $1 " " $2}' and grep -v inet6
    system("ifconfig|grep 'inet'|cut -d':' -f2|awk '{print $1}'|grep -v 127.0.0.1 > /tmp/ip.txt");
    char ip[100];
    FILE * ipf = fopen("/tmp/ip.txt","r");
    if(ipf != NULL){
        fscanf(ipf,"%s\n",ip);
        return ip;
        fclose(ipf);
    }else{
        cout << "Unable to read IP from this platform"<<endl;
        return "0";
    }

}

void RoboGuardian::speakIPAddress()const {
    //Currently we poll the broker. If this breaks in the future
    //you can try to call /opt/naoqi/bin/ip.sh or
    //parse the output of if config yourself
    const string IP = discoverIP();//broker->getIP();
    const string host = getHostName();

    string speech_command = sout;
    speech_command += " "+mynameis_wav;
    speech_command += " "+nbsdir+host+wav;
    speech_command += " "+my_address_is_wav;

    for (unsigned int i = 0; i < IP.size(); i++){
        char digit = IP[i];
        if(digit == dot[0])
            speech_command += " "+sdir+"point"+wav;
        else
            speech_command += " "+sdir+digit+wav;

    }
    speech_command += " &";
    cout <<Thread::name<<" is speaking: My name is "<<host
         << " my internet address is "
         <<IP<<endl;

    system(speech_command.c_str());
}


boost::shared_ptr<ClickableButton>  RoboGuardian::getButton(ButtonID buttonID) const{
    switch(buttonID){
    case CHEST_BUTTON:
        return chestButton;
    case LEFT_FOOT_BUTTON:
        return leftFootButton;
    case RIGHT_FOOT_BUTTON:
        return rightFootButton;
    default:
        cout << "This button doesnt exist. Returning chest button" <<endl;
        return chestButton;
    }
}
//TODO: comment
void RoboGuardian::checkConnection(){
    const string IP = discoverIP();
    if (IP.size() >= 7 && (IP[0] == '1' || IP[0] == '2')) {
        wifiReconnectTimeout = 0;
        return;
    } else {
        if (wifiReconnectTimeout < WIFI_RECONNECTS_MAX) {
            cout    << "No connection detected, trying to reconnect interfaces, attempt "
                    << wifiReconnectTimeout << endl;
            reconnectWifiConnection();
            wifiReconnectTimeout++;
        }
    }
}

bool RoboGuardian::checkWired(){
    FILE * f1 = popen("connman services | awk '/Wired/ {print $1}'", "r");
    char status[3] = "";
    fscanf(f1,"%s\n",status);
    pclose(f1);
    if(status[0] == '*') {
        cout<<"wired "<<status<<endl;
        return true;
    }
    return false;
}

bool RoboGuardian::checkWireless(){

    FILE * f2 = popen("connman services | awk '/ROBOTICS/ {print $1}'", "r");
    char status[3] = "";
    fscanf(f2,"%s\n",status);
    pclose(f2);
    if (status[0] == '*') {
        cout<<"wireless"<<endl;
        return true;
    }
    return false;
}

// we assume that autoconnect is on and that we already  have connected
// to the network before
void RoboGuardian::reconnectWifiConnection(){
    // playFile(wifi_restart_wav);
    FILE * f3 = popen("connman services | awk '/ROBOTICS/ {print $4}'", "r");
    char service[100] = "";
    fscanf(f3,"%s\n", service);
    pclose(f3);

    if (service[0] != ' ') {
        char command[100] = "connman connect ";
        strcat(command, service);
        system(command);
    } else {
        cout<<"couldn't find specified wifi network to reconnect to";
    }
}

void RoboGuardian::ifUpDown(){
    char ifdown[] = "su -c 'ifdown wlan0'";
    char ifup[] = "su -c 'ifup wlan0'&";
    cout << "RoboGuardian::ifUpDown() -- reconnecting interfaces\n";
    playFile(wifi_restart_wav);
    system(ifdown);
    system(ifup);
}
