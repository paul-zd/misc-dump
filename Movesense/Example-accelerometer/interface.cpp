#include "myApp.h"
#include "interface.h"

// sensor libraries for data interperting
#include "meas_acc/resources.h"
#include "meas_gyro/resources.h"
#include "meas_magn/resources.h"
#include "meas_imu/resources.h"

//Commands formated as byte array with [Command, data?...] (data optional)
enum Commands 
{
    HELLO = 0,
    START = 1,
    STOP = 2,
    LED = 3,
};
//Responses formated as byte array with [Response, tag, data?...] (data optional)
enum Responses 
{
    COMMAND_RESULT = 1,
    DATA = 2,
    ERROR = 3,
};

const uint8_t REF=1;
uint16_t pattern[6];      
void myApp::handleCommand(uint8_t cmd, const uint8_t values[], size_t len){
    switch (cmd)
    {
        case Commands::HELLO: {
            // Hello response, for use as a sanity check <3
            uint8_t helloMsg[] = {'H','e','l','l','o','!'};
            uint8_t tag=1;
            sendPacket(helloMsg, sizeof(helloMsg), tag, Responses::COMMAND_RESULT);
        }
        break;
        case Commands::START: {
            char path[] = "Meas/IMU6/52";
            subscribe(path, sizeof(path), REF);
        }
        break;
        case Commands::STOP: {
            unsubscribe(REF);
        }
        break;
        case Commands::LED: {
            for(int i=0; i<6; i++) pattern[i] = i%2==1? 500:0;
            for(size_t i=0; i<3 && i<len; i++){
                pattern[i*2] = (uint16_t)values[i]*1000;
            }
            ledSetPattern_n(pattern, 6);
        }
        break;
    }
}

void myApp::processData(wb::ResourceId resourceId, const wb::Value &value){
    if(findDataSub(resourceId)->clientReference != REF)
        return;
    const WB_RES::IMU6Data &data = value.convertTo<WB_RES::IMU6Data&>();

    float magnitudes[16]; 
    const wb::Array<wb::FloatVector3D> &accData = data.arrayAcc;

    float averageMagnitude=0;
    size_t i;
    for(i=0; i<15 && i<accData.size(); i++) {
        wb::FloatVector3D a = accData[i];
        float magnitude = sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
        magnitudes[i+1] = magnitude;
        averageMagnitude += magnitude;
    }
    averageMagnitude /= i;

    *((char*)magnitudes+3) = averageMagnitude<3.0f? 1:0;
    uint8_t tag=5;
    sendPacket((uint8_t*)magnitudes+3, 1+i*sizeof(float), tag, Responses::DATA);
}