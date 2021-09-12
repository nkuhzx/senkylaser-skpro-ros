// Copyright [nkuhzx] [name of copyright owner]

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


# ifndef SENKEYLASER_H
# define SENKEYLASER_H

# include <iostream>
# include <math.h>
# include <modbus/modbus.h>

using namespace std;

/* Freq mode  */
# define SINGLE    0
# define FREQ_5HZ  1
# define FREQ_10HZ 2
# define FREQ_20HZ 3
# define FREQ_30HZ 4

/* Running status */
# define STOP    0
# define RUNNING 1


/* Register Address */
modbus_t *m_modbus =NULL;
uint16_t ERRORSTATE=0x0000;
uint16_t RUNNINGSTATE=0x0001;
uint16_t DISMEASURE=0x0002;
uint16_t DEVICEADD=0x0003;
uint16_t SERIALSET=0x0004;
uint16_t DISOFFSET=0x0005;
uint16_t SOFTWAREVER=0x0006;
uint16_t MEASUREFREQ=0x0007;
uint16_t DEVICETEMP=0x0008;
uint16_t SERIALNUM=0x0009;


/* Error state */
uint HIGHTEMP=252;
uint LOWTEMP=253;
uint WEAKSIGNAL=255;
uint STRONGSIGNAL=256;
uint STRONGLIGHT=257;


// All register function of senkylaser are encapsulated into classes.
// Example for read distance from senkeylaser:
//    SenkyLaser skfactory(port,service);
//    uint state=skfactory.LaserReadDis();
class SenkyLaser
{
private:

    // USB Port Expample: "/dev/ttyUSB0"
    string port;

    // the senkylaser device ID, when power on for the first time, the device ID
    // can be obtained through the srial port assistant
    uint8_t device_id;

    // baud rate for communication
    uint baudrate; 
    // odd 2 / even 1/ None 0 parity method
    uint16_t parity;

    // offset to the Groud Truth value
    int16_t offset;

    // freq mode
    uint8_t freq_mode;

    // device temp
    int16_t device_temp;

    // serial number
    uint16_t serialnum;



    
public:
    SenkyLaser(){};

    SenkyLaser(string port,int device_id);

    // void setC(string port)

    // Open the Serial
    int openSerial();

    // Set the device id
    void SetLaserDeivce();  

    // Obtain the fault status //work
    int GetLaserErrorState(); 

    // Obtain the measure status //work
    int GetLaserMeasureState();

    // Set the measure state  
    int SetLaserMeasureState(int setmode);

    // Read dis from senkylaser //work
    int GetLaserDis();

    // Obtain the device address //work
    int GetLaserDeviceAddress();

    // Obtain the Buad rate and verification method //work
    int GetLaserSerialPara();
    
    // Set the Baud rate and verification method for communication
    int SetLaserSerialPara(); 

    // Obtain the offset value //work
    int GetLaserOffset();  

    // Obtain the laser frequent //work
    int GetLaserFreq(); 
    
    // Set the laser frequent  //work
    int SetLaserFreq(int setmode);

    //Obtain the device temperature //work
    int GetLaserTemp();  

    // Obtain the serial number of the device //work
    int GetLaserSerialNum(); 

    // Obtain the BaudRate
    uint getBaudrate(); //work

    // Obtain the parity
    uint getParity(); //work

    // Obtain the serial num
    uint getSerialnum();

    string getPort();

    uint getDeviceid();
    
    
    ~SenkyLaser(){};
};


SenkyLaser::SenkyLaser(string port,int device_id)
{
    this->port=port;
    this->device_id=device_id;

}


// Open the Serial
// Return the openSerial state (1:sucess -1:fail)
int SenkyLaser::openSerial()
{

    m_modbus=modbus_new_rtu(this->port.data(),115200,'N',8,1);
    
    if(m_modbus==nullptr)
    {
        // cout<<"wrong modbus parameter";
        return 0;
    }
    timeval time_out;
    time_out.tv_sec=0;
    time_out.tv_usec=1000*100;

    modbus_rtu_set_serial_mode(m_modbus,MODBUS_RTU_RS485);

    // modbus_set_debug(m_modbus,1);
    if(modbus_connect(m_modbus)==-1)
    {
        // cout<<"Cannot connetct modbus at port: "<<port<<endl;

        return -1;
    }
    else
    {
        // cout<<"Connect Sucessfully!! Connected modbus at port: "<<port<<endl;
    
        return 1;
    }
}

//  Set the Device ID 
void SenkyLaser::SetLaserDeivce()
{
    int flag=modbus_set_slave(m_modbus,this->device_id);
}

//  Read Error state 
//  Return Measure state (0:no wrong/255:wrong/-1:fail to connect 
int SenkyLaser::GetLaserErrorState()
{
    int measure_state;

    uint16_t tab_reg[64];
    int flag=modbus_read_registers(m_modbus,ERRORSTATE,1,tab_reg);
    if (flag==-1)
    {
        return -1;
    }
    else
    {
        measure_state=tab_reg[0];
    }

    return measure_state;
  
}


//  Read Measure state 
//  Return Measure state (0:no wrong/2:measuring/-1:fail to connect 
int SenkyLaser::GetLaserMeasureState()
{
    int measure_state;

    uint16_t tab_reg[64];
    int flag=modbus_read_registers(m_modbus,RUNNINGSTATE,1,tab_reg);
    if (flag==-1)
    {
        return -1;
    }
    else
    {
        measure_state=tab_reg[0];
    }

    return measure_state;
  
}


//TODO
// uint8_t SenkyLaser::SetLaserMeasureState()
// {

// }


//  Read the distance from laser 
//  Return the distance (oct)
int SenkyLaser::GetLaserDis()
{
    uint16_t tab_reg[64];
    int measure_dis=0;

    int flag=modbus_read_registers(m_modbus,DISMEASURE,2,tab_reg);
    if (flag==-1)
    {
        return -1;
    }
    else
    {
        // cout<<tab_reg[0]<<" "<<tab_reg[1]<<endl; 
        uint32_t high=tab_reg[0];
        uint32_t low=tab_reg[1];
        measure_dis=(high<<16)+low;

    }    

    return measure_dis;
}


//  Read the device add from laser 
//  Return the device add (1-247)
int SenkyLaser::GetLaserDeviceAddress()
{
    uint16_t tab_reg[64];
    uint8_t device_add;
    int flag=modbus_read_registers(m_modbus,DEVICEADD,1,tab_reg);
    if(flag==-1)
    {   
        return -1;
    }
    else
    {
        device_add=tab_reg[0];
        this->device_id=device_add;
        return device_add;
    }
}


//  Read the Baud rate from laser 
//  Return 1:sucess -1:fail
int SenkyLaser::GetLaserSerialPara()
{
    uint16_t tab_reg[64];
    uint baudrate_value;
    uint parity_value;

    int flag=modbus_read_registers(m_modbus,SERIALSET,2,tab_reg);
    if (flag==-1)
    {
        return -1;
    }
    else
    {
        baudrate_value=((tab_reg[0]&0x00FF)<<16)+tab_reg[1];
        parity_value=((tab_reg[0]&0xFF00)>>8);

        this->baudrate=baudrate_value;
        this->parity=parity_value;

        return 1;
    }    

}


//  Read the offset value to GT from laser 
//  Return offset value
int SenkyLaser::GetLaserOffset()
{
    uint16_t tab_reg[64];
    int16_t offset_value;


    int flag=modbus_read_registers(m_modbus,DISOFFSET,1,tab_reg);
    if(flag==-1)
    {   
        return -1;
    }
    else
    {
        offset_value=tab_reg[0];
        this->offset=offset_value;
        return offset_value;
    }
}

//  Read the laser frequent mode from laser
//  Return frequent mode
int SenkyLaser::GetLaserFreq()
{
    uint16_t tab_reg[64];
    uint8_t freq_mode;

    int flag=modbus_read_registers(m_modbus,MEASUREFREQ,1,tab_reg);
    if(flag==-1)
    {   
        return -1;
    }
    else
    {
        freq_mode=tab_reg[0];
        this->freq_mode=freq_mode;
        return freq_mode;
    }  
}

//  Read the laser tempeture from laser
//  Return device tempeture
int SenkyLaser::GetLaserTemp()
{
    uint16_t tab_reg[64];
    int16_t device_temp;

    int flag=modbus_read_registers(m_modbus,DEVICETEMP,1,tab_reg);
    if(flag==-1)
    {   
        return -1;
    }
    else
    {
        device_temp=tab_reg[0];

        this->device_temp=device_temp;
        return device_temp;
    }  
}

//  Read the serial numbers from laser
//  Return serial numbers
int SenkyLaser::GetLaserSerialNum()
{
    uint16_t tab_reg[64];
    uint8_t serialnum;

    int flag=modbus_read_registers(m_modbus,SERIALNUM,2,tab_reg);
    if(flag==-1)
    {   
        return -1;
    }
    else
    {
        serialnum=(tab_reg[0]<<16)+tab_reg[1];
        this->serialnum=serialnum;
        return serialnum;
    }      

}


int SenkyLaser::SetLaserFreq(int setmode)
{
    int flag=modbus_write_register(m_modbus,MEASUREFREQ,setmode);
    if (flag==-1)
    {
        return -1;
    }
    else
    {
        return 1;
    }
}


int SenkyLaser::SetLaserMeasureState(int setmode)
{
    int flag=modbus_write_register(m_modbus,RUNNINGSTATE,setmode);
    if (flag==-1)
    {
        return -1;
    }
    else
    {
        return 1;
    }
}



// return the baudrate
uint SenkyLaser::getBaudrate()
{
    return this->baudrate;
}

//return the paritymode
uint SenkyLaser::getParity()
{
    return this->parity;
}

uint SenkyLaser::getSerialnum()
{
    return this->serialnum;
}

string SenkyLaser::getPort()

{

    return this->port;
}

uint SenkyLaser::getDeviceid()
{

    return this->device_id;
}


//  












# endif