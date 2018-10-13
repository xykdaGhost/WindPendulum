#include "MPU9250_IIC.h"

// 定义MPU9250内部地址
//****************************************
#define SMPLRT_DIV    0x19  //陀螺仪采样率，典型值：0x07(125Hz)
#define CONFIG      0x1A  //低通滤波频率，典型值：0x06(5Hz)
#define GYRO_CONFIG   0x1B  //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define ACCEL_CONFIG  0x1C  //加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2g，5Hz)

#define ACCEL_XOUT_H  0x3B
#define ACCEL_XOUT_L  0x3C
#define ACCEL_YOUT_H  0x3D
#define ACCEL_YOUT_L  0x3E
#define ACCEL_ZOUT_H  0x3F
#define ACCEL_ZOUT_L  0x40

#define TEMP_OUT_H    0x41
#define TEMP_OUT_L    0x42

#define GYRO_XOUT_H   0x43
#define GYRO_XOUT_L   0x44  
#define GYRO_YOUT_H   0x45
#define GYRO_YOUT_L   0x46
#define GYRO_ZOUT_H   0x47
#define GYRO_ZOUT_L   0x48

    
#define MAG_XOUT_L    0x03
#define MAG_XOUT_H    0x04
#define MAG_YOUT_L    0x05
#define MAG_YOUT_H    0x06
#define MAG_ZOUT_L    0x07
#define MAG_ZOUT_H    0x08

#define PWR_MGMT_1    0x6B  //电源管理，典型值：0x00(正常启用)
#define WHO_AM_I      0x75  //IIC地址寄存器(默认数值0x68，只读)

//****************************

#define GYRO_ADDRESS   0xD0   //陀螺地址
#define MAG_ADDRESS    0x18   //磁场地址
#define ACCEL_ADDRESS  0xD0 

unsigned char TX_DATA[4];    //显示据缓存区
unsigned char BUF[10];       //接收数据缓存区
char  test=0;          //IIC用到
float ACCEL_X=0.0f,ACCEL_Y=0.0f,ACCEL_Z=0.0f;    //X,Y,Z轴加速度
float GYRO_X=0.0f,GYRO_Y=0.0f,GYRO_Z=0.0f;
short MAG_X=0,MAG_Y=0,MAG_Z=0;
//************************************
/*模拟IIC端口输出输入定义*/
//#define SCL_H         GPIOA->BSRR = GPIO_Pin_1
//#define SCL_L         GPIOA->BRR  = GPIO_Pin_1 
//   
//#define SDA_H         GPIOA->BSRR = GPIO_Pin_0
//#define SDA_L         GPIOA->BRR  = GPIO_Pin_0

//#define SCL_read      GPIOA->IDR  & GPIO_Pin_1
//#define SDA_read      GPIOA->IDR  & GPIO_Pin_0
#define SCL_H         PAout(12)=1
#define SCL_L         PAout(12)=0
   
#define SDA_H         PAout(11)=1
#define SDA_L         PAout(11)=0

#define SCL_read      PAin(12)
#define SDA_read      PAin(11)
/* 变量定义 ----------------------------------------------*/

  /*******************************/
void DATA_printf(uchar *s,short temp_data)
{
  if(temp_data<0){
  temp_data=-temp_data;
    *s='-';
  }
  else *s=' ';
    *++s =temp_data/100+0x30;
    temp_data=temp_data%100;     //取余运算
    *++s =temp_data/10+0x30;
    temp_data=temp_data%10;      //取余运算
    *++s =temp_data+0x30;   
}

/*******************************************************************************
* Function Name  : I2C_GPIO_Config
* Description    : Configration Simulation IIC GPIO
* Input          : None 
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_GPIO_Config(void)
{
  RCC->APB2ENR|=1<<2; //?? PORTB ??
  GPIOA->CRH&=0XFFF00FFF;
  GPIOA->CRH|=0X00077000;//PB.5 ????
//  GPIO_InitTypeDef  GPIO_InitStructure; 
// 
//  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  
//  GPIO_Init(GPIOA, &GPIO_InitStructure);

//  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name  : I2C_delay
* Description    : Simulation IIC Timing series delay
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_delay(void)
{
    
   u8 i=6; //这里可以优化速度 ，经测试最低到5还能写入
   while(i) 
   { 
     i--; 
   }  
}

/*******************************************************************************
* Function Name  : I2C_Start
* Description    : Master Start Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : Wheather  Start
****************************************************************************** */
bool I2C_Start(void)
{
  SDA_H;
  SCL_H;
  I2C_delay();
  if(!SDA_read)return FALSE;  //SDA线为低电平则总线忙,退出
  SDA_L;
  I2C_delay();
  if(SDA_read) return FALSE;  //SDA线为高电平则总线出错,退出
  SDA_L;
  I2C_delay();
  return TRUE;
}
/*******************************************************************************
* Function Name  : I2C_Stop
* Description    : Master Stop Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Stop(void)
{
  SCL_L;
  I2C_delay();
  SDA_L;
  I2C_delay();
  SCL_H;
  I2C_delay();
  SDA_H;
  I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_Ack
* Description    : Master Send Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Ack(void)
{ 
  SCL_L;
  I2C_delay();
  SDA_L;
  I2C_delay();
  SCL_H;
  I2C_delay();
  SCL_L;
  I2C_delay();
}   
/*******************************************************************************
* Function Name  : I2C_NoAck
* Description    : Master Send No Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_NoAck(void)
{ 
  SCL_L;
  I2C_delay();
  SDA_H;
  I2C_delay();
  SCL_H;
  I2C_delay();
  SCL_L;
  I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : Master Reserive Slave Acknowledge Single
* Input          : None
* Output         : None
* Return         : Wheather  Reserive Slave Acknowledge Single
****************************************************************************** */
bool I2C_WaitAck(void)   //返回为:=1有ACK,=0无ACK
{
  SCL_L;
  I2C_delay();
  SDA_H;      
  I2C_delay();
  SCL_H;
  I2C_delay();
  if(SDA_read)
  {
      SCL_L;
    I2C_delay();
      return FALSE;
  }
  SCL_L;
  I2C_delay();
  return TRUE;
}
/*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : Master Send a Byte to Slave
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_SendByte(u8 SendByte) //数据从高位到低位//
{
    u8 i=8;
    while(i--)
    {
        SCL_L;
        I2C_delay();
      if(SendByte&0x80)
        SDA_H;  
      else 
        SDA_L;   
        SendByte<<=1;
        I2C_delay();
    SCL_H;
        I2C_delay();
    }
    SCL_L;
}  
/*******************************************************************************
* Function Name  : I2C_RadeByte
* Description    : Master Reserive a Byte From Slave
* Input          : None
* Output         : None
* Return         : Date From Slave 
****************************************************************************** */
unsigned char I2C_RadeByte(void)  //数据从高位到低位//
{ 
    u8 i=8;
    u8 ReceiveByte=0;

    SDA_H;        
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L;
      I2C_delay();
    SCL_H;
      I2C_delay();  
      if(SDA_read)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L;
    return ReceiveByte;
} 
//ZRX          
//单字节写入*******************************************

bool Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)         //void
{
    if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress);   //发送设备地址+写信号//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
    I2C_SendByte(REG_Address );   //设置低起始地址      
    I2C_WaitAck();  
    I2C_SendByte(REG_data);
    I2C_WaitAck();   
    I2C_Stop(); 
    delay_ms(5);
    return TRUE;
}

//单字节读取*****************************************
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address)
{   unsigned char REG_data;       
  if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop();test=1; return FALSE;}
    I2C_SendByte((u8) REG_Address);   //设置低起始地址      
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(SlaveAddress+1);
    I2C_WaitAck();

  REG_data= I2C_RadeByte();
    I2C_NoAck();
    I2C_Stop();
    //return TRUE;
  return REG_data;

}

//初始化MPU9250，根据需要请参考pdf进行修改************************
void Init_MPU9250(void)
{
  I2C_GPIO_Config();
/*
   Single_Write(GYRO_ADDRESS,PWR_M, 0x80);   //
   Single_Write(GYRO_ADDRESS,SMPL, 0x07);    //
   Single_Write(GYRO_ADDRESS,DLPF, 0x1E);    //±2000°
   Single_Write(GYRO_ADDRESS,INT_C, 0x00 );  //
   Single_Write(GYRO_ADDRESS,PWR_M, 0x00);   //
*/
  
  Single_Write(GYRO_ADDRESS,PWR_MGMT_1, 0x00);  //解除休眠状态
  Single_Write(GYRO_ADDRESS,SMPLRT_DIV, 0x07);
  Single_Write(GYRO_ADDRESS,CONFIG, 0x06);
  Single_Write(GYRO_ADDRESS,GYRO_CONFIG, 0x18);
  Single_Write(GYRO_ADDRESS,ACCEL_CONFIG, 0x01);
  //----------------
//  Single_Write(GYRO_ADDRESS,0x6A,0x00);//close Master Mode  

}
  
//******读取MPU9250数据****************************************
void READ_MPU9250_ACCEL(void)
{ 
  short ACCEL_X_DATA,ACCEL_Y_DATA,ACCEL_Z_DATA;
   BUF[0]=Single_Read(ACCEL_ADDRESS,ACCEL_XOUT_L); 
   BUF[1]=Single_Read(ACCEL_ADDRESS,ACCEL_XOUT_H);
   ACCEL_X_DATA = (BUF[1]<<8)|BUF[0];
   ACCEL_X = ACCEL_X_DATA/164;               //读取计算X轴数据

   BUF[2]=Single_Read(ACCEL_ADDRESS,ACCEL_YOUT_L);
   BUF[3]=Single_Read(ACCEL_ADDRESS,ACCEL_YOUT_H);
   ACCEL_Y_DATA = (BUF[3]<<8)|BUF[2];
   ACCEL_Y = ACCEL_Y_DATA/164;               //读取计算Y轴数据
   BUF[4]=Single_Read(ACCEL_ADDRESS,ACCEL_ZOUT_L);
   BUF[5]=Single_Read(ACCEL_ADDRESS,ACCEL_ZOUT_H);
   ACCEL_Z_DATA = (BUF[5]<<8)|BUF[4];
   ACCEL_Z=ACCEL_Z_DATA/164;                 //读取计算Z轴数据
 
}

void READ_MPU9250_GYRO(void)
{ 
  short GYRO_X_DATA,GYRO_Y_DATA,GYRO_Z_DATA;
   BUF[0]=Single_Read(GYRO_ADDRESS,GYRO_XOUT_L); 
   BUF[1]=Single_Read(GYRO_ADDRESS,GYRO_XOUT_H);
   GYRO_X_DATA= (BUF[1]<<8)|BUF[0];
   GYRO_X=GYRO_X_DATA/16.384f;               //读取计算X轴数据

   BUF[2]=Single_Read(GYRO_ADDRESS,GYRO_YOUT_L);
   BUF[3]=Single_Read(GYRO_ADDRESS,GYRO_YOUT_H);
   GYRO_Y_DATA =  (BUF[3]<<8)|BUF[2];
   GYRO_Y=GYRO_Y_DATA/16.384f;               //读取计算Y轴数据
   BUF[4]=Single_Read(GYRO_ADDRESS,GYRO_ZOUT_L);
   BUF[5]=Single_Read(GYRO_ADDRESS,GYRO_ZOUT_H);
   GYRO_Z_DATA =  (BUF[5]<<8)|BUF[4];
   GYRO_Z=GYRO_Z_DATA/16.384f;                 //读取计算Z轴数据
 
 
  // BUF[6]=Single_Read(GYRO_ADDRESS,TEMP_OUT_L); 
  // BUF[7]=Single_Read(GYRO_ADDRESS,TEMP_OUT_H); 
  // T_T=(BUF[7]<<8)|BUF[6];
  // T_T = 35+ ((double) (T_T + 13200)) / 280;// 读取计算出温度
}

void READ_MPU9250_MAG(void)
{ 
//  short MAG_X_DATA,MAG_Y_DATA,MAG_Z_DATA;
   Single_Write(GYRO_ADDRESS,0x37,0x02);//turn on Bypass Mode 
   delay_ms(10);  
   Single_Write(MAG_ADDRESS,0x0A,0x01);
   delay_ms(10);  
   BUF[0]=Single_Read (MAG_ADDRESS,MAG_XOUT_L);
   BUF[1]=Single_Read (MAG_ADDRESS,MAG_XOUT_H);
   MAG_X=(BUF[1]<<8)|BUF[0];

   BUF[2]=Single_Read(MAG_ADDRESS,MAG_YOUT_L);
   BUF[3]=Single_Read(MAG_ADDRESS,MAG_YOUT_H);
   MAG_Y= (BUF[3]<<8)|BUF[2];
                 //读取计算Y轴数据
   
   BUF[4]=Single_Read(MAG_ADDRESS,MAG_ZOUT_L);
   BUF[5]=Single_Read(MAG_ADDRESS,MAG_ZOUT_H);
   MAG_Z= (BUF[5]<<8)|BUF[4];
                 //读取计算Z轴数据
}

/*****************************************************************************/
//#define Kp      2.0f
//#define Ki      0.005f
#define halfT   0.005f    //100Hz,10ms
#define FACTOR  0.002f

/**
*@breif:四元素
*/
static float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;

/**
*@breif:开平方函数，速度比math.h中的快
*/
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

/**
*@breif:结算姿态角
*@param accel :用于传入加速度数据，单位：m^2/s
*@param gyro  :用于传入角速度数据，单位：弧度/s
*@param angle :用于返回解算出来的欧拉角数据，单位：度
*/
void imu6AxisUpdate(float *accel,float *gyro,float *angle)
{
  float norm;
  float vx, vy, vz;
  float ex, ey, ez;

  float delta_2=0;

  float ax = accel[0];
  float ay = accel[1];
  float az = accel[2];

  float gx = gyro[0];
  float gy = gyro[1];
  float gz = gyro[2];
    // normalise the measurements
  norm = invSqrt(ax*ax + ay*ay + az*az);
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;

  // estimated direction of gravity
  vx = 2*(q1*q3 - q0*q2);
  vy = 2*(q0*q1 + q2*q3);
  vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  // error is sum of cross product between reference direction of field and direction measured by sensor
  ex = (ay*vz - az*vy);
  ey = (az*vx - ax*vz);
  ez = (ax*vy - ay*vx);

  gx = gx + ex*FACTOR/halfT;
  gy = gy + ey*FACTOR/halfT;
  gz = gz + ez*FACTOR/halfT;

  delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);

  q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    // normalise quaternion
  norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;

  // roll
  angle[0]  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.295780f;
  // pitch
  angle[1] = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.295780f;
  // yaw
  angle[2]   = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 57.295780f;
}

bool TM_I2C_ReadMulti(uint8_t device_address, uint8_t register_address, uint8_t* data, uint16_t count)
{
      uint16_t i;
  if(!I2C_Start())return FALSE;
    I2C_SendByte(device_address); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop();test=1; return FALSE;}
    I2C_SendByte((u8) register_address);   //设置低起始地址      
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(device_address+1);
    I2C_WaitAck();
    for(i=0;i<(count-1);i++)
    {
      *data= I2C_RadeByte();
      data++;
      I2C_Ack();
    }
    *data= I2C_RadeByte();
    I2C_NoAck();
    I2C_Stop();
    //return TRUE;
  return TRUE;
}


