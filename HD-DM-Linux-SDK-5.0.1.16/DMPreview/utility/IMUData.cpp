#include "IMUData.h"


std::map<IMUData::IMUPacketSymbol, size_t> IMUData::sizeTable = {
	{ FRAME_COUNT, 2},
	{ SUB_SECOND, 2 },
	{ SEC,		  1 },
	{ MIN,		  1 },
	{ HOUR,		  1 },
	{ ACCEL_X,	  2 },
	{ ACCEL_Y,	  2 },
	{ ACCEL_Z,	  2 },
	{ TEMPARATURE,2 },
	{ GYROSCOPE_X,2 },
	{ GYROSCOPE_Y,2 },
	{ GYROSCOPE_Z,2 },
	{ COMPASS_X,2 },
	{ COMPASS_Y,2 },
	{ COMPASS_Z,2 } };

std::map<IMUData::IMUPacketSymbol, size_t> IMUData::sizeTable_DMP = {
	{ FRAME_COUNT, 2 },
	{ SUB_SECOND, 2 },
	{ SEC,		  1 },
	{ MIN,		  1 },
	{ HOUR,		  1 },
	{ ACCEL_X,	  4 },
	{ ACCEL_Y,	  4 },
	{ ACCEL_Z,	  4 },
	{ TEMPARATURE,2 },
	{ GYROSCOPE_X,4 },
	{ GYROSCOPE_Y,4 },
	{ GYROSCOPE_Z,4 },
	{ COMPASS_X,4 },
	{ COMPASS_Y,4 },
	{ COMPASS_Z,4 },
	{ COMPASS_X_TBC,4 },
	{ COMPASS_Y_TBC,4 },
	{ COMPASS_Z_TBC,4 },
	{ ACCURACY_FLAG,1 } };

std::map<IMUData::IMUPacketSymbol, size_t> IMUData::sizeTable_Quaternion = {
    { FRAME_COUNT, 2 },
    { SUB_SECOND, 2 },
    { SEC,		  1 },
    { MIN,		  1 },
    { HOUR,		  1 },
    { QUATERNION_0, 4 },
    { QUATERNION_1, 4 },
    { QUATERNION_2, 4 },
    { QUATERNION_3, 4 },
    { ACCURACY_FLAG,1 } };

IMUData::IMUData()
{
}


IMUData::~IMUData()
{
}

#define PARSE1Byte(buf)  ( *buf )
#define PARSE2Bytes(buf) ( buf[0] + (buf[1] << 8))
#define PARSE3Bytes(buf) ( buf[0] + (buf[1] << 8) + (buf[2] << 16) )
#define PARSE4Bytes(buf) ( buf[0] + (buf[1] << 8) + (buf[2] << 16) + (buf[3] << 24) )

int parseBuff(unsigned char* &buf, size_t size) {
    unsigned char *parseBuf = buf;
    if(size <= 4){
        buf += size;
    }

    if (size == 1) return PARSE1Byte(parseBuf);
    if (size == 2) return PARSE2Bytes(parseBuf);
    if (size == 3) return PARSE3Bytes(parseBuf);
    if (size == 4) return PARSE4Bytes(parseBuf);
    return 0;
}

float parseBuff_Float(unsigned char* &buf, size_t size) {
    unsigned char *parseBuf = buf;
    if(size <= 4){
        buf += size;
    }

    if (size == 1) return PARSE1Byte(parseBuf);
    if (size == 2) return PARSE2Bytes(parseBuf);
    if (size == 3) return PARSE3Bytes(parseBuf);
    if (size == 4) {
        int data = PARSE4Bytes(parseBuf);
        return *((float *)&data);
    }
    return 0;
}

void IMUData::parsePacket(unsigned char* buf, bool normalization)
{	
	_frameCount  =				parseBuff(buf, sizeTable[FRAME_COUNT  ]);
	_subSecond  =				parseBuff(buf, sizeTable[SUB_SECOND	  ]);
	_sec =						parseBuff(buf, sizeTable[SEC		  ]);
	_min =						parseBuff(buf, sizeTable[MIN		  ]);
	_hour  =					parseBuff(buf, sizeTable[HOUR		  ]);
	signed short accelX =		parseBuff(buf, sizeTable[ACCEL_X	  ]);	 
	signed short accelY =		parseBuff(buf, sizeTable[ACCEL_Y	  ]);
	signed short accelZ =		parseBuff(buf, sizeTable[ACCEL_Z	  ]);
	_temprature =				parseBuff(buf, sizeTable[TEMPARATURE  ]);
	signed short gyroScopeX =	parseBuff(buf, sizeTable[GYROSCOPE_X  ]);
	signed short gyroScopeY =	parseBuff(buf, sizeTable[GYROSCOPE_Y  ]);
	signed short gyroScopeZ =	parseBuff(buf, sizeTable[GYROSCOPE_Z  ]);
	signed short compassX =		parseBuff(buf, sizeTable[COMPASS_X]);
	signed short compassY =		parseBuff(buf, sizeTable[COMPASS_Y]);
	signed short compassZ =		parseBuff(buf, sizeTable[COMPASS_Z]);
	 /*
	printf("%d:Time:%2d,%2d,%2d,%5d Acc:%5hd,%5hd,%5hd Temp:%5d Gyro:%5hd,%5hd,%5hd\n",
		_frameCount,
		_hour,_min,_sec,_subSecond,
		_accelX,_accelY,_accelZ,
		_temprature,
		_gyroScopeX,_gyroScopeY,_gyroScopeZ);
		*/
	if (normalization) {
		int max = (1 << 16) / 2; //+32786 ~ -32786 

		_accelX = ((float)accelX / max)*MAX_G;
		_accelY = ((float)accelY / max)*MAX_G;
		_accelZ = ((float)accelZ / max)*MAX_G;

		_gyroScopeX = ((float)gyroScopeX / max)*MAX_DPS;
		_gyroScopeY = ((float)gyroScopeY / max)*MAX_DPS;
		_gyroScopeZ = ((float)gyroScopeZ / max)*MAX_DPS;

		_compassX = compassX * BASE_uT;
		_compassY = compassY * BASE_uT;
		_compassZ = compassZ * BASE_uT;
	}
	else {
		_accelX = accelX;
		_accelY = accelY;
		_accelZ = accelZ;

		_gyroScopeX = gyroScopeX;
		_gyroScopeY = gyroScopeX;
		_gyroScopeZ = gyroScopeZ;

		_compassX = compassX;
		_compassY = compassY;
		_compassZ = compassZ;
	}
	
}

void IMUData::parsePacket_DMP(unsigned char* buf)
{
	_frameCount = parseBuff(buf, sizeTable_DMP[FRAME_COUNT]);
	_subSecond = parseBuff(buf, sizeTable_DMP[SUB_SECOND]);
	_sec = parseBuff(buf, sizeTable_DMP[SEC]);
	_min = parseBuff(buf, sizeTable_DMP[MIN]);
	_hour = parseBuff(buf, sizeTable_DMP[HOUR]);
	_accelX = parseBuff(buf, sizeTable_DMP[ACCEL_X]);
	_accelY = parseBuff(buf, sizeTable_DMP[ACCEL_Y]);
	_accelZ = parseBuff(buf, sizeTable_DMP[ACCEL_Z]);
	_temprature = parseBuff(buf, sizeTable_DMP[TEMPARATURE]);
	_gyroScopeX = parseBuff(buf, sizeTable_DMP[GYROSCOPE_X]);
	_gyroScopeY = parseBuff(buf, sizeTable_DMP[GYROSCOPE_Y]);
	_gyroScopeZ = parseBuff(buf, sizeTable_DMP[GYROSCOPE_Z]);
	_compassX = parseBuff(buf, sizeTable_DMP[COMPASS_X]);
	_compassY = parseBuff(buf, sizeTable_DMP[COMPASS_Y]);
	_compassZ = parseBuff(buf, sizeTable_DMP[COMPASS_Z]);
	_compassX_TBC = parseBuff(buf, sizeTable_DMP[COMPASS_X_TBC]);
	_compassY_TBC = parseBuff(buf, sizeTable_DMP[COMPASS_Y_TBC]);
	_compassZ_TBC = parseBuff(buf, sizeTable_DMP[COMPASS_Z_TBC]);
	_accuracy_FLAG = parseBuff(buf, sizeTable_DMP[ACCURACY_FLAG]);

}

void IMUData::parsePacket_Quaternion(unsigned char * buf)
{
    _frameCount = parseBuff(buf, sizeTable_Quaternion[FRAME_COUNT]);
    _subSecond = parseBuff(buf, sizeTable_Quaternion[SUB_SECOND]);
    _sec = parseBuff(buf, sizeTable_Quaternion[SEC]);
    _min = parseBuff(buf, sizeTable_Quaternion[MIN]);
    _hour = parseBuff(buf, sizeTable_Quaternion[HOUR]);
    _quaternion[0] = parseBuff_Float(buf, sizeTable_Quaternion[QUATERNION_0]);
    _quaternion[1] = parseBuff_Float(buf, sizeTable_Quaternion[QUATERNION_1]);
    _quaternion[2] = parseBuff_Float(buf, sizeTable_Quaternion[QUATERNION_2]);
    _quaternion[3] = parseBuff_Float(buf, sizeTable_Quaternion[QUATERNION_3]);
    _accuracy_FLAG = parseBuff(buf, sizeTable_Quaternion[ACCURACY_FLAG]);
}
