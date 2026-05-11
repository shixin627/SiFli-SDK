
#include "gh30x_example_config.h"
#if ((!__HBD_HB_ALGORITHM_ENABLE__) && (!__HBD_HRV_ALGORITHM_ENABLE__) && (!__HBD_SPO2_ALGORITHM_ENABLE__) && (!__HBD_NADT_ALGORITHM_ENABLE__))

typedef unsigned char GU8;
typedef signed char GS8;
typedef unsigned short GU16;
typedef signed short GS16;

typedef int GS32;
typedef unsigned int GU32;
typedef unsigned long long   GU64;
typedef signed long long GS64;
typedef float GF32;
typedef unsigned char GBOOL;
typedef char                GCHAR;  /**< char type */

const char HBD_VER_STRING[] = "HBD_LIB_v7.8.6\r\n";

#define HBD_PTR_NULL              ((void *) 0)

#define HBD_MAKEUP_DWORD(uchByte3, uchByte2, uchByte1, uchByte0)  (((((GU32)uchByte3) << 24) & 0xFF000000U) | ((((GU32)uchByte2) << 16) & 0x00FF0000U) |\
                                                                     ((((GU32)uchByte1) << 8) & 0x0000FF00U) |(((GU32)uchByte0) & 0x000000FFU))
#define HBD_MAKEUP_DWORD2(usHighWord, usLowWord)                  (((((GU32)usHighWord) << 16) & 0xFFFF0000U) | (((GU32)usLowWord) & 0x0000FFFFU))

GCHAR *Gh30xHBDVersionGet(void)
{
    return (GCHAR *)HBD_VER_STRING;
}

void Gh30xRawdata24BitTo32Bit(GU8 *puchRawdataFifo, GU16 usSamplePointNum, GU16 usSamplePointNumMax)
{
    //copy to tail area
    for(GU16 usCnt = 0; usCnt < (usSamplePointNum*3); usCnt ++)
    {
        puchRawdataFifo[usSamplePointNumMax*4 - usCnt - 1] = puchRawdataFifo[usSamplePointNum*3 - usCnt - 1];
    }
    //24bit to 32 bit, and copy to head area
    for(GU16 usCnt = 0; usCnt < usSamplePointNum; usCnt ++)
    {
        GU16 usByteIndex;
        GU32 unRawdata32bit;
        GU32 unRawDataTransform;
        usByteIndex = (usCnt*3 + usSamplePointNumMax*4 - usSamplePointNum*3);
        unRawdata32bit = HBD_MAKEUP_DWORD(0x00, puchRawdataFifo[usByteIndex], puchRawdataFifo[usByteIndex + 1], puchRawdataFifo[usByteIndex + 2]);
        unRawdata32bit= (unRawdata32bit << 7) & 0xFFFFFF80;
        unRawDataTransform = unRawdata32bit & 0xffffff;
        unRawdata32bit = ((unRawdata32bit & (~0xffffff)) | unRawDataTransform);   //add check bit
        ((GU32*)puchRawdataFifo)[usCnt] = unRawdata32bit;
    }
}

#endif // ((!__HBD_HB_ALGORITHM_ENABLE__) && (!__HBD_HRV_ALGORITHM_ENABLE__) && (!__HBD_SPO2_ALGORITHM_ENABLE__) && (!__HBD_NADT_ALGORITHM_ENABLE__))

