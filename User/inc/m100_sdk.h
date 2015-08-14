#ifndef _M100_SDK_H
#define _M100_SDK_H

uint16_t sdk_stream_crc16_calc(const uint8_t* pMsg, uint32_t nLen);

uint32_t sdk_stream_crc32_calc(const uint8_t* pMsg, uint32_t nLen);


#endif
