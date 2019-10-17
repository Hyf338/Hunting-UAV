//环形缓冲区

#pragma once

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define RingBuf( TYPE ) \
typedef struct \
{ \
	TYPE* buffer; \
	unsigned short length; \
	\
	unsigned short read_pos; \
	unsigned short write_pos; \
}RingBuf_##TYPE
RingBuf( float );
RingBuf( uint8_t );

//初始化环形缓冲区
#define RingBuf_init( TYPE ) \
void RingBuf_##TYPE##_init( RingBuf_##TYPE* ringbuf , TYPE* buffer , unsigned short length ) \
{ \
	ringbuf->buffer = buffer; \
	ringbuf->length = length; \
	ringbuf->read_pos = ringbuf->write_pos = 0; \
}
static inline RingBuf_init( float )
static inline RingBuf_init( uint8_t )

//压入新的数据
#define RingBuf_push( TYPE ) \
void RingBuf_##TYPE##_push( RingBuf_##TYPE* ringbuf , TYPE data ) \
{ \
	ringbuf->buffer[ ringbuf->write_pos ] = data; \
	uint16_t temp_write_pos = ringbuf->write_pos + 1; \
	if( temp_write_pos >= ringbuf->length ) \
		temp_write_pos = 0; \
	\
	/*数据已满*/ \
	if( temp_write_pos == ringbuf->read_pos ) \
	{	\
		if( ringbuf->read_pos >= ringbuf->length - 1 ) \
			ringbuf->read_pos = 0; \
		else \
			++ringbuf->read_pos; \
	} \
	ringbuf->write_pos = temp_write_pos; \
}
static inline RingBuf_push( float )
static inline RingBuf_push( uint8_t )

//读出最早的数据
#define RingBuf_pop( TYPE ) \
TYPE RingBuf_##TYPE##_pop( RingBuf_##TYPE* ringbuf )\
{ \
	/*无最新数据*/ \
	/*返回上一次的数据*/ \
	if( ringbuf->read_pos == ringbuf->write_pos ) \
	{ \
		unsigned short pos = ringbuf->read_pos; \
		if( pos >= 1 ) \
			pos -= 1; \
		else \
			pos = ringbuf->length - 1; \
		return ringbuf->buffer[ pos ]; \
	} \
	\
	float data = ringbuf->buffer[ ringbuf->read_pos ]; \
	if( ringbuf->read_pos >= ringbuf->length - 1 ) \
		ringbuf->read_pos = 0; \
	else \
		++ringbuf->read_pos; \
	return data; \
}
static inline RingBuf_pop( float )
static inline RingBuf_pop( uint8_t )

//读取待读元素个数
#define RingBuf_get_Bytes2read( TYPE ) \
uint16_t RingBuf_##TYPE##_get_Bytes2read( RingBuf_##TYPE* ringbuf )\
{ \
	if( ringbuf->write_pos >= ringbuf->read_pos ) \
		return ringbuf->write_pos - ringbuf->read_pos; \
	else \
		return ringbuf->write_pos + ringbuf->length - ringbuf->read_pos; \
}
static inline RingBuf_get_Bytes2read( float )
static inline RingBuf_get_Bytes2read( uint8_t )
	
//读取剩余空间
#define RingBuf_get_Freesize( TYPE ) \
uint16_t RingBuf_##TYPE##_get_Freesize( RingBuf_##TYPE* ringbuf )\
{ \
	if( ringbuf->write_pos >= ringbuf->read_pos ) \
		return ringbuf->length - ringbuf->write_pos + ringbuf->read_pos; \
	else \
		return ringbuf->read_pos - ringbuf->write_pos; \
}
static inline RingBuf_get_Freesize( float )
static inline RingBuf_get_Freesize( uint8_t )

//读取n个之前的数据
#define RingBuf_GetHis( TYPE ) \
TYPE RingBuf_##TYPE##_GetHis( RingBuf_##TYPE* ringbuf , unsigned short n ) \
{ \
	++n; \
	if( n >= ringbuf->length - 1 ) \
		n = ringbuf->length - 1; \
	\
	unsigned short pos = ringbuf->write_pos; \
	if( pos >= n ) \
		return ringbuf->buffer[ pos - n ]; \
	else \
		return ringbuf->buffer[ pos + ringbuf->length - n ]; \
}
static inline RingBuf_GetHis(float)
static inline RingBuf_GetHis(uint8_t)

#define RingBuf_GetHis_Pointer( TYPE ) \
TYPE* RingBuf_##TYPE##_GetHis_Pointer( RingBuf_##TYPE* ringbuf , unsigned short n ) \
{ \
	++n; \
	if( n >= ringbuf->length ) \
		n = ringbuf->length; \
	\
	unsigned short pos = ringbuf->write_pos; \
	if( pos >= n ) \
		return &ringbuf->buffer[ pos - n ]; \
	else \
		return &ringbuf->buffer[ pos + ringbuf->length - n ]; \
}
static inline RingBuf_GetHis_Pointer(float)
static inline RingBuf_GetHis_Pointer(uint8_t)


//压出用于DMA发送/接收的连续缓冲数据
#define RingBuf_pop_DMABuf( TYPE ) \
TYPE* RingBuf_##TYPE##_pop_DMABuf( RingBuf_##TYPE* ringbuf , uint16_t* length )\
{ \
	uint16_t p = ringbuf->read_pos; \
	if( ringbuf->write_pos >= ringbuf->read_pos ) \
	{ \
		*length = ringbuf->write_pos - ringbuf->read_pos; \
		ringbuf->read_pos = ringbuf->write_pos; \
		return &ringbuf->buffer[ p ]; \
	} \
	else \
	{ \
		*length = ringbuf->length - ringbuf->read_pos; \
		ringbuf->read_pos = 0; \
		return &ringbuf->buffer[ p ]; \
	} \
}
static inline RingBuf_pop_DMABuf(float)
static inline RingBuf_pop_DMABuf(uint8_t)
	
//压出指定长度的缓冲数据
#define RingBuf_pop_length( TYPE ) \
uint16_t RingBuf_##TYPE##_pop_length( RingBuf_##TYPE* ringbuf , TYPE* data , uint16_t length  )\
{ \
	uint16_t bytes2read = RingBuf_##TYPE##_get_Bytes2read( ringbuf ); \
	if( length > bytes2read ) \
		length = bytes2read; \
	uint16_t bytes2end = ringbuf->length - ringbuf->read_pos; \
	\
	if( length <= bytes2end ) \
		memcpy( data , &ringbuf->buffer[ ringbuf->read_pos ] , length ); \
	else \
	{ \
		memcpy( data , &ringbuf->buffer[ ringbuf->read_pos ] , bytes2end ); \
		memcpy( &data[bytes2end] , &ringbuf->buffer[0] , length - bytes2end ); \
	} \
	if( ringbuf->read_pos + length >= ringbuf->length ) \
		ringbuf->read_pos = ringbuf->read_pos + length - ringbuf->length; \
	else \
		ringbuf->read_pos = ringbuf->read_pos + length; \
	return length; \
}
static inline RingBuf_pop_length(float)
static inline RingBuf_pop_length(uint8_t)
	

#define RingBuf_push_length( TYPE ) \
void RingBuf_##TYPE##_push_length( RingBuf_##TYPE* ringbuf , const TYPE* data , uint16_t length ) \
{ \
	if( length >= ringbuf->length ) \
		length = ringbuf->length; \
	\
	/*复制*/ \
	uint16_t bytes2end = ringbuf->length - ringbuf->write_pos; \
	if( length <= bytes2end ) \
		memcpy( &ringbuf->buffer[ ringbuf->write_pos ] , data , length ); \
	else \
	{ \
		memcpy( &ringbuf->buffer[ ringbuf->write_pos ] , data , bytes2end ); \
		memcpy( &ringbuf->buffer[ 0 ] , &data[ bytes2end ] , length - bytes2end ); \
	} \
	\
	/*更新写指针位置*/ \
	uint16_t temp_write_pos = ringbuf->write_pos + length; \
	if( temp_write_pos >= ringbuf->length ) \
		temp_write_pos -= ringbuf->length; \
	\
	/*如果写满了，更新读指针位置*/ \
	uint16_t freesize = RingBuf_##TYPE##_get_Freesize( ringbuf ); \
	if( freesize <= length ) \
	{	\
		if( temp_write_pos + 1 >= ringbuf->length ) \
			ringbuf->read_pos = 0; \
		else \
			ringbuf->read_pos = temp_write_pos + 1; \
	}\
	\
	ringbuf->write_pos = temp_write_pos; \
}
static inline RingBuf_push_length(float)
static inline RingBuf_push_length(uint8_t)