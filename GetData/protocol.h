#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * 帧结构:
 *   | SOF1 | SOF2 | Length | Type | Payload[0~252] | CRC16_HI | CRC16_LO |
 *   | 0xAA | 0x55 |   1B   |  1B  |      n B       |    1B    |    1B    |
 *
 *   - Length = Type + Payload + CRC16 = n + 3, 有效范围 3~255
 *   - CRC16-CCITT: poly=0x1021, init=0xFFFF, 无反射, 无终异或,
 *     计算范围 Type..Payload 末字节, 高字节在前发送
 *   - Payload 内多字节字段一律小端序
 * ========================================================================= */

#define PROTOCOL_SOF1          (0xAAU)
#define PROTOCOL_SOF2          (0x55U)
#define PROTOCOL_MIN_LEN       (3U)     /* Type + CRC16, 空 Payload 帧 */
#define PROTOCOL_MAX_PAYLOAD   (252U)
#define PROTOCOL_MAX_BODY      (255U)   /* Type + Payload + CRC16 最大长度 */
#define PROTOCOL_MAX_FRAME     (258U)   /* SOF(2) + Length(1) + Body(255) */

typedef enum
{
    PROTOCOL_PARSER_WAIT_SOF1 = 0,
    PROTOCOL_PARSER_WAIT_SOF2,
    PROTOCOL_PARSER_WAIT_LEN,
    PROTOCOL_PARSER_WAIT_BODY
} Protocol_ParserState_t;

typedef struct
{
    uint32_t rx_bytes;
    uint32_t valid_frames;
    uint32_t crc_errors;
    uint32_t length_errors;
} Protocol_Stats_t;

typedef struct
{
    uint8_t  state;          /* Protocol_ParserState_t */
    uint8_t  type;           /* 最近一帧的 Type, 调试用 */
    uint8_t  len;            /* 当前帧 Length 字节 */
    uint8_t  idx;            /* 当前帧 body 已收字节数 */
    uint8_t *body_buffer;    /* 调用方提供存储, 容量 body_capacity */
    uint8_t  body_capacity;  /* 建议取 PROTOCOL_MAX_BODY; Length 超过它按长度错误丢弃 */
    Protocol_Stats_t *stats; /* 可为 NULL */
    void    *cb_ctx;         /* 透传给 on_frame */
    void (*on_frame)(void *ctx, uint8_t type, const uint8_t *payload, uint8_t plen);
} Protocol_Parser_t;

/* body_capacity 决定可接收的最大帧长 */
void Protocol_ParserInit(Protocol_Parser_t *parser, uint8_t *body_buffer,
                         uint8_t body_capacity, Protocol_Stats_t *stats);

void Protocol_ParserReset(Protocol_Parser_t *parser);

/* 逐字节喂入; 帧校验通过后在调用上下文中回调 on_frame */
void Protocol_ParserFeed(Protocol_Parser_t *parser, uint8_t byte);

/* 整块喂入(DMA/缓冲区场景), 等价于逐字节 Feed */
void Protocol_ParserFeedBlock(Protocol_Parser_t *parser,
                              const uint8_t *block, uint16_t len);

/* ---------------- 编码 ---------------- */

/* 编码一帧到 out, 返回帧总长度(plen+6);
 * 参数非法或 out_cap 不足时返回 0 */
uint16_t Protocol_EncodeFrame(uint8_t *out, uint16_t out_cap,
                              uint8_t type, const uint8_t *payload, uint8_t plen);

/* CRC16-CCITT(poly=0x1021, init=0xFFFF,) */
uint16_t Protocol_CRC16(const uint8_t *data, uint16_t len);

int16_t  Protocol_ReadS16LE(const uint8_t *data);
uint16_t Protocol_ReadU16LE(const uint8_t *data);
void     Protocol_WriteS16LE(uint8_t *data, int16_t value);
void     Protocol_WriteU16LE(uint8_t *data, uint16_t value);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H */
