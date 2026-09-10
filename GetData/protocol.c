#include "protocol.h"

#define P_STATS_INC(parser, field)  \
    do {                            \
        if ((parser)->stats != NULL) \
        {                            \
            (parser)->stats->field++; \
        }                            \
    } while (0)

uint16_t Protocol_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    uint8_t bit;

    if ((data == NULL) && (len > 0U))
    {
        return crc;
    }

    for (index = 0U; index < len; index++)
    {
        crc ^= (uint16_t)((uint16_t)data[index] << 8U);
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc = (uint16_t)(crc << 1U);
            }
        }
    }

    return crc;
}

void Protocol_ParserInit(Protocol_Parser_t *parser, uint8_t *body_buffer,
                         uint8_t body_capacity, Protocol_Stats_t *stats)
{
    if (parser == NULL)
    {
        return;
    }

    parser->state = (uint8_t)PROTOCOL_PARSER_WAIT_SOF1;
    parser->type = 0U;
    parser->len = 0U;
    parser->idx = 0U;
    parser->body_buffer = body_buffer;
    parser->body_capacity = body_capacity;
    parser->stats = stats;
    parser->cb_ctx = NULL;
    parser->on_frame = NULL;

    if (stats != NULL)
    {
        stats->rx_bytes = 0U;
        stats->valid_frames = 0U;
        stats->crc_errors = 0U;
        stats->length_errors = 0U;
    }
}

void Protocol_ParserReset(Protocol_Parser_t *parser)
{
    if (parser == NULL)
    {
        return;
    }

    parser->state = (uint8_t)PROTOCOL_PARSER_WAIT_SOF1;
    parser->len = 0U;
    parser->idx = 0U;
}

void Protocol_ParserFeed(Protocol_Parser_t *parser, uint8_t byte)
{
    uint16_t calculated_crc;
    uint16_t received_crc;
    uint8_t payload_length;

    if ((parser == NULL) || (parser->body_buffer == NULL) ||
        (parser->on_frame == NULL))
    {
        return;
    }

    if (parser->stats != NULL)
    {
        parser->stats->rx_bytes++;
    }

    switch (parser->state)
    {
    case PROTOCOL_PARSER_WAIT_SOF1:
        if (byte == PROTOCOL_SOF1)
        {
            parser->state = (uint8_t)PROTOCOL_PARSER_WAIT_SOF2;
        }
        break;

    case PROTOCOL_PARSER_WAIT_SOF2:
        if (byte == PROTOCOL_SOF2)
        {
            parser->state = (uint8_t)PROTOCOL_PARSER_WAIT_LEN;
        }
        else if (byte != PROTOCOL_SOF1)
        {
            /* 连续 0xAA 时保持等待 SOF2, 实现 AA AA 55 重同步 */
            parser->state = (uint8_t)PROTOCOL_PARSER_WAIT_SOF1;
        }
        break;

    case PROTOCOL_PARSER_WAIT_LEN:
        if ((byte < PROTOCOL_MIN_LEN) || (byte > parser->body_capacity))
        {
            P_STATS_INC(parser, length_errors);
            Protocol_ParserReset(parser);
            break;
        }
        parser->len = byte;
        parser->idx = 0U;
        parser->state = (uint8_t)PROTOCOL_PARSER_WAIT_BODY;
        break;

    case PROTOCOL_PARSER_WAIT_BODY:
    default:
        parser->body_buffer[parser->idx] = byte;
        parser->idx++;
        if (parser->idx < parser->len)
        {
            break;
        }

        /* body = Type + Payload + CRC16, CRC 覆盖前 len-2 字节, 高字节在前 */
        calculated_crc = Protocol_CRC16(parser->body_buffer,
                                        (uint16_t)(parser->len - 2U));
        received_crc = (uint16_t)(((uint16_t)parser->body_buffer[parser->len - 2U] << 8U) |
                                  parser->body_buffer[parser->len - 1U]);

        if (calculated_crc == received_crc)
        {
            parser->type = parser->body_buffer[0];
            payload_length = (uint8_t)(parser->len - 3U);
            P_STATS_INC(parser, valid_frames);
            parser->on_frame(parser->cb_ctx, parser->type,
                             &parser->body_buffer[1], payload_length);
        }
        else
        {
            P_STATS_INC(parser, crc_errors);
        }
        Protocol_ParserReset(parser);
        break;
    }
}

void Protocol_ParserFeedBlock(Protocol_Parser_t *parser,
                              const uint8_t *block, uint16_t len)
{
    uint16_t i;

    if ((parser == NULL) || (block == NULL))
    {
        return;
    }

    for (i = 0U; i < len; i++)
    {
        Protocol_ParserFeed(parser, block[i]);
    }
}

uint16_t Protocol_EncodeFrame(uint8_t *out, uint16_t out_cap,
                              uint8_t type, const uint8_t *payload, uint8_t plen)
{
    uint16_t crc;
    uint16_t frame_size;
    uint16_t i;

    if ((out == NULL) || (plen > PROTOCOL_MAX_PAYLOAD) ||
        ((plen > 0U) && (payload == NULL)) ||
        (out_cap < (uint16_t)((uint16_t)plen + 6U)))
    {
        return 0U;
    }

    out[0] = PROTOCOL_SOF1;
    out[1] = PROTOCOL_SOF2;
    out[2] = (uint8_t)(plen + 3U);
    out[3] = type;
    for (i = 0U; i < plen; i++)
    {
        out[4U + i] = payload[i];
    }

    crc = Protocol_CRC16(&out[3], (uint16_t)((uint16_t)plen + 1U));
    out[4U + plen] = (uint8_t)(crc >> 8U);
    out[5U + plen] = (uint8_t)(crc & 0xFFU);
    frame_size = (uint16_t)((uint16_t)plen + 6U);

    return frame_size;
}

int16_t Protocol_ReadS16LE(const uint8_t *data)
{
    return (int16_t)Protocol_ReadU16LE(data);
}

uint16_t Protocol_ReadU16LE(const uint8_t *data)
{
    uint16_t value;

    if (data == NULL)
    {
        return 0U;
    }

    value = (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
    return value;
}

void Protocol_WriteS16LE(uint8_t *data, int16_t value)
{
    Protocol_WriteU16LE(data, (uint16_t)value);
}

void Protocol_WriteU16LE(uint8_t *data, uint16_t value)
{
    if (data == NULL)
    {
        return;
    }

    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8U);
}
