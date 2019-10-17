#include "mavlink.h"

mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];

mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];