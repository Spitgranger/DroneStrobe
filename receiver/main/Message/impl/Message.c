#include <string.h>
#include "Message.h"

void build_transmission_message(received_data_t *data)
{
    // State byte stores the information related to the button states
    // LSB (Right most) is equal to button b
    // 2 bit is button two, 3 bit is toggle state
    uint8_t state_byte = 0;
    if (data->button_one_state)
    {
        state_byte |= 0x1;
    }
    else if (!data->button_one_state)
    {
        state_byte &= ~(0x1);
    }
    if (data->button_two_state)
    {
        state_byte |= (0x1 << 1);
    }
    else if (!data->button_two_state)
    {
        state_byte &= ~(0x1 << 1);
    }
    if (data->toggle_state)
    {
        state_byte |= (0x1 << 2);
    }
    else if (!data->toggle_state)
    {
        state_byte &= ~(0x1 << 2);
    }
    message[0] = 0x0;
    message[13] = state_byte;
    return;
}

void build_pairing_message(const char *pairing_key)
{
    // Copies the pairing key into the 13 position of the byte array
    memcpy(message + 13, pairing_key, 16);
    // Set the message id byte equal to 1
    message[0] = 1;
}

void build_heartbeat_messages(int voltage)
{
    message[13] = (voltage >> 24) & 0xFF;
    message[14] = (voltage >> 16) & 0xFF;
    message[15] = (voltage >> 8) & 0xFF; 
    message[16] = voltage & 0xFF;
    message[0] = 2;
}
