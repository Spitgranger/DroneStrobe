#include "Message.h"

void build_transmission_message(generic_data_t *data)
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