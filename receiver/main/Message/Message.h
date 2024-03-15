#ifndef MESSAGE_H_
#define MESSAGE_H_

#include <stdint.h>
#include <stdbool.h>
/**
 * Data Structure Definitions
 * These types are used internally in the program to maintain current state.
 */

/**
 * @details Generic data stores the current state of the buttons in the program
 */
typedef struct Data_t
{
    uint8_t current_state;
    bool momentary_state;
    bool on_state;
    bool strobe_state;
} generic_data_t;

typedef struct received_t
{
    bool button_one_state;
    bool button_two_state;
    bool toggle_state;
    unsigned char mac[6];
    uint8_t paired_receiver_mac[6];
} received_data_t;

typedef struct Pairing_Data_t
{
    unsigned char mac[6];
    char pairing_key[17];
    char paired_receiver_mac[6];
} pairing_data_t;

typedef struct Heartbeat_Data_t
{
    int adc_raw;
    int voltage;
    uint8_t mac[6];
    uint8_t paired_transmitter_mac[6];
} heartbeat_data_t;

extern uint8_t message[33];

/**
 * @brief This function constructs the message that contains the current button state.
 * The message is defined and allocated as a 33 byte array once in the Messages.h file.
 * @param data generic_data_t * Pointer to the data structure holding the current state of the buttons set in input processor.
 * @return void
 */
void build_transmission_message(received_data_t *data);

/**
 * @brief This function constructs the message that contains the pairing key data.
 * The message is defined and allocated as a 33 byte array once in the Messages.h file.
 * @param pairing_key char* Pointer to the pairing key data.
 * @return void
 */
void build_pairing_message(const char *pairing_key);

/**
 * @brief This function constructs the message that contains the heartbeat data. It puts the voltage starting at the
 * first byte of data that being offset 13 bytes from the start.
 * @param voltage int the integer representing the voltage of the device read from adc.
 * @return void
*/
void build_heartbeat_messages(int voltage);


#endif /* MESSAGE_H_ */