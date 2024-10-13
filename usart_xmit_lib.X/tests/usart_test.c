#include "CppUTest/TestHarness.h"
#include "usart.h"
#include "mock_xc.h" // Mock for xc.h functions

TEST_GROUP(USART)
{
    void setup() {
        // Initialize or reset any necessary variables, mock expectations, etc.
    }

    void teardown() {
        // Clean up after tests
    }
};

TEST(USART, TestUartSetupTx) {
    // Call the function
    uart_setup_tx();

    // Validate register settings (use mocks)
    CHECK_EQUAL(0, ANSELCbits.ANSC4); // Check ANSELCbits.ANSC4 = 0
    CHECK_EQUAL(0b10100, RC4PPS); // Check RC4PPS is set correctly
    CHECK_EQUAL(1, TX1STAbits.BRGH);
    CHECK_EQUAL(1, BAUD1CONbits.BRG16);
    CHECK_EQUAL(0, TX1STAbits.SYNC);
    CHECK_EQUAL(68, SPBRGL);
    CHECK_EQUAL(1, RC1STAbits.SPEN);
    CHECK_EQUAL(1, TX1STAbits.TXEN);
}

TEST(USART, TestUsartPrepSigned32Msg) {
    int32_t val = -12345;
    const char *label = "Label";
    
    usart_prep_signed_32_msg(val, label);
    
    // Validate TX_MSG is correctly prepared
    STRCMP_EQUAL("Label -12345     \r\n", TX_MSG);
}

TEST(USART, TestUsartXmitStr) {
    char test_msg[] = "Hello World!\r\n";
    usart_xmit_str(test_msg);
    
    // Validate that the message was transmitted correctly (check mock functions)
    // Example: Check that TX1REG has been filled with 'H', 'e', 'l', etc.
}

TEST(USART, TestUsartXmitSigned32Msg) {
    usart_xmit_signed_32_msg();
    
    // Validate that TX_MSG was transmitted correctly
}

TEST(USART, TestUsartStat) {
    usart_stat('U', 5);
    
    // Validate TX_MSG = "5 U \r\n"
    STRCMP_EQUAL("5 U              \r\n", TX_MSG);
}


