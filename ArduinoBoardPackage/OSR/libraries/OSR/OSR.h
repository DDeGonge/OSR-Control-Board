#include <Arduino.h>
#include <SPI.h>

/* Stepper driver object, contains 2 stepper objects */
struct TMC2041
{
    TMC2041(uint8_t en_pin, uint8_t cs_pin, uint8_t step0_pin, uint8_t step1_pin, uint8_t dir0_pin, uint8_t dir1_pin);

    public:
    void enable();
    void disable();

    TMCstep step0(0);
    TMCstep step1(1);

    /* Driver com functions */
    void write_gconf();
    void write_iholdirun(uint8_t motor_index);
    void write_chop(uint8_t motor_index);
    void write_cool(uint8_t motor_index);
    void write_all();


    private:
    uint8_t EN_PIN;
    uint8_t CS_PIN;

    /* SPI functions */
    void write_cmd(uint8_t addr, uint8_t chunk[4]);
    void read_cmd(uint8_t addr);
    SPISettings TMCspiSettings(4000000, MSBFIRST, SPI_MODE3); 
    
    /* Addresses. Tuple if unique address for each stepper */
    uint8_t a_gconf = 0x00;
    uint8_t a_iholdirun[2] = {0x30, 0x50};
    uint8_t a_chop[2] = {0x6C, 0x7C};
    uint8_t a_cool[2] = {0x6D, 0x7D};

    /* Driver level bitfields */
    uint8_t gconf[4] = {B00000000, B00000000, B00000000, B00000110};
};

/* Stepper object */
struct TMCstep
{
    TMCstep(uint8_t index);
    TMCstep(uint8_t index, uint8_t step_pin, uint8_t dir_pin);

    public:
    void set_pins(uint8_t step_pin, uint8_t dir_pin);
    void set_index(uint8_t index);
    uint8_t get_index();

    void step();
    int32_t get_step();
    void set_dir(bool dir);
    bool get_dir();
    

    private:
    uint8_t INDEX;
    uint8_t STEP_PIN;
    uint8_t DIR_PIN;
    uint16_t step_pulse_len_us = 1;

    int32_t step_count = 0;
    bool direction = false;

    /* Motor level bitfields */
    uint8_t ihold[4] = {B00000000, B00000001, B00010000, B00001000};
    uint8_t chop[4]  = {B00000000, B00000001, B00000000, B00000011};
    uint8_t cool[4]  = {B00000000, B00000000, B00000000, B00000000};
};