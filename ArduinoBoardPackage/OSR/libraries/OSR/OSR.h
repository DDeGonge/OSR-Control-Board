#include <Arduino.h>
#include <SPI.h>
#include <vector>

#define NOVALUE 999999


/* Stepper object */
struct TMCstep
{
    public:
    void set_pins(uint8_t step_pin, uint8_t dir_pin);
    void set_index(uint8_t index);
    uint8_t get_index();

    void step();
    void set_step(int32_t steps);
    int32_t get_step();
    void set_dir(bool dir);
    bool get_dir();

    void update_status(uint32_t new_status);

    /* Motor level bitfields */
    // TODO these should probably be private
    uint8_t ihold[4] = {B00000000, B00000001, B00010000, B00001000};
    uint8_t chop[4]  = {B00000010, B00000001, B00000000, B00000011};
    uint8_t cool[4]  = {B00000000, B00000000, B00000000, B00000000};


    private:
    uint8_t INDEX;
    uint8_t STEP_PIN;
    uint8_t DIR_PIN;
    uint16_t step_pulse_len_us = 1;

    int32_t step_count = 0;
    bool motor_dir = false;

    uint32_t status_bits;
};

/* Stepper driver object, contains 2 stepper objects */
struct TMC2041
{
    TMC2041(uint8_t en_pin, uint8_t cs_pin, uint8_t step0_pin, uint8_t step1_pin, uint8_t dir0_pin, uint8_t dir1_pin);

    public:
    void enable();
    void disable();

    TMCstep motor0;
    TMCstep motor1;

    /* Driver com functions */
    void write_gconf();
    void write_iholdirun(uint8_t motor_index);
    void write_chop(uint8_t motor_index);
    void write_cool(uint8_t motor_index);
    void write_all();
    void update_driver_status(uint8_t motor_index);


    private:
    uint8_t EN_PIN;
    uint8_t CS_PIN;

    /* SPI functions */
    void write_cmd(uint8_t addr, uint8_t chunk[4]);
    int32_t read_cmd(uint8_t addr);
    SPISettings TMCspiSettings = SPISettings(4000000, MSBFIRST, SPI_MODE3); 
    
    /* Addresses. Tuple if unique address for each stepper */
    uint8_t a_gconf = 0x00;
    uint8_t a_iholdirun[2] = {0x30, 0x50};
    uint8_t a_chop[2] = {0x6C, 0x7C};
    uint8_t a_cool[2] = {0x6D, 0x7D};
    uint8_t a_status[2] = {0x6F, 0x7F};

    /* Driver level bitfields */
    uint8_t gconf[4] = {B00000000, B00000000, B00000000, B00000110};
};

/* Stepper motor wrapper to handle motion profiles */
struct motorDrive
{
    motorDrive(TMCstep &new_stepper, int32_t steps_per_mm_new);

    public:
    // Configuration functions
    void set_default_vel_mmps(float max_vel_new);
    void set_default_acc_mmps2(float max_accel_new);

    // Zeroing & Other Functions
    void set_current_pos_mm(double target);
    void zero();
    double get_current_pos_mm();
    double get_current_vel_mmps();

    // Async move related supporting real time target adjustment. Limit of ~20KHz step speed
    void set_pos_target_mm_async(double target, float feedrate = NOVALUE);
    bool step_if_needed();
    
    // Standard Synchronous move
    void set_pos_target_mm_sync(double target, float feedrate = NOVALUE);
    
    // Pre-planned async move. Runs way faster than the realtime version
    void plan_move(double target, float feedrate = NOVALUE);
    void execute_move_async();
    bool async_move_step_check(uint32_t t_now = micros());


    private:
    TMCstep stepper;

    // Configuration
    float steps_per_mm = 160;
    float max_vel = 100;
    float max_accel = 500;
    float step_size_mm;

    // Backend funcs
    void quad_solve(double &t_0, double &t_1, double a, double b, double c);
    int32_t solve_for_t_us(float v, float a, float d);

    // Control vars
    double target_mm = 0;

    // Async Motion tracking vars
    double current_velocity = 0;
    double diff_exact_us = 0;
    uint32_t last_step_us = 0;
    uint32_t next_step_us = 0;

    // Sync motion tracking vars
    int8_t plan_sign;
    int32_t plan_tmin;
    uint32_t plan_asteps;
    uint32_t plan_nsteps;
    std::vector<uint16_t> plan_accel_timings;

    uint32_t plan_stepstaken;
    uint32_t plan_nextstep_us;
};