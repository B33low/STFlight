typedef struct {
    float u;
    float time_s;  
} SimulatorInput;

typedef struct {
    float ax, ay, az;
    float vx, vy, vz;
    float px, py, pz;
    float time_s;  
} SimulatorState;

typedef struct {
    float ax, ay, az;
    float time_s;   
} SimulatorOutput;



void sim_step(SimulatorState* sim_state, SimulatorInput* sim_in, SimulatorOutput* sim_out);
void sim_init(SimulatorState* sim_state);