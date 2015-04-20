#ifndef READFILECTRLH
#define READFILECTRLH 0

typedef struct Config
{
    float disable_sensor;
    int evolution;
    int sociallearning;
    float threshold;
    int total_evals;
    int max_robot_lifetime;
    int genome_tournament_size;
    float mutate_sensor;
    char experimentname[10];
} Configuration;

void evaluateLine(char* line, Configuration* config);
int ReadConfigurationFile(Configuration* config);
void getValueText(char* line,char* out);
#endif
