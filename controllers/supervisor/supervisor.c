#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <limits.h>
#include "supervisor.h"

static int time_step=50;
static WbDeviceTag emitter;   // to send genes to robot
//static const char *FILE_NAME = "fittest.txt";
//const char *controller_args0;  // a translation needs 3 doubles


int main() {
  wb_robot_init();
  
  emitter = wb_robot_get_device("emitter");
  wb_emitter_set_channel(emitter, 1);
  WbDeviceTag receiver;
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, time_step);
  wb_receiver_set_channel(receiver, 1);

  //read configuration files
  Configuration config;
  if(ReadConfigurationFile(&config))
  {
      printf("Done or config files unreadable!");
      char* someString = "We're gonna stop... thanks for playing.";
      wb_emitter_send(emitter, &someString, strlen(someString)+1);
      wb_robot_step(500);
      return 1;
  }

  wb_robot_step(500);
  wb_emitter_send(emitter,&config,sizeof(Configuration));
 
  wb_robot_step(100);
  char str[3];
  sprintf(str, "go");
  wb_emitter_send(emitter, str, strlen(str) + 1);
  wb_robot_step(50);
  
   printf("length data que super %d\n" , wb_receiver_get_queue_length(receiver));
  wb_robot_step(50);
  int number_ho=0;
	
  while (number_ho < 6)
  {
    while (wb_receiver_get_queue_length(receiver) > 0)
    {
       //printf("something in que \n");
       if(wb_receiver_get_data_size(receiver)!=3)
       {
            wb_robot_step(50);
	  //printf("not of correct size\n");

       } else 
         {
           char* data= (char*)wb_receiver_get_data(receiver);
           wb_robot_step(50);
           //printf("data %c %c \n", data[0], data[1]);
           if(data[0] != 'h' || data[1] != 'o')
            {
             //printf("wrong letters \n");
            } else
              {
                number_ho++;
              }     
         }
         
       wb_receiver_next_packet(receiver);
       wb_robot_step(50);
  
    }
    wb_robot_step(50);
  }
  
  printf("number stop %d\n", number_ho);
  
  if(number_ho==6)
  {
  printf("REVERT\n");
    wb_supervisor_simulation_revert();
  }
  
  return 0;
}

char* configFileNames[] = {"Configs/Experiment16_ESYT00.txt"};
int noExperiments = 1;
int repeatExperiments = 50;

int ReadConfigurationFile(Configuration* config)
{
    const char* currentConfigFileName = "currentConfig.txt";
    
    FILE* fp;
    char line[LINE_MAX];
    int configNo = 0, configrep = 0;
    
    fp = fopen(currentConfigFileName,"r");
    if(fp != NULL)
    {
        while(fgets(line,LINE_MAX,fp) != NULL)
        {
            if(line[0] == ('0'+configNo))
            {
                configrep++;
            }
            if(configrep >= repeatExperiments || line[0] > ('0'+configNo))
            {
                configNo++;
                configrep = 0;
            }
        }
    }
    fclose(fp);
    
    if(configNo >= noExperiments)
    {
      printf("Reached end of all experiments.");
      return 1;
    }
    
    fp = fopen(configFileNames[configNo],"r");
    if(fp == NULL)
    {
        printf("Could not open file %s\n",configFileNames[configNo]);
        return 1;
    }
    
    while(fgets(line,LINE_MAX,fp) != NULL)
    {
        evaluateLine(line,config);
    }
    
    fclose(fp);
    
    fp = fopen("currentConfig.txt","a+");
    if(fp == NULL)
    {
        printf("WARNING: Could not open file currentConfig.txt. Opening the same configuration file again on same run\n");
        return 0;
    }
    
    char* configLine = "%d\n";
    sprintf(line,configLine,configNo);
    fprintf(fp,line);
    fclose(fp);
    return 0;
}

void evaluateLine(char* line, Configuration* config)
{
    char configElement[LINE_MAX];
    int i;
    //Comment or EOF or newline
    if(line[0] == '#' || line[0] == '\0' || line[0] == '\n')
        return;
    
    for(i = 0; i < LINE_MAX;i++)
    {
        if(line[i] != ':')
        {
            configElement[i] = line[i];
        }
        else
            break;
    }
    // +1 for ':' +1 for ' '
    int startFrom = i+2;
    configElement[i] = '\0';
    
    if(strcmp(configElement,"disable_sensor") == 0)
    {
        printf("%s: ",configElement);
        getValueText(&line[startFrom],configElement);
        config->disable_sensor = atof(configElement);
        printf("%s\n",configElement);
    }
    else if(strcmp(configElement,"evolution")==0)
    {
         printf("%s: ",configElement);
        getValueText(&line[startFrom],configElement);
        config->evolution = atoi(configElement);
        printf("%s\n",configElement);
    }
    else if(strcmp(configElement,"sociallearning")==0)
    {
         printf("%s: ",configElement);
        getValueText(&line[startFrom],configElement);
        config->sociallearning = atoi(configElement);
        printf("%s\n",configElement);
    }
    else if(strcmp(configElement,"threshold")==0)
    {
         printf("%s: ",configElement);
        getValueText(&line[startFrom],configElement);
        config->threshold = atof(configElement);
        printf("%s\n",configElement);
    }
    else if(strcmp(configElement,"total_evals")==0)
    {
         printf("%s: ",configElement);
        getValueText(&line[startFrom],configElement);
        config->total_evals = atoi(configElement);
        printf("%s\n",configElement);
    }
    else if(strcmp(configElement,"max_robot_lifetime")==0)
    {
         printf("%s: ",configElement);
        getValueText(&line[startFrom],configElement);
        config->max_robot_lifetime = atoi(configElement);
        printf("%s\n",configElement);
    }
    else if(strcmp(configElement,"genome_tournament_size")==0)
    {
         printf("%s: ",configElement);
        getValueText(&line[startFrom],configElement);
        config->genome_tournament_size = atoi(configElement);
        printf("%s\n",configElement);
    }
    else if(strcmp(configElement,"mutate_sensor")==0)
    {
         printf("%s: ",configElement);
        getValueText(&line[startFrom],configElement);
        config->mutate_sensor = atof(configElement);
        printf("%s\n",configElement);
    }
    else if(strcmp(configElement,"experimentname")==0)
    {
        printf("%s: ",configElement);
        getValueText(&line[startFrom],configElement);
        strcpy(config->experimentname,configElement);
        printf("\"%s\"\n",configElement);
    }
    else
    {
        printf("Unknown element %s in configuration file!\n",configElement);
    }
}

void getValueText(char* line,char* out)
{
    int i;
    for(i=0;i<LINE_MAX;i++)
    {
        if(line[i] == '\n' || line[i] == '\0')
        {
            break;
        }
        out[i] = line[i];
    }
    
    out[i] = '\0';
}