/*
 * Authors: Jaqueline Heinerman & Dexter Drupsteen
 * Date last modified: 20-4-2015
 * Description: Three fold adaptivity algorithm
 */

// Included libraries
#include "three_fold_adaptivity.h" //obtain main library of webots

int motorspeed[2] = { 0, 0 };
WbDeviceTag ps[NB_DIST_SENS];	/* proximity sensors */ //WbDeviceTag
int ps_value[NB_DIST_SENS]={0,0,0,0,0,0,0,0};

//parameters for experiment
double disable_sensor = 0.0;   // change that sensor is enabled
int evolution = 0; // evolution
int sociallearning = 1; // social learning
double threshold = 0.1 ;
int total_evals = 200; //200 or 800
int max_robot_lifetime = 200; //either 200 or 100
int genome_tournament_size = 2;
double mutate_sensor = 0.05; //bitflip after uniform 
char experimentname[10];


//genome parameters
int collected_genomes_total = 0; 
const int collected_genomes_max = 7;
//int collected_genomes[5][8]; // maximal number of collected genomes is pop size -1, same genome?

// phenotye/memotype parameters
double range_weights = 4;    // range of the weights. between [-range_weights, range_weights]
int collected_sensor_weights_total = 0; //
const int collected_sensor_weights_max = 20; //
//double collected_sensor_weights[20][3]; // social learning genomes : sensor, weights
double sigmainitial = 1;	// initital sigma
double sigma_max = 4;
double sigma_min = 0.01;
double sigma_increase = 2;
double seed;

// parameters for action reevaluate
int tau = 30;		//Recovery  period tau, in steps
int evaltime = 175;          //Evaluation time, in steps
double re_weight = 0.8;    //part or reevaluation fitness that stays the same
int restart_max = 5; // restart indicator
double max_fitness = 22400; // max fitness (30 times 1)

// Eanble/disable adaptive mechanisms
int lifetimelearning = 1; // lifetimelearning

// changes for actions in model setup (re-evalutaion vs try new weights)
double set_communication_range = 20.0; // meters? 

// Memory for the received messages
RobotPhenotypeDataMessage collected_phenotypes[collected_sensor_weights_max];
RobotGenotypeDataMessage collected_genotypes[collected_genomes_max];

int no_bots = 1;
const char* bots[] = {"1001","1002"};

candidate champion;

static void init() 
{
            //printf("Initializing\n");
        	  // get distance sensors
        	  char textPS[] = "ps0";
        	  int it=0;
        	  for (it=0;it<NB_DIST_SENS;it++)
        	  {
        	    ps[it] = wb_robot_get_device(textPS);
        	    textPS[2]++;
        	  }
        
        	  for(it=0;it<NB_DIST_SENS;it++) 
        	  {
        	    wb_distance_sensor_enable(ps[it],TIME_STEP);
        	  }
}

int prepareExperiment()
{
    WbDeviceTag receiver;
    receiver = wb_robot_get_device("receiver");
    
    printf("length data que %d\n" , wb_receiver_get_queue_length(receiver));
// Get the configuration structure
    while (wb_receiver_get_queue_length(receiver) ==0)
    {
       wb_robot_step(TIME_STEP);
    }
    printf("length data size %d, %ld\n" , wb_receiver_get_data_size(receiver),sizeof(Configuration));
	
    if(wb_receiver_get_data_size(receiver)!= sizeof(Configuration))
    {
      return 1;
    }
	  
    Configuration* config = (Configuration*)wb_receiver_get_data(receiver);
    
    disable_sensor = config->disable_sensor;
    evolution = config->evolution;
    sociallearning = config->sociallearning;
    threshold = config->threshold;
    total_evals = config->total_evals;
    max_robot_lifetime = config->max_robot_lifetime;
    genome_tournament_size = config->genome_tournament_size;
    mutate_sensor = config->mutate_sensor;
    strcpy(experimentname,config->experimentname);
    
    /*printf("ds: %f\n",disable_sensor);
    printf("ev: %d\n",evolution);
    printf("sl: %d\n",sociallearning);
    printf("th: %f\n",threshold);
    printf("te: %d\n",total_evals);
    printf("ml: %d\n",max_robot_lifetime);
    printf("gz: %d\n",genome_tournament_size);
    printf("ms: %f\n",mutate_sensor);
    printf("en: %s\n",experimentname);*/
    
    wb_receiver_next_packet(receiver);
//Wait for 'go'
    while (wb_receiver_get_queue_length(receiver) ==0)
	{
	 wb_robot_step(TIME_STEP);
	}
	printf("length data size %d\n" , wb_receiver_get_data_size(receiver));
	
	if(wb_receiver_get_data_size(receiver)!=3)
	{
	  return 1;
	}
	  
	char* data= (char*)wb_receiver_get_data(receiver);
	printf("data %c %c \n", data[0], data[1]);
	if(data[0] != 'g' || data[1] != 'o')
	{
	  return 1;
	}
	wb_receiver_next_packet(receiver);
	return 0;
	
}

int main(int argc, char** args) 
{         
          wb_robot_init();
          WbDeviceTag receiver;
	receiver = wb_robot_get_device("receiver");
         wb_receiver_enable(receiver, TIME_STEP);
	wb_receiver_set_channel(receiver, 1);
	
	init();
	wb_robot_step(TIME_STEP);
	
	if(prepareExperiment())
          {
            return 1;
          }
	
	
          parseParameters(argc,args);
          printf("The random seed is: %f\n", seed);          
          printf("threshold: %f \n", threshold);
          
	time_t now;
          

          struct tm *today;
          char date[230];
          char date2[240];
          
	time(&now);  
          today = localtime(&now);
          double starttime_exp = time(NULL);
                
          strcpy(date,experimentname);
          strcpy(date2,experimentname);
          int experimentNameLen = strlen(experimentname);
	strftime(&date[experimentNameLen], 23, "exp%Y%m%d.%H%M%S.txt", today);
	strftime(&date2[experimentNameLen], 23, "gexp%Y%m%d.%H%M%S.txt", today);	
	printf("%s \n %s \n",date,date2);

////////////////////////////
	//return 0;
	// this needs to be changed for real epuck??
	
	
	if (seed !=0)
	{
            seed = seed * (double)time(NULL);
            srand ( seed );
	} 
	else 
	{
            seed = time(NULL);
            srand(seed);
	}
	
	
	printf("The random seed is: %f\n", seed);
	
	
	
	WbDeviceTag emitter;
	emitter = wb_robot_get_device("emitter");
	wb_emitter_set_channel(emitter, 1);
	
	wb_emitter_set_range(emitter,set_communication_range);
	
	
	

	FILE *fp;
	fp=fopen(date, "a"); 
	fclose(fp);
	
	FILE *genomes;
	genomes=fopen(date2, "a");
	fclose(genomes);
	
	/* main loop */
	int evals = 0; // number of evaluations
	int generation=0;
	int lifetime = 0;
	
	while(wb_robot_step(TIME_STEP) != -1 && evals < total_evals) {
                  
                  generation +=1;
                  
                  if(evolution ==1)
                  { 
                    // only one generation
                    lifetime = max_robot_lifetime ;
                  }
                  else
                  {
                    // Lifetime depends on the experiment to be run/ran?
                    /*if((double)rand()/(double)RAND_MAX <0.5 && generation==1)
                    {
                      lifetime = total_evals;
                    }
                    else
                    {
                      lifetime = total_evals/2;
                    }*/
                    
                    lifetime = total_evals;
                  }
                  
                  printf("generation: %d \n new lifetime: %d\n", generation,lifetime);
                  
                  // pick new genome
                  if(collected_genomes_total==0)
                  { 
                    champion.genome = 0;
                    // no genomes collected and therefore random
                    printf("no genomes collected \n");
                    int i=0;
                    for(i=0 ; i < NB_DIST_SENS ;i++)
                    {
                      if((double)rand()/(double)RAND_MAX > disable_sensor)
                      { 
                        // We do bitwise or to show that genome is a flag type.
                        champion.genome = champion.genome | (int)pow(2,i);
                      }
                    }  
                                                
                  }
                  else
                  {
                    //pick random by tournament
                    printf("Going to pick a genome by tournament\n");
                    printf("Old Genome %s: %x \n", wb_robot_get_name(), champion.genome);
                    RobotGenotypeDataMessage tournamentChampion;
                    int i;
                    int largestFitness = -1;
                    for(i = 0; i < genome_tournament_size; i++)
                    {
                      int tournamentChallenger = rand()%collected_genomes_total;
                      if(collected_genotypes[tournamentChallenger].Fitness > largestFitness || i == 0)
                      {
                        tournamentChampion = collected_genotypes[tournamentChallenger];
                        largestFitness =collected_genotypes[tournamentChallenger].Fitness;
                      }
                    }
                    
                    //compare with own controller
                    printf("Old Genome %s and champion %x %x \n", wb_robot_get_name(), champion.genome, tournamentChampion.Genotype);
                    //printf("compare fitness tournament %f and champion %f \n", tournamentChampion.Fitness, champion.fitness);
                    
       
                    for(i = 0; i < NB_DIST_SENS; i++)
                    {
                      if((double)rand()/(double)RAND_MAX < 0.5)
                      {
     
                      
                        int currentBit = (int)pow(2,i);
                        int curBitTrue = ((champion.genome & currentBit) == (tournamentChampion.Genotype & currentBit));
                        if(curBitTrue == 0)
                        {
                        //printf("other is not same\n");
                          champion.genome = champion.genome ^ (int)pow(2,i);
                        }
                      } 
                    
                      if((double)rand()/(double)RAND_MAX < mutate_sensor)
                      {
                      //printf("mutate sensor %d \n",i);
                        champion.genome = champion.genome ^ (int)pow(2,i);
                      }
                    }
                  }                  
                   
                  printf("new genome \n");
                  int i;
             
                  //printf("genome robot %s: %x\n", wb_robot_get_name(),champion.genome);
                  
                  //store genomes
                  
                  int genome[NB_DIST_SENS];
                  for(i = 0; i < NB_DIST_SENS; i++)
                    {
                      genome[i] = ((champion.genome & (int)pow(2,i) )== (int)pow(2,i));
                    }
                  
                  genomes=fopen(date2, "a");
	        fprintf(genomes,"%d \t %s \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \n", generation, wb_robot_get_name(),genome[0], genome[1], genome[2], genome[3], genome[4], genome[5], genome[6], genome[7]);
	        fclose(genomes);
                  
                  
                  // empty information
                  collected_genomes_total = 0;
                  collected_sensor_weights_total = 0;
                  
                  //create new weights
                  double *tmp = CreateRandomGenome(); //weights can be empty or a number, empty means unabled sensor
          
                  int j=0;
                  for (j = 0; j < NB_DIST_SENS; j++)
                  {
                    if((champion.genome & (int)pow(2,j)) != (int)pow(2,j))
                    {
                      // set values very high to detect later on (prever to have NULL)
                      champion.weights[j] =100;
                      champion.weights[j+(NMBRWEIGHTS/2)] =100;              
                    } 
                    else
                    {
                      champion.weights[j] = tmp[j];
                      champion.weights[j+(NMBRWEIGHTS/2)] = tmp[j+(NMBRWEIGHTS/2)];
                    }
                  }
                   
                   // bias node
                   champion.weights[(NMBRWEIGHTS/2)-1] = tmp[(NMBRWEIGHTS/2)-1];
	         champion.weights[(NMBRWEIGHTS)-1] = tmp[(NMBRWEIGHTS)-1];
          
                  champion.sigma = sigmainitial;      
                  champion.fitness = RunAndEvaluate(&champion);
                  
                  //int restart = 0;
                  int l =0;
                                    
                  for(l=0 ; l < lifetime; l++)
                  {
                    if(l==0 && evolution==1)
                    {
                      //SendGenome(champion,emitter);
                    }
                    evals +=1;           
                    if( (double)rand()/(double)RAND_MAX <= 0.2)
                      { 
                        //printf("Reevaluate\n");
                        //reevalaute 
                        double re_fitness = RunAndEvaluate(&champion);
                        //printf("reevaluate from %f to %f \n", champion.fitness, re_fitness);
                        champion.fitness = champion.fitness *re_weight + re_fitness*(1-re_weight);    
                      } 
                      else
                      {
                        if(collected_sensor_weights_total >0 && sociallearning==1 && (double)rand()/(double)RAND_MAX <= 0.3)
                          { 
                       //printf("Social learning\n");
                          candidate challenger = champion;
                          int i; 
                          for(i = 0; i < NMBRWEIGHTS; i++)
                          {
                            int cswt = collected_sensor_weights_total-1;
                            
                            if(challenger.weights[i] != 100 && collected_phenotypes[cswt].Phenotype[i] != 100)
                            {

                              challenger.weights[i] = collected_phenotypes[cswt].Phenotype[i];
                            }
                          }
                          
    		       collected_sensor_weights_total = collected_sensor_weights_total -1;                         
    		       challenger.fitness = RunAndEvaluate(&challenger);
    		 
    		       if (challenger.fitness > champion.fitness) 
    		       { 
    		         // Replace champion	
		        // printf("old and new champion fitness: %f \t %f  \n",champion.fitness, challenger.fitness);
		         champion = challenger;
		         champion.sigma = sigma_min;
		       }
		       else
		       {
		         champion.sigma = sigma_increase*champion.sigma;
		         if(champion.sigma > sigma_max)
		         {
			 champion.sigma = sigma_max;
		         }
		       }
    		     }
	               else
	               {
	                 if(lifetimelearning==1)
	                 {
	                   //printf("try new one from individual \n");
	                   candidate challenger = champion;
	                   Mutate(&challenger);
	                   challenger.fitness = RunAndEvaluate(&challenger);
	            
	                   if (challenger.fitness > champion.fitness) 
	                   { 
	                     // Replace champion
                               //printf("old and new champion fitness: %f \t %f  \n",champion.fitness, challenger.fitness);
                               champion = challenger;
		            champion.sigma = sigma_min;
			}
			else
			{
			  champion.sigma = sigma_increase*champion.sigma;
			  if(champion.sigma > sigma_max)
			  {
			    champion.sigma = sigma_max;
			  }
			}
	                 }
	                 else
	                 {
	                   // re-evaluate or dummy run
	                  // printf("reevaluate second try\n");
	                   double re_fitness = RunAndEvaluate(&champion);
                             champion.fitness = champion.fitness *re_weight + re_fitness*(1-re_weight);
                           }
                         }
                       }

                  // sending and receiving
                  
                  if(sociallearning ==1 && (champion.fitness/max_fitness) > threshold)
                  {
                    //printf("Sending phenotype\n");
                    RobotPhenotypeDataMessage message;
                    memcpy(message.Phenotype,champion.weights,sizeof(double)*NMBRWEIGHTS);
                    //printf("Finished sending phenotype: ");
                    
                    int i;                    
                    for (i = 0; i < NMBRWEIGHTS; i++)
                    {
                    //  printf("%f, ",message.Phenotype[i]);
                    }
                    //printf("\n");

                    message.Fitness = champion.fitness;
                    
                    for(i = 0; i < no_bots; i++)
                    {
                      // spread based on fitness
                      strcpy(message.Destination,bots[i]);
                      wb_emitter_send(emitter, &message, sizeof(message));
                    }                    
    	        }
    	        
    	        //send genome once in 10 evals
    	        if(l == (lifetime-2))
    	        {
    	          //printf("send genomes %d \n", evals);
    	          printf("genome send %x \n", champion.genome);
    	          SendGenome(champion,emitter);
    	        }
    	        
                                             
                  //get collected info after every run
	        while(wb_receiver_get_queue_length(receiver)>0)
	        {	
	          //printf("Receiving messages\n");
    	          const void *message = wb_receiver_get_data(receiver);
    		int messageSize = wb_receiver_get_data_size(receiver);
    		
    		if(messageSize == sizeof(RobotPhenotypeDataMessage))
    		{
    		  //printf("Robot phenotype datamessage received: ");
    		  if(collected_sensor_weights_total <collected_sensor_weights_max)
    		  {
    		    memcpy(&(collected_phenotypes[collected_sensor_weights_total]),message,messageSize);
    		    
            	     int i;
                        for (i = 0; i < NMBRWEIGHTS; i++)
                        {
                         // printf("%f, ",collected_phenotypes[collected_sensor_weights_total].Phenotype[i]);
                        }
                        //printf("\n");
   
    		    
    		    collected_sensor_weights_total += 1;
    		  }
    		}
    		else if(messageSize == sizeof(RobotGenotypeDataMessage))
    		{
    		 // printf("Robot genotype datamessage received\n");
                     // Check if we already have it
    		  int genotype_i = -1;
    		  int i;
    		  for (i = 0; i < collected_genomes_total; i++)
    		  {
    		  //printf("compare %f with %f \n", (RobotGenotypeDataMessage*)message)->Genotype,collected_genotypes[i].Genotype);
    		    if(((RobotGenotypeDataMessage*)message)->Genotype == collected_genotypes[i].Genotype)
    		    {
    		      genotype_i = i;
    		    }
    		  }
    		  
    		  // If not add it to the collected_genotypes array and send our own.
    		  if(genotype_i == -1)
    		  {
    		    if(collected_genomes_total < collected_genomes_max)
    		    {
    		      memcpy(&(collected_genotypes[collected_genomes_total]), message, messageSize);
    		      collected_genomes_total += 1;
    		    }
    		  
    		    //SendGenome(champion,emitter);
    		  }
    		  else
    		  {
    		    collected_genotypes[genotype_i].Fitness = ((RobotGenotypeDataMessage*)message)->Fitness;
    		  }
    		}
    		else
    		{
    		  printf("We're doomed\n");
    		}
    		
    		wb_receiver_next_packet(receiver);
                 }
                 
                 printf("%s new fitness %f , age %d, collected genomes %d, collected phenotyes %d\n", wb_robot_get_name(), champion.fitness, l ,collected_genomes_total, collected_sensor_weights_total);
                    
                 fp=fopen(date, "a");
	       fprintf(fp,"%d \t %d \t %d \t %f\n",evals, generation, l, champion.fitness); 
	       fclose(fp);
	     }
	   }	
	   double endtime_exp = time(NULL);
	   printf("time %f \n", starttime_exp-endtime_exp);
	
	    // send done to supervisor
	  printf("send ho \n");
            char str[3];
            sprintf(str, "ho");
            wb_emitter_send(emitter, str, strlen(str) + 1);
            wb_robot_step(50);
	   
	   wb_robot_cleanup();
	   

            return 0;
}            


static void SendGenome(candidate chmp, WbDeviceTag emitter)
{
  RobotGenotypeDataMessage msg;
  msg.Fitness = chmp.fitness;
  msg.Genotype = chmp.genome;
  
  int i;
  for(i = 0; i < no_bots; i++)
  {
    strcpy(msg.Destination,bots[i]);
    wb_emitter_send(emitter, &msg, sizeof(msg));
  }
}

static double* CreateRandomGenome() 
{
	//double *weights = malloc(NMBRWEIGHTS*sizeof(double));
	static double weights[NMBRWEIGHTS];
	int i = 0;
	for (i = 0; i < NMBRWEIGHTS; i++) 
	{
	  weights[i] = range_weights*((2*(double) rand()) / (double) RAND_MAX)-range_weights;
	}
	
	return weights;
}

static void Mutate(candidate* solution) 
{
	int i = 0;
	for (i = 0; i < NMBRWEIGHTS; i++)
          {
	  if(solution->weights[i] != 100)
	  {
	    solution->weights[i] = solution->weights[i] + gaussrand() * solution->sigma;
	    if(solution->weights[i] > range_weights)
	    {
                solution->weights[i] = range_weights;
              }
	    if(solution->weights[i] < -range_weights)
	    {
	      solution->weights[i] = -range_weights;
	    }
	  }
	}
}

static double RunAndEvaluate(candidate* evaluee) 
{
	//recover
	int i = 0;
	while (i < tau) 
	{
	  RunAndEvaluateForOneTimeStep(evaluee);
	  i++;
	}
	
	//evaluate
	i = 0;
	double fitness = 0;
	while (i < evaltime) 
	{
	  fitness += RunAndEvaluateForOneTimeStep(evaluee);
	  i++;
	}

	return fitness;
}

static double RunAndEvaluateForOneTimeStep(candidate* evaluee) 
{
	double fitness = 0;
	// read
	wb_differential_wheels_enable_encoders(TIME_STEP);
	wb_differential_wheels_set_encoders(0,0);
	// read sensors
	int i = 0;
	for(i=0;i<NB_DIST_SENS;i++)
	{
	  ps_value[i] = (int)wb_distance_sensor_get_value(ps[i]);
	}
	
	// calculate neural net
	double left=0.0;
	double right=0.0;
	for(i=0;i<NB_DIST_SENS;i++)
	{
	
	//printf("value sensor %d %d \n",i,  ps_value[i]);
	
	  if(evaluee->weights[i] != 100)
	  {
	    //only evaluate when enabled
              left  += mindouble((ps_value[i]-(SENSOR_MAX/2))/(SENSOR_MAX/2), 1.0) * evaluee->weights[i];
              right += mindouble((ps_value[i]-(SENSOR_MAX/2))/(SENSOR_MAX/2), 1.0) * evaluee->weights[i+(NMBRWEIGHTS/2)];
	  }
	}
	  
	left += evaluee->weights[(NMBRWEIGHTS/2)-1];
	right +=  evaluee->weights[(NMBRWEIGHTS)-1];
	left = tanh(left);
	right = tanh(right);
	
	if (left > 1) {left=1; printf("Wut?");}
	if (left <-1) {left=-1; printf("Wut?");}
	if (right > 1) {right=1; printf("Wut?");}
	if (right <-1) {right=-1; printf("Wut?");}
	
	motorspeed[LEFT]=left*MAXSPEED;
	motorspeed[RIGHT]=right*MAXSPEED;
	
	// set motor speed
	wb_differential_wheels_set_speed(motorspeed[LEFT],motorspeed[RIGHT]);
	
	// wait till end of step
	wb_robot_step(TIME_STEP);
  	
  	// determine fitness
  	double sensorpenalty=0;
          for(i=0;i<NB_DIST_SENS;i++)
          {
            if(evaluee->weights[NB_DIST_SENS] !=100)
            {
              if (sensorpenalty < (double)ps_value[i]/(double)SENSOR_MAX)
              {
                sensorpenalty = (double)ps_value[i]/((double)SENSOR_MAX);
                //printf("sensorpenalty %f: \n", sensorpenalty);
              }
	   }
	}
	  
	if (sensorpenalty>1) {sensorpenalty=1;}
	double speedpenalty = 0.0;
	if (motorspeed[LEFT]>motorspeed[RIGHT]) 
	{
  	  speedpenalty = (double)(motorspeed[LEFT]-motorspeed[RIGHT])/(double)(MAXSPEED);
	} 
	else 
	{
  	  speedpenalty = (double)(motorspeed[RIGHT]-motorspeed[LEFT])/(double)(MAXSPEED);
	}
	
	if (speedpenalty>1) {speedpenalty=1;}
	  
	fitness = (double)(wb_differential_wheels_get_left_encoder()+wb_differential_wheels_get_right_encoder())
	           *((double)1-speedpenalty)*((double)1-sensorpenalty);

	return fitness;
}

/* uniform distribution, (0..1] */
double drand()
{
  return (rand()+1.0)/(RAND_MAX+1.0);
}

/* normal distribution, centered on 0, std dev 1 */
double random_normal()  
{
  return (-2*log(drand())) ;
}

// Used because else webots gives a strange segfault during cross compilation
double sqrt_rand_normal()
{
  return sqrt(random_normal());
}

static double gaussrand() 
{	
  return sqrt_rand_normal()* cos(2*M_PI*drand()); 	
}

static double mindouble(double a, double b) 
{
  if (a<b) 
  {
    return a;
  }
  else 
  {
    return b;
  }
}

bool parseParameters(int argc, char **argv){
	const struct option longopts[] =
    {
	    {"threshold",		required_argument,	0, 't'},
	    {"seed",			required_argument,	0, 's'},
	    {0,0,0,0},
    };
    
	int option_index = 0;
	int c;
	while((c = getopt_long(argc,argv,"ts",longopts,&option_index)) != -1) {
		switch(c){

            case 't':
                if(optarg){
                    threshold = atof(optarg);
                }else{
                    printf("Error parsing option seed: needs argument");
                    return false;
                }
                break;
            case 's':
                if(optarg){
                    seed = atof(optarg);
                    srand(seed);
                }else{
                    printf("Error parsing option seed: needs argument");
                    return false;
                }
                break;
		}
	}
	
	return true;
}
