#ifndef IPC_H
#define IPC_H

#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/msg.h>
#include <sys/sem.h>
#include <sys/wait.h>
#include <unistd.h>
#include <pthread.h>
#include "genetic_algorithm.h"

// IPC keys (using IPC_PRIVATE so no need for keys)
#define MSG_TYPE_WORK 1
#define MSG_TYPE_RESULT 2
#define MSG_TYPE_TERMINATE 3

// Message structures
typedef struct {
    long mtype;
    int worker_id;
    int chrom_index;
    Chromosome chromosome;
    int generation;
} WorkMessage;

typedef struct {
    long mtype;
    int worker_id;
    int chrom_index;
    double fitness;
    int survivors;
    double coverage;
    double risk;
    int generation;
} ResultMessage;

// Worker status
typedef struct {
    int active;
    int completed_tasks;
    double total_work_time;
    pid_t pid;
} WorkerStatus;

// Shared data structure
typedef struct {
    Population pop;
    Grid3D* grid;
    Config cfg;
    int current_generation;
    int termination_requested;
    WorkerStatus* workers;
    int num_workers;
    pthread_mutex_t mutex;
} SharedData;

// Custom semaphore functions
void my_sem_wait(int sem_id, int sem_num);
void my_sem_signal(int sem_id, int sem_num);

// IPC Functions
int create_shared_memory(size_t size);
SharedData* attach_shared_memory(int shm_id);
void detach_shared_memory(SharedData* data);

int create_message_queue();
void remove_message_queue(int msg_id);

int create_semaphore(int num_workers);
void remove_semaphore(int sem_id);

// Worker Pool Management
void create_worker_pool(int num_workers, pid_t* worker_pids, 
                       int shm_id, int msg_id, int sem_id);
void terminate_worker_pool(int num_workers, pid_t* worker_pids, 
                          int msg_id, int sem_id);
void print_worker_status(SharedData* shared_data);

// Worker Process
void worker_process(int worker_id, int shm_id, int msg_id, int sem_id);

#endif
