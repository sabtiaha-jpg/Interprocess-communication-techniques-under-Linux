#include "ipc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/msg.h>
#include <sys/sem.h>
#include <sys/wait.h>
#include <unistd.h>
#include <pthread.h>

// Custom semaphore functions
void my_sem_wait(int sem_id, int sem_num) {
    struct sembuf op;
    op.sem_num = sem_num;
    op.sem_op = -1;
    op.sem_flg = 0;
    semop(sem_id, &op, 1);
}

void my_sem_signal(int sem_id, int sem_num) {
    struct sembuf op;
    op.sem_num = sem_num;
    op.sem_op = 1;
    op.sem_flg = 0;
    semop(sem_id, &op, 1);
}

// Create shared memory segment
int create_shared_memory(size_t size) {
    int shm_id = shmget(IPC_PRIVATE, size, IPC_CREAT | 0666);
    if (shm_id == -1) {
        perror("shmget failed");
        exit(1);
    }
    return shm_id;
}

// Attach to shared memory
SharedData* attach_shared_memory(int shm_id) {
    SharedData* data = (SharedData*)shmat(shm_id, NULL, 0);
    if (data == (SharedData*)-1) {
        perror("shmat failed");
        exit(1);
    }
    return data;
}

// Detach from shared memory
void detach_shared_memory(SharedData* data) {
    if (shmdt(data) == -1) {
        perror("shmdt failed");
    }
}

// Create message queue
int create_message_queue() {
    int msg_id = msgget(IPC_PRIVATE, IPC_CREAT | 0666);
    if (msg_id == -1) {
        perror("msgget failed");
        exit(1);
    }
    return msg_id;
}

// Remove message queue
void remove_message_queue(int msg_id) {
    msgctl(msg_id, IPC_RMID, NULL);
}

// Create semaphore set
int create_semaphore(int num_workers) {
    int num_sems = 2;
    int sem_id = semget(IPC_PRIVATE, num_sems, IPC_CREAT | 0666);
    if (sem_id == -1) {
        perror("semget failed");
        exit(1);
    }
    
    // Initialize semaphores
    union semun {
        int val;
        struct semid_ds *buf;
        unsigned short *array;
    } arg;
    
    arg.val = 1;  // mutex = 1
    semctl(sem_id, 0, SETVAL, arg);
    
    arg.val = 0;  // work completed = 0
    semctl(sem_id, 1, SETVAL, arg);
    
    return sem_id;
}

// Remove semaphore set
void remove_semaphore(int sem_id) {
    semctl(sem_id, 0, IPC_RMID);
}

// Worker process function
void worker_process(int worker_id, int shm_id, int msg_id, int sem_id) {
    SharedData* shared_data = attach_shared_memory(shm_id);
    struct timespec start_time, end_time;
    
    printf("Worker %d started (PID: %d)\n", worker_id, getpid());
    
    // Main worker loop
    while (1) {
        WorkMessage work_msg;
        
        // Wait for work assignment
        if (msgrcv(msg_id, &work_msg, sizeof(WorkMessage) - sizeof(long), 
                  worker_id + 1, 0) == -1) {
            if (errno != EINTR) {
                perror("Worker: msgrcv failed");
            }
            continue;
        }
        
        // Check for termination
        if (work_msg.chrom_index == -1) {
            printf("Worker %d: Termination signal received\n", worker_id);
            break;
        }
        
        // Mark as active
        my_sem_wait(sem_id, 0);  // Lock mutex
        shared_data->workers[worker_id].active = 1;
        my_sem_signal(sem_id, 0);  // Unlock mutex
        
        // Start timing
        clock_gettime(CLOCK_MONOTONIC, &start_time);
        
        // Evaluate chromosome
        evaluate_chromosome(&work_msg.chromosome, shared_data->grid,
                           shared_data->cfg.w1, shared_data->cfg.w2,
                           shared_data->cfg.w3, shared_data->cfg.w4);
        
        // End timing
        clock_gettime(CLOCK_MONOTONIC, &end_time);
        double work_time = (end_time.tv_sec - start_time.tv_sec) + 
                          (end_time.tv_nsec - start_time.tv_nsec) / 1e9;
        
        // Send result back
        ResultMessage result_msg;
        result_msg.mtype = MSG_TYPE_RESULT;
        result_msg.worker_id = worker_id;
        result_msg.chrom_index = work_msg.chrom_index;
        result_msg.fitness = work_msg.chromosome.fitness;
        result_msg.survivors = work_msg.chromosome.survivors_rescued;
        result_msg.coverage = work_msg.chromosome.coverage;
        result_msg.risk = work_msg.chromosome.total_risk;
        result_msg.generation = work_msg.generation;
        
        msgsnd(msg_id, &result_msg, sizeof(ResultMessage) - sizeof(long), 0);
        
        // Update worker status
        my_sem_wait(sem_id, 0);  // Lock mutex
        shared_data->workers[worker_id].active = 0;
        shared_data->workers[worker_id].completed_tasks++;
        shared_data->workers[worker_id].total_work_time += work_time;
        my_sem_signal(sem_id, 0);  // Unlock mutex
        
        // Signal work completion
        my_sem_signal(sem_id, 1);
    }
    
    detach_shared_memory(shared_data);
    printf("Worker %d exiting\n", worker_id);
    exit(0);
}

// Create worker pool
void create_worker_pool(int num_workers, pid_t* worker_pids, 
                       int shm_id, int msg_id, int sem_id) {
    printf("Creating worker pool with %d workers...\n", num_workers);
    
    for (int i = 0; i < num_workers; i++) {
        pid_t pid = fork();
        if (pid == 0) {
            worker_process(i, shm_id, msg_id, sem_id);
            exit(0);
        } else if (pid > 0) {
            worker_pids[i] = pid;
            printf("  Worker %d: PID %d\n", i, pid);
        } else {
            perror("fork failed");
            exit(1);
        }
    }
    
    // Brief pause for workers to initialize
    usleep(100000);
}

// Terminate worker pool
void terminate_worker_pool(int num_workers, pid_t* worker_pids, 
                          int msg_id, int sem_id) {
    printf("\nTerminating worker pool...\n");
    
    for (int i = 0; i < num_workers; i++) {
        WorkMessage term_msg;
        term_msg.mtype = i + 1;
        term_msg.chrom_index = -1;
        term_msg.generation = -1;
        
        msgsnd(msg_id, &term_msg, sizeof(WorkMessage) - sizeof(long), 0);
    }
    
    // Wait for all workers to terminate
    for (int i = 0; i < num_workers; i++) {
        if (worker_pids[i] > 0) {
            waitpid(worker_pids[i], NULL, 0);
            printf("  Worker %d terminated\n", i);
        }
    }
}

// Print worker status
void print_worker_status(SharedData* shared_data) {
    printf("\n=== Worker Pool Status ===\n");
    for (int i = 0; i < shared_data->num_workers; i++) {
        printf("Worker %d: %s | Tasks: %d | Time: %.2fs\n",
               i,
               shared_data->workers[i].active ? "ACTIVE" : "IDLE",
               shared_data->workers[i].completed_tasks,
               shared_data->workers[i].total_work_time);
    }
}
