// os.c
// Runs on LM4F120/TM4C123/MSP432
// Uses SysTick and TA1_0_IRQHandler
// Daniel Valvano
// March 24, 2016

#include <stdint.h>
#include "os.h"
#include "..\inc\CortexM.h"
#include "..\inc\Clock.h"
#include "msp.h"
// function definitions in osasm.s
void StartOS(void);

#define NUMTHREADS  6        // maximum number of threads
#define NUMPERIODIC 2        // maximum number of periodic threads
#define STACKSIZE   100      // number of 32-bit words in stack per thread
struct tcb{
  int32_t *sp;       // pointer to stack (valid for threads not running
  struct tcb *next;  // linked-list pointer
  int32_t *BlockPt;  // nonzero if blocked on this semaphore
  uint32_t Sleep;    // nonzero if this thread is sleeping
};
typedef struct tcb tcbType;
tcbType tcbs[NUMTHREADS];
tcbType *RunPt;
int32_t Stacks[NUMTHREADS][STACKSIZE];
int EventThreadIndex;                           // index into EventThread arrays
uint32_t EventThreadPeriodsReload[NUMPERIODIC]; // periodic task reload values
uint32_t EventThreadPeriods[NUMPERIODIC];       // periodic task time values
void (*PeriodicEvents[NUMPERIODIC])(void);      // periodic user functions
void DummyEventThread(void){};
void static runperiodicevents(void);

// ***************** PeriodicTask_InitB ****************
// Activate 16-bit Timer A1 interrupts to run user task periodically
// assumes SMCLK is 12MHz, using divide by 24, 500kHz/65536=7.6 Hz
// Input:  task is a pointer to a user function
//         freq is number of interrupts per second
//           8 Hz to 10 kHz
//         priority is a number 0 to 6
// Outputs: none
// comment: it is accurate if 500000/freq is an integer
void (*PeriodicTaskB)(void);   // user function
void PeriodicTask_InitB(void(*task)(void), uint32_t freq, uint8_t priority){
  if((freq < 8) || (freq > 10000)){
    return;                        // invalid input
  }
  if(priority > 6){
    priority = 6;
  }
  PeriodicTaskB = task;  // user function
  TIMER_A1->CTL &= ~0x0030;     // halt Timer A1
  // bits15-10=XXXXXX, reserved
  // bits9-8=10,       clock source to SMCLK
  // bits7-6=10,       input clock divider /4
  // bits5-4=00,       stop mode
  // bit3=X,           reserved
  // bit2=0,           set this bit to clear
  // bit1=0,           no interrupt on timer
  // bit0=0,           clear interrupt pending
  TIMER_A1->CTL = 0x0280;
  TIMER_A1->EX0 = 0x0005;    // configure for input clock divider /6
  // bits15-14=00,     no capture mode
  // bits13-12=XX,     capture/compare input select
  // bit11=X,          synchronize capture source
  // bit10=X,          synchronized capture/compare input
  // bit9=X,           reserved
  // bit8=0,           compare mode
  // bits7-5=XXX,      output mode
  // bit4=1,           enable capture/compare interrupt on CCIFG
  // bit3=X,           read capture/compare input from here
  // bit2=0,           output this value in output mode 0
  // bit1=X,           capture overflow status
  // bit0=0,           clear capture/compare interrupt pending
  TIMER_A1->CCTL[0] = 0x0010;
  TIMER_A1->CCR[0] = (500000/freq) - 1;  // compare match value
// interrupts enabled in the main program after all devices initialized
  NVIC->IP[2] = (NVIC->IP[2]&0xFF00FFFF)|(priority<<21);
  NVIC->ISER[0] = 0x00000400; // enable interrupt 10 in NVIC
  TIMER_A1->CTL |= 0x0014;        // reset and start Timer A1 in up mode
}

// ------------PeriodicTask_StopB------------
// Deactivate the interrupt running a user task periodically.
// Input: none
// Output: none
void PeriodicTask_StopB(void){
  TIMER_A1->CTL &= ~0x0030;           // halt Timer A1
  NVIC->ISER[0] = 0x00000400;     // disable interrupt 10 in NVIC
}

void TA1_0_IRQHandler(void){
  TIMER_A1->CCTL[0] &= ~0x0001;          // acknowledge capture/compare interrupt TA1_0
  (*PeriodicTaskB)();           // execute user task
}

// ******** OS_Init ************
// Initialize operating system, disable interrupts
// Initialize OS controlled I/O: periodic interrupt, bus clock as fast as possible
// Initialize OS global variables
// Inputs:  none
// Outputs: none
void OS_Init(void){int i;
  DisableInterrupts();
  Clock_Init48MHz();  // makes SMCLK=12 MHz
  EventThreadIndex = 0;
  for(i=0; i<NUMPERIODIC; i=i+1){
    EventThreadPeriodsReload[i] = 0; // reload value of 0 means unused
    EventThreadPeriods[i] = 0;
    PeriodicEvents[i] = DummyEventThread;
  }
  PeriodicTask_InitB(&runperiodicevents, 1000, 0);
}

void SetInitialStack(int i){
  tcbs[i].sp = &Stacks[i][STACKSIZE-16]; // thread stack pointer
  Stacks[i][STACKSIZE-1] = 0x01000000;   // thumb bit
  Stacks[i][STACKSIZE-3] = 0x14141414;   // R14
  Stacks[i][STACKSIZE-4] = 0x12121212;   // R12
  Stacks[i][STACKSIZE-5] = 0x03030303;   // R3
  Stacks[i][STACKSIZE-6] = 0x02020202;   // R2
  Stacks[i][STACKSIZE-7] = 0x01010101;   // R1
  Stacks[i][STACKSIZE-8] = 0x00000000;   // R0
  Stacks[i][STACKSIZE-9] = 0x11111111;   // R11
  Stacks[i][STACKSIZE-10] = 0x10101010;  // R10
  Stacks[i][STACKSIZE-11] = 0x09090909;  // R9
  Stacks[i][STACKSIZE-12] = 0x08080808;  // R8
  Stacks[i][STACKSIZE-13] = 0x07070707;  // R7
  Stacks[i][STACKSIZE-14] = 0x06060606;  // R6
  Stacks[i][STACKSIZE-15] = 0x05050505;  // R5
  Stacks[i][STACKSIZE-16] = 0x04040404;  // R4
}

//******** OS_AddThreads ***************
// Add six main threads to the scheduler
// Inputs: function pointers to six void/void main threads
// Outputs: 1 if successful, 0 if this thread can not be added
// This function will only be called once, after OS_Init and before OS_Launch
int OS_AddThreads(void(*thread0)(void),
                  void(*thread1)(void),
                  void(*thread2)(void),
                  void(*thread3)(void),
                  void(*thread4)(void),
                  void(*thread5)(void)){
  int32_t status,i;
  status = StartCritical();
  tcbs[0].next = &tcbs[1]; // 0 points to 1
  tcbs[1].next = &tcbs[2]; // 1 points to 2
  tcbs[2].next = &tcbs[3]; // 2 points to 3
  tcbs[3].next = &tcbs[4]; // 3 points to 4
  tcbs[4].next = &tcbs[5]; // 4 points to 5
  tcbs[5].next = &tcbs[0]; // 5 points to 0
  for(i=0;i<NUMTHREADS;i++){
    tcbs[i].BlockPt = 0;     // initially not blocked
    tcbs[i].Sleep = 0;       // initially not sleeping
    SetInitialStack(i);
  }
  Stacks[0][STACKSIZE-2] = (int32_t)(thread0); // PC
  Stacks[1][STACKSIZE-2] = (int32_t)(thread1); // PC
  Stacks[2][STACKSIZE-2] = (int32_t)(thread2); // PC
  Stacks[3][STACKSIZE-2] = (int32_t)(thread3); // PC
  Stacks[4][STACKSIZE-2] = (int32_t)(thread4); // PC
  Stacks[5][STACKSIZE-2] = (int32_t)(thread5); // PC
  RunPt = &tcbs[0];       // thread 0 will run first
  EndCritical(status);
  return 1;               // successful
}

//******** OS_AddPeriodicEventThread ***************
// Add one background periodic event thread
// Typically this function receives the highest priority
// Inputs: pointer to a void/void event thread function
//         period given in units of OS_Launch (msec)
// Outputs: 1 if successful, 0 if this thread cannot be added
// It is assumed that the event threads will run to completion and return
// It is assumed the time to run these event threads is short compared to 1 msec
// These threads cannot spin, block, loop, sleep, or kill
// These threads can call OS_Signal
// In Lab 3 this will be called exactly twice
int OS_AddPeriodicEventThread(void(*thread)(void), uint32_t period){
  if(EventThreadIndex < NUMPERIODIC){
    EventThreadPeriodsReload[EventThreadIndex] = period;
    EventThreadPeriods[EventThreadIndex] = EventThreadIndex + 1; // staggered
    PeriodicEvents[EventThreadIndex] = thread;
    EventThreadIndex = EventThreadIndex + 1;
    return 1;
  } else{
    return 0;
  }
}

void static runperiodicevents(void){int i;
  tcbType *searchPt;
  for(i=0; i<NUMPERIODIC; i=i+1){
    if(EventThreadPeriodsReload[i] > 0){
      EventThreadPeriods[i] = EventThreadPeriods[i] - 1;
      if(EventThreadPeriods[i] == 0){
        EventThreadPeriods[i] = EventThreadPeriodsReload[i];
        (*PeriodicEvents[i])(); // runs every EventThreadPeriodsReload[i] ticks
      }
    }
  }
  searchPt = RunPt;
  for(i=0; i<NUMTHREADS; i=i+1){
    if((searchPt->Sleep) > 0){
      searchPt->Sleep = searchPt->Sleep - 1;
    }
    searchPt = searchPt->next;
  }
}

//******** OS_Launch ***************
// Start the scheduler, enable interrupts
// Inputs: number of clock cycles for each time slice
// Outputs: none (does not return)
// Errors: theTimeSlice must be less than 16,777,216
void OS_Launch(uint32_t theTimeSlice){
  SysTick->CTRL = 0;                  // disable SysTick during setup
  SysTick->VAL = 0;               // any write to current clears it
  SCB->SHP[11] = 7<<5; // priority 7
  SysTick->LOAD = theTimeSlice - 1; // reload value
  SysTick->CTRL = 0x00000007;         // enable, core clock and interrupt arm
  StartOS();                   // start on the first task
}
// runs every ms
void Scheduler(void){ // every time slice
  RunPt = RunPt->next;    // Round Robin
  while(((RunPt->BlockPt) != 0) || ((RunPt->Sleep) > 0)){
    RunPt = RunPt->next;
  }
}

//******** OS_Suspend ***************
// Called by main thread to cooperatively suspend operation
// Inputs: none
// Outputs: none
// Will be run again depending on sleep/block status
void OS_Suspend(void){
  SysTick->VAL = 0;        // any write to current clears it
  SCB->ICSR = 0x04000000; // trigger SysTick
// next thread gets a full time slice
}

// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(uint32_t sleepTime){
  RunPt->Sleep = sleepTime; // match to BSP_periodic ISR
  OS_Suspend();     // stops running
}

// ******** OS_InitSemaphore ************
// Initialize counting semaphore
// Inputs:  pointer to a semaphore
//          initial value of semaphore
// Outputs: none
void OS_InitSemaphore(int32_t *semaPt, int32_t value){
  *semaPt = value;
}

// ******** OS_Wait ************
// Decrement semaphore and block if less than zero
// Lab2 spinlock (does not suspend while spinning)
// Lab3 block if less than zero
// Inputs:  pointer to a counting semaphore
// Outputs: none
void OS_Wait(int32_t *semaPt){
  int32_t status;
  status = StartCritical();
  *semaPt = *semaPt - 1;
  if(*semaPt < 0){
    RunPt->BlockPt = semaPt; // block
    EndCritical(status); // end critical section
    OS_Suspend();        // this thread stops running
    return;
  }
  EndCritical(status);   // end critical section
}

// ******** OS_Signal ************
// Increment semaphore
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate
// Inputs:  pointer to a counting semaphore
// Outputs: none
void OS_Signal(int32_t *semaPt){
  tcbType *searchPt;
  int32_t status;
  status = StartCritical();
  *semaPt = *semaPt + 1;
  if((*semaPt) < 1){
    searchPt = RunPt->next;
    while(searchPt->BlockPt != semaPt){
      searchPt = searchPt->next; // find one blocked on this semaphore
    }
    searchPt->BlockPt = 0; // wake up first one it finds
  }
  EndCritical(status);
}

#define FSIZE 10    // can be any size
uint32_t PutI;      // index of where to put next
uint32_t GetI;      // index of where to get next
uint32_t Fifo[FSIZE];
int32_t CurrentSize;// 0 means FIFO empty, FSIZE means full
uint32_t LostData;  // number of lost pieces of data

// ******** OS_FIFO_Init ************
// Initialize FIFO.  The "put" and "get" indices initially
// are equal, which means that the FIFO is empty.  Also
// initialize semaphores to track properties of the FIFO
// such as size and busy status for Put and Get operations,
// which is important if there are multiple data producers
// or multiple data consumers.
// Inputs:  none
// Outputs: none
void OS_FIFO_Init(void){
  PutI = GetI = 0;   // Empty
  OS_InitSemaphore(&CurrentSize, 0);
  LostData = 0;
}

// ******** OS_FIFO_Put ************
// Put an entry in the FIFO.  Consider using a unique
// semaphore to wait on busy status if more than one thread
// is putting data into the FIFO and there is a chance that
// this function may interrupt itself.
// Inputs:  data to be stored
// Outputs: 0 if successful, -1 if the FIFO is full
int OS_FIFO_Put(uint32_t data){
  if(CurrentSize == FSIZE){
    LostData++;
    return -1;  // full
  } else{
    Fifo[PutI] = data;       // Put
    PutI = (PutI+1)%FSIZE;
    OS_Signal(&CurrentSize);
    return 0;   // success
  }
}

// ******** OS_FIFO_Get ************
// Get an entry from the FIFO.  Consider using a unique
// semaphore to wait on busy status if more than one thread
// is getting data from the FIFO and there is a chance that
// this function may interrupt itself.
// Inputs:  none
// Outputs: data retrieved
uint32_t OS_FIFO_Get(void){uint32_t data;
  OS_Wait(&CurrentSize);   // block if empty
  data = Fifo[GetI];       // get
  GetI = (GetI+1)%FSIZE;   // place to get next
  return data;
}

