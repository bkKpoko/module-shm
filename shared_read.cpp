#include <cstddef>
#include <cstdio>
#include <iostream>
#include <iterator>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <vector>
#include <chrono>
#include <semaphore.h>
#include <stdbool.h>

typedef struct {
  sem_t sem_produce;     
  sem_t sem_consume;     
  size_t data_size;
  double data[];
} SharedBuffer;



void consumer(SharedBuffer* sbuf) {
  printf("Wait sem_consume \n");
  sem_wait(&sbuf->sem_consume);
  printf("Done\n");
  for (int i = 0; i < sbuf->data_size; ++i) {
    printf("data[%i] = %g\n", i, sbuf->data[i]);
  }
  sem_post(&sbuf->sem_produce);
}

int main() {
  printf("O_RDWR = %i\n", O_RDWR);
  printf("O_CREAT = %i\n", O_CREAT);
  printf("O_RDWR | O_CREAT = %i\n", O_RDWR | O_CREAT);
  bool x = true;
  bool y = false;
  printf("O_RDWR | true * O_CREAT = %i\n", O_RDWR | (x * O_CREAT));
  printf("O_RDWR | false * O_CREAT = %i\n", O_RDWR | (y * O_CREAT));
    // Открываем существующую shared memory
  int fd = shm_open("out", O_RDWR, 0666);
  size_t shm_size = sizeof(SharedBuffer) + sizeof(double) * 12; 
  SharedBuffer* sbuf = (SharedBuffer*)mmap(NULL, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  printf("buffer done\n");

  while(1){
      consumer(sbuf);
  }

  munmap(sbuf, sizeof(SharedBuffer));
  return 0;
}
