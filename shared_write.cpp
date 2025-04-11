#include <cstddef>
#include <cstdio>
#include <iterator>
#include <semaphore.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <iostream>
#include <vector>

typedef struct {
  sem_t sem_produce;
  sem_t sem_consume;    
  size_t data_size;
  double data[];
  

} SharedBuffer;


void init_shared_buffer(SharedBuffer* sbuf, int n) {
  sbuf->data_size = n;
  sem_init(&sbuf->sem_consume, 1, 0);  
  sem_init(&sbuf->sem_produce, 1, 1);  
}

void producer(SharedBuffer* sbuf, double* new_data) {
  printf("Wait sem_produce \n");
  sem_wait(&sbuf->sem_produce);
  printf("Done\n");
  for (int i = 0; i < 12; ++i)
  {
    sbuf->data[i] = new_data[i];
  }
  sem_post(&sbuf->sem_consume);
}

int main() {
  // Создаем shared memory
  int fd = shm_open("from_mbdyn", O_CREAT | O_RDWR, 0666);
  size_t shm_size = sizeof(SharedBuffer) + sizeof(double) * 12; 
  ftruncate(fd, shm_size);
  SharedBuffer* sbuf = (SharedBuffer*)mmap(NULL, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  // sbuf->data = (double*)(((void*)sbuf) + sizeof(bool) + sizeof(sem_t) + sizeof(size_t));
  printf("sbuf = %p\n", sbuf);
  printf("sbuf->data = %p\n", sbuf->data);

  init_shared_buffer(sbuf, 12);
  printf("size = sbuf->data_size\n");
  printf("buffer inited\n");

  double zeros[] = {0.0, 0.0, 0.0};
  producer(sbuf, zeros);
  int i = 0;
  while (true) {
    printf("before s\n");
    double s[] = {
      i * 3.14/365 * 1, 
      i * 3.14/365 * 2, 
      i * 3.14/365 * 3,
      i * 3.14/365 * 4,
      i * 3.14/365 * 5,
      i * 3.14/365 * 6,
      i * 3.14/365 * 7,
      i * 3.14/365 * 8,
      i * 3.14/365 * 9,
      i * 3.14/365 * 10,
      i * 3.14/365 * 11,
      i * 3.14/365 * 12
    }; 
    printf("1 = %g, 2 = %g, 3 = %g, 4 = %g, 5 = %g, 6 = %g, 7 = %g, 8 = %g, 9 = %g, 10 = %g, 11 = %g, 12 = %g\n", 
           sbuf->data[0], 
           sbuf->data[1], 
           sbuf->data[2],
           sbuf->data[3],
           sbuf->data[4],
           sbuf->data[5],
           sbuf->data[6],
           sbuf->data[7],
           sbuf->data[8],
           sbuf->data[9],
           sbuf->data[10],
           sbuf->data[11]);
    producer(sbuf, s);  // Пишем данные 0, 10, 20, 30, 40
    i++;
    // sleep(1);
  }

  munmap(sbuf, sizeof(SharedBuffer));
  shm_unlink("in");
  return 0;
}
