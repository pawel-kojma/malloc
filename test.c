#include "mm.h"
#include "memlib.h"
#include <string.h>

void *ptr(long mem) {
  return (void *)(0x800000000 | mem);
}

void *mem_malloc(size_t size) {
  long *mem = mm_malloc(size);
  memset(ptr((long)mem), 0x1, size);
  return ptr((long)mem);
}

/*
0x10 - 0x20
0x21 - 0x40
0x41 - 0x80
0x81 - 0x100



*/

int main(int argc, char const *argv[]) {
  mem_init();
  mm_init();
  long *mem = mem_malloc(0x20);
  long *mem2 = mem_malloc(0x30);
  long *mem3 = mem_malloc(0x50);
  long *mem4 = mem_malloc(0x90);
  long *mem5 = mem_malloc(0x80000);
  mm_free(mem);
  mm_free(mem3);
  mm_free(mem5);
  // mem_malloc(5);
  // mem_malloc(5);
  // mm_free(mem2);

  // mem_malloc(12);
  return 0;
}