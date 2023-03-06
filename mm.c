/*
 Paweł Kojma 331757

  1. Blok wolny i zajęty

  Ogólne informacje o wszystkich blokach:
  1)  Rozmiar jest wielokrotnością 16 (ALIGNMENT)
  2)  Payload (miejsce na dane usera) zaczyna się pod adresem podzielnym przez
      ALIGNMENT
  3)  Każdy blok może zawierać do 4 flag w polu size, które określają przepływ
      wykonywania programu
  4)  Pole payload ma przynajmniej tyle bajtów ile trzeba było zaalokować

  - Blok wolny składa się z nagłówka, payloadu oraz footera,
  który służy do tego żeby złączanie bloków działało w O(1).
  W pierwszym quadwordzie payloadu mieszczą się zkompresowane do 4 bajtów
  pointery na następny i poprzedni blok na wolnej liscie bloków.
  Block w pamięci wygląda mniej więcej jak niżej
  (ft - footer,fd,bk - forward i backward pointery na wolne bloki,
  pola oznaczone x należą do innego bloku). Pola size,fd,bk,ft mają 4 bajty
  ---------------------------
  |xxxxxxxxxxxx| size |xxxxx|
  | fd  |  bk  |============|
  |=========================|
  .=========================.
  .=======Payload===========.
  .=========================.
  |============|xxxxxx| ft  |

  - Blok zajęty ma dużo prostszą strukturę, ponieważ
  znajduje się w nim tylko pole size oraz payload.

  2. Krótki opis przydziału i zwalniania bloków

  - Przydział (malloc)
    Procedura najpierw szuka już istniejącego bloku w listach wolnych bloków
 (Segregated Fits z wykładu), następnie jeśli znalazł pasujący blok, to w razie
 potrzeby podzieli go, a reszę zwróci do list wolnych bloków. Jeśli nie znalazł
 to prosi o więcej pamięci używając mem_sbrk() a następnie wstawia potrzebne
 metadane i zwraca. Jeśli okaże się ze rozmiar do zaalokowania mieści się w
 specjalnym przedziale małych bloków to malloc stworzy "drzazgi", czyli zalokuje
 określoną ilość bloków tego samego rozmiaru na raz i przydzieli pierwszy z
 nich. (więcej o tym czym są drzazgi niżej)

  - Zwalnianie (free)
    Procedura tworzy footer dla danego bloku i czyści flagę USED. Następnie
 jeśli blok jest oznaczony jako drzazga to, poprostu umieści blok na
 odpowiedniej liście wolnych bloków. Jeśli nie to natychniast złączy go z
 sąsiadującymi z nim innymi blokami (Jeśli tamte tez nie są drzazgami) i dopiero
 wtedy umieści na liscie wolnych bloków.

  - Zmiana rozmiaru bloku (realloc)
    Procedura najpierw sprawdza czy użytkownik nie zamierza skrócić bloku. W
 takim przypadku dzieli blok wg. podanego rozmiaru, a resztę umieszcza na
 odpowiedniej liście wolnych bloków lub złącza jeśli następny blok jest wolny i
 nie jest drzazgą. Jeśli użytkownik chce zwiększyć rozmiar bloku, to procedura
 patrzy czy dane blok nie jest ostatni na stercie, jeśli to ma miejsce to prosi
 o przydzielenie różnicy rozmiarów używając mem_sbrk(), jeśli tak nie jest to
 domyślnym rozwiązaniem jest zaalokowanie kompletnie nowego bloku o danym
 rozmiarze i zwolnieniem starego.

  3. Drzazgi

  Drzazga to blok wolny lub nie, który ma specjalną flagę SPLINTER w polu size.
 Tak jak było napisane w opisie przydziału tworzenie drzazg opiera się na
 zaalokowaniu określonej (stała SPLN_BLKS) liczby bloków na raz, tak aby były
 obok siebie w pamięci. Mechanizm ten ma na celu zredukować fragmentację, i
 kontrolować chaotyczne rozmieszczenie małych bloków na stercie. Generalne
 zasady dot. drzazg respektowane przez procedury przydziału/zwalniania to:
 1) Nie złącza się bloku, który jest drzazgą
 2) Wybrany blok z listy wolnych bloków to drzazga
    - Jeśli jest perfect-fitem (size ten sam) to zatrzymuje flagę SPLINTER
    - Jeśli jest potrzeba podzielić ten blok to oba stworzone bloki tracą flagę
 SPLINTER (Z testów wynika ze takie podejście jest efektywniejsze)
 3) Procedura do tworzenia drzazg jest uruchamiana tylko gdy size jest w
 określonym przedziale

  4. Segregated Fits

  Program używa struktury Segregated Fits opisanej na wykładzie. W tej
 implementacji każdy "kubełek" to podwójnie połączona zacyklona lista wiązana,
 której strażnikiem (Sentinel Element) jest element root, zaalokowany na
 początku przydzielonej sterty. Każdy kubełek ma logicznie przydzielony
 przedział rozmiarów bloków, jakie się w nim znajdują. W tym przypadku i-ty
 kubełek zawiera bloki o rozmiarze (2^i) + 1 do 2^(i+1), przy czym ostatni nie
 ma górnego ograniczenia i zawiera wszystkie większe bloki. Element root składa
 się z zkompresowanego pointera fd, bk oraz paddingu (sizeof(word_t)) na
 początku ze względów technicznych. szereg udostępnianych funkcji przez tą
 strukturę to m.in:
  - seg_findclass(size) - znajduje odpowiedni kubełek dla podanego rozmiaru
  - seg_unlink(chunk)   - zdejmuje blok z listy
  - seg_link(chunk)     - umieszcza blok na pasującej do niego liscie
  - seg_findfit(size)   - usiłuje znaleźć pasujący wolny blok dla podanego
 rozmiaru, zwraca NULL wpp

  Oprócz tego jest jeszcze pare wrapperów zdefiniowanych dla wygody
 implementacji. Stałą N_CLASS określa ilość dostępnych kubełków.
*/
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>

#include "mm.h"
#include "memlib.h"

/* If you want debugging output, use the following macro.  When you hand
 * in, remove the #define DEBUG line. */
#define DEBUG
#ifdef DEBUG
#define debug(...) printf(__VA_ARGS__)
#else
#define debug(...)
#endif

/* do not change the following! */
#ifdef DRIVER
/* create aliases for driver tests */
#define malloc mm_malloc
#define free mm_free
#define realloc mm_realloc
#define calloc mm_calloc
#endif /* def DRIVER */

typedef int32_t word_t; /* Sterta jest tablicą 4-bajtowych słów */

typedef enum {
  FREE = 0,      /* blok jest wolny */
  USED = 1,      /* blok jest używany */
  PREVINUSE = 2, /* Poprzedni blok jest używany (dla opt. BT) */
  SPLINTER = 4,  /* blok jest drzazgą i nie powinien być nigdy złączany */
} bt_flags;

static word_t *heap_start; /* Adres pierwszego bloku na stercie */
static word_t *last;       /* Adres ostatniego bloku na stercie */

/* --=[ Makra związane z BT, source: mm-implicit.c]=------------------- */

static inline word_t bt_size(word_t *bt) {
  return ((*bt) & (~(USED | PREVINUSE | SPLINTER)));
}

#define bt_used(bt) ((*bt) & USED)

#define bt_free(bt) (!((*bt) & USED))

#define bt_previnuse(bt) ((*bt) & PREVINUSE)

#define bt_splinter(bt) ((*bt) & SPLINTER)

#define bt_footer(ptr) (word_t *)(((void *)ptr + bt_size(ptr)) - sizeof(word_t))

#define bt_fromptr(ptr) (word_t *)(((word_t *)ptr - 1))

#define bt_make(bt, size, flags) (*bt = (size | flags))

#define bt_clr_previnuse(bt) (*bt &= ~PREVINUSE)

#define bt_set_previnuse(bt) (*bt |= PREVINUSE)

#define bt_clr_free(bt) (*bt &= ~FREE)

#define bt_clr_used(bt) (*bt &= ~USED)

#define bt_set_free(bt) (*bt |= FREE)

#define bt_set_used(bt) (*bt |= USED)

#define bt_payload(bt) (bt + 1)

#define bt_next(bt) (word_t *)(((void *)bt + bt_size(bt)))

#define bt_prev(bt) (word_t *)((void *)bt - bt_size(bt - 1))

#define bt_setfrom(base, offset) ((word_t *)((void *)base + offset))

#define bt_heapend() ((word_t *)(mem_heap_hi() + 1))

/* --=[ Segregated Fits API ]=------------------------------------------ */

#define N_CLASS 10
static word_t *seglists;
static word_t *root_end;

#define seg_ptr(ptr) ((word_t *)(ptr | 0x800000000))

#define seg_fromptr(ptr) ((word_t)((unsigned long)ptr & 0xffffffff))

#define seg_fdval(chunk) (*(chunk + 2))

#define seg_bkval(chunk) (*(chunk + 1))

#define seg_fd(chunk) (seg_ptr(seg_fdval(chunk)))

#define seg_bk(chunk) (seg_ptr(seg_bkval(chunk)))

#define seg_fdmake(chunk, fd) (*(chunk + 2) = fd)

#define seg_bkmake(chunk, fd) (*(chunk + 1) = fd)

#define seg_nextroot(root) (word_t *)(root + 3)

#define seg_rootat(i) (seglists + (i * 3))

/* wyszukaj kubełek dla danego rozmiaru bloku i zwróc roota */
static word_t *seg_findclass(size_t size) {
  size_t boundary = 32;
  int i = 0;
  for (; i < N_CLASS - 1; i++) {
    if (size <= boundary)
      break;
    boundary <<= 1;
  }
  return seg_rootat(i);
}

/* odłącz blok z listy, czyli przepinając pointery */
static inline void seg_unlink(word_t *chunk) {
  word_t *fd_ptr = seg_fd(chunk), *bk_ptr = seg_bk(chunk);
  seg_fdmake(bk_ptr, seg_fromptr(fd_ptr));
  seg_bkmake(fd_ptr, seg_fromptr(bk_ptr));
}

/* Umieść blok na liscie wg. polityki FIFO */
#if 0
static inline void seg_link(word_t *chunk, word_t *r) {
  word_t *root = (r == NULL) ? seg_findclass(bt_size(chunk)) : r;
  word_t *root_fd = seg_fd(root);

  seg_fdmake(root, seg_fromptr(chunk));
  seg_bkmake(chunk, seg_fromptr(root));

  seg_fdmake(chunk, seg_fromptr(root_fd));
  seg_bkmake(root_fd, seg_fromptr(chunk));
}
#else
/* Umieść blok na liscie wg. polityki LIFO */
static inline void seg_link(word_t *chunk, word_t *r) {
  /* Jeśli root został podany przez użytkownika to użyj go zamiast wyszukiwać */
  word_t *root = (r == NULL) ? seg_findclass(bt_size(chunk)) : r,
         *root_bk = seg_bk(root);
  word_t root_ptr = seg_fromptr(root), chunk_ptr = seg_fromptr(chunk);

  seg_fdmake(chunk, root_ptr);
  seg_bkmake(root, chunk_ptr);

  seg_fdmake(root_bk, chunk_ptr);
  seg_bkmake(chunk, seg_fromptr(root_bk));
}
#endif

/* Strategia Find-First */
static word_t *seg_findfit(size_t reqsz) {
  word_t *root = seg_findclass(reqsz);
  word_t *chunk;

  /* przeszukuje potencjalnie wszystkie listy wzwyż od startowej */
  for (; root != root_end; root = seg_nextroot(root)) {
    chunk = seg_fd(root);
    for (; chunk != root; chunk = seg_fd(chunk)) {
      if (reqsz <= bt_size(chunk))
        return chunk;
    }
  }
  return NULL;
}

/* --=[ funkcje generalnego użytku ]=--------------------------------------- */

static inline size_t blksz(size_t size) {
  return (size + sizeof(word_t) + ALIGNMENT - 1) & -ALIGNMENT;
}

static void *morecore(size_t size) {
  void *ptr = mem_sbrk(size);
  if (ptr == (void *)-1)
    return NULL;
  return ptr;
}

/* ustaw metadane wolnego bloku tj. header i footer */
static inline void set_free_metadata(word_t *chunk, size_t size,
                                     bt_flags add_flags) {
  bt_make(chunk, size, FREE | add_flags);
  word_t *footer = bt_footer(chunk);
  bt_make(footer, size, FREE | add_flags);
}

/* --=[ Procedura przydziału drzazg ]=------------------------------------ */
#define SPLN_BLKS 6
#define SPLN_MAX 0x50
static word_t *spln_malloc(size_t size) {
  word_t *chunk = (word_t *)morecore(SPLN_BLKS * size);
  if (chunk == (word_t *)NULL)
    return NULL;
  /* Pierwszy blok w serii, ten zwracamy do użytkownika */
  bt_make(chunk, size, SPLINTER | USED | (PREVINUSE * bt_used(last)));
  last = (word_t *)((void *)bt_heapend() - size);

  /* Następny blok musi mieć ustawioną flagę PREVINUSE więc jest poza pętlą */
  word_t *root = seg_findclass(size);
  word_t *next = bt_next(chunk);
  set_free_metadata(next, size, SPLINTER | PREVINUSE);
  seg_link(next, root);

  for (int i = 2; i < SPLN_BLKS; i++) {
    next = bt_next(next);
    set_free_metadata(next, size, SPLINTER);
    seg_link(next, root);
  }

  return chunk;
}

/* --=[ Inicjalizacja Sterty ]=------------------------------------ */

int mm_init(void) {
  /* alokujemy miejsce na rooty kubełków każdy z nich potrzebuje 3 słów
   * (padding,fb,bk) */
  if ((seglists = (word_t *)morecore(N_CLASS * (3 * sizeof(word_t)))) == NULL)
    return -1;
  int i = 0;
  word_t *root = seglists;
  word_t *init_end = (word_t *)mem_sbrk(0);

  /* Aby zachować alignment sterty musimy dopilnować by
     size field pierwszego bloku był po addressem 0x.*C */
  root_end = (word_t *)morecore(ALIGNMENT - (seg_fromptr(init_end) & 0xf) -
                                sizeof(word_t));

  /* inicjalizujemy fd i bk każdego roota na samego siebie */
  for (; i < N_CLASS; i++) {
    seg_fdmake(root, seg_fromptr(root));
    seg_bkmake(root, seg_fromptr(root));
    root = seg_nextroot(root);
  }

  heap_start = bt_heapend();
  last = heap_start;
  return 0;
}

/* --=[ Implementacja Malloc ]=------------------------------------ */

static void split(word_t *chunk, size_t total, size_t csize) {
  bt_flags flags = USED | bt_previnuse(chunk);
  bt_make(chunk, csize, flags);

  flags = FREE | PREVINUSE;

  word_t *nextchunk = bt_next(chunk);
  set_free_metadata(nextchunk, total - csize, flags);

  seg_unlink(chunk);
  seg_link(nextchunk, NULL);
  last = (chunk == last) ? nextchunk : last;
}

static void put_chunk(word_t *chunk, size_t size) {
  size_t chunksize = bt_size(chunk);
  if (chunksize == size) {
    bt_set_used(chunk);
    bt_set_previnuse(bt_next(chunk));
    seg_unlink(chunk);
  } else
    split(chunk, chunksize, size);
}

void *malloc(size_t size) {
  word_t *chunk;

  /*funkcja blksz zaokrągla rozmiar podany przez użytkownika do wielokrotnosci
   * ALIGNMENT */
  size = blksz(size);

  /* Jeśli nie udało się znaleźć pasującego bloku */
  if ((chunk = seg_findfit(size)) == NULL) {

    /* Jeśli rozmiar mieści się w przedziale tworzenia drzazg to stwórz je */
    if (size <= SPLN_MAX)
      chunk = spln_malloc(size);

    /* W przeciwnym przypadku prosimy o więcej pamięci wystarczającej na jeden
     * blok */
    else {
      chunk = (word_t *)morecore(size);
      if (chunk == (word_t *)NULL)
        return NULL;
      bt_make(chunk, size, USED | (PREVINUSE * bt_used(last)));
      last = chunk;
    }
  }
  /* Jeśli udało się znaleźć blok to ustaw metadane i ewentualnie podziel go
     jeśli jest za duży */
  else
    put_chunk(chunk, size);

  return bt_payload(chunk);
}

/* --=[ Implementacja Free ]=------------------------------------ */

/* funkcja coalesce zajmuje się złączaniem wolnych bloków na stercie */
static void coalesce(word_t *chunk) {
  word_t *next = bt_next(chunk), *prev = bt_prev(chunk);
  int next_free = bt_free(next), prev_free = !bt_previnuse(chunk);

  /* tutaj chcemy obsłużyć ewentualne edge-casy jakie mogą nastąpic */

  /* jeśli blok jest ostatni lub jest drzazgą to nie można złączać w przód */
  if ((chunk == last) || bt_splinter(next))
    next_free = 0;

  /* jeśli blok jest pierwszy na stercie lub jest drzazgą to nie można złączać w
   * tył */
  if (chunk == heap_start)
    prev_free = 0;
  else if (prev_free && bt_splinter(prev))
    prev_free = 0;

  size_t size = bt_size(chunk);
  if (prev_free) {
    seg_unlink(prev);
    if (next_free) { /* złącz next + prev */
      seg_unlink(next);
      last = (next == last) ? prev : last;
      size += bt_size(prev) + bt_size(next);
      set_free_metadata(prev, size, bt_previnuse(prev));
      chunk = prev;
    }

    else { /* złącz prev */
      last = (chunk == last) ? prev : last;
      size += bt_size(prev);
      set_free_metadata(prev, size, bt_previnuse(prev));
      chunk = prev;
    }

  } else if (next_free) { /* złącz next */
    seg_unlink(next);
    last = (next == last) ? chunk : last;
    size += bt_size(next);
    set_free_metadata(chunk, size, bt_previnuse(chunk));
  }

  /*na końu złączony blok umieść na odpowiedniej liście */
  seg_link(chunk, NULL);
}

void free(void *ptr) {
  if (ptr == NULL)
    return;

  word_t *chunk = bt_fromptr(ptr);

  /* ustaw odpowiednie metadane */
  set_free_metadata(chunk, bt_size(chunk),
                    bt_previnuse(chunk) | bt_splinter(chunk));

  /* jeśli blok to drzazga do umieść na liscie wolnych bloków, wpp złącz z
   * innymi */
  if (bt_splinter(chunk))
    seg_link(chunk, NULL);
  else
    coalesce(chunk);

  /* wyczyść flagę PREVINUSE z następnego bloku na stercie */
  bt_clr_previnuse(bt_next(chunk));
}

/* --=[ Implementacja Realloc ]=------------------------------------ */

void *realloc(void *old_ptr, size_t size) {
  /* Jeśli rozmiar to 0 to poprostu zwalniamy ten blok */
  if (size == 0) {
    free(old_ptr);
    return NULL;
  }

  /* Jeśli blok nie istnieje to przydzielamy nowy */
  if (!old_ptr)
    return malloc(size);

  word_t *block = bt_fromptr(old_ptr);
  size_t old_size = bt_size(block), csize = blksz(size);
  bt_flags origin_flags = bt_previnuse(block);

  /* jeśli skracamy istniejący blok */
  if (csize <= old_size) {
    if (csize == old_size)
      return old_ptr;

    /* dzielimy blok */
    bt_make(block, csize, USED | origin_flags);
    word_t *next = bt_next(block);
    set_free_metadata(next, old_size - csize, FREE | PREVINUSE);

    /* jeśli blok jest ostatni to nie zrozważamy złączania bloków */
    if (block == last) {
      last = next;
      seg_link(next, NULL);
    }

    /* w przeciwnym przypadku sprawdzamy czy następny blok jest wolny i nie jest
       drzazgą */
    else {
      word_t *nnext = bt_next(next);
      if (bt_free(nnext) && !bt_splinter(nnext))
        coalesce(next);
      else {
        bt_clr_previnuse(nnext);
        seg_link(next, NULL);
      }
    }

    return old_ptr;
  }

  /* jeśli blok jest ostatni na stercie to bierzemy więcej pamięci */
  if (block == last) {
    size_t asize = csize - old_size;
    (void)morecore(asize);
    bt_make(block, csize, USED | origin_flags);
    return old_ptr;
  }

  /* inaczej tworzymy zupełnie nowy blok o danym rozmiarze, przenosimy dane i
   * zwalniamy poprzedni */

  void *new_ptr = malloc(size);
  if (!new_ptr)
    return NULL;

  memmove(new_ptr, old_ptr, old_size);
  free(old_ptr);
  return new_ptr;
}

void *calloc(size_t nmemb, size_t size) {
  size_t bytes = nmemb * size;
  void *new_ptr = malloc(bytes);

  /* If malloc() fails, skip zeroing out the memory. */
  if (new_ptr)
    memset(new_ptr, 0, bytes);

  return new_ptr;
}

/* --=[ Implementacja mm_checkheap ]=------------------------------------ */

/*
  Funkcja zajmuje się sprawdzeniem spójności sterty. Ma też możliwość
  printowania ouputu w celach odpluskwiania kodu (flaga verbose > 0). Wypisuje
  podstawowe informacje na temat każdego bloku na stercie tj. Start, koniec,
  rozmiar, ustawione flagi, czy jest wolny czy nie. Oprócz tego funkcja zatrzyma
  się natychmiastowo gdy wykryje, że:
  - Ostatni blok nie jest ostatni
  - następny blok od aktualnie sprawdzanego jest poza stertą ( < 0x800000000 lub
  > 0x806400000 )
  - rozmiar headera i footera w wolnym bloku się nie pokrywają
  - ilość wolnych bloków na stercie i we wszystkich kubełkach nie są równe
  - używany blok ma przed sobą blok bez flagi PREVINUSE
  - wolny blok ma przed sobą blok z flagą PREVINUSE

*/

void mm_checkheap(int verbose) {
  word_t *header, *footer, *chunk;
  chunk = heap_start;

  assert((bt_next(last) == bt_heapend()) || (last == heap_start));
  int i = 0, free_chunks = 0;
  for (; chunk != bt_heapend(); chunk = bt_next(chunk)) {
    assert((word_t *)((void *)chunk + bt_size(chunk)) >=
             (word_t *)0x800000000 &&
           (word_t *)((void *)chunk + bt_size(chunk)) < (word_t *)0x806400000);
    footer = bt_footer(chunk);
    header = chunk;

    if (verbose > 0) {
      if (i == 0)
        printf("HEAP STRUCTURE\n");
      char *previnuse = (bt_previnuse(chunk)) ? "P" : "0";
      char *used = (bt_used(chunk)) ? "U" : "0";
      char *free = (bt_free(chunk)) ? "F" : "0";
      char *splinter = (bt_splinter(chunk)) ? "S" : "0";
      if (bt_free(chunk))
        printf("chunk %d: start %p, end %p, size %d/0x%x,\tflags %s|%s|%s|%s, "
               "%s %s\n",
               i, header, footer, bt_size(chunk), bt_size(chunk), splinter,
               previnuse, used, free, "FREE",
               (chunk == last) ? "<-- last" : "");
      else
        printf("chunk %d: start %p, end %p, size %d/0x%x,\tflags %s|%s|%s|%s, "
               "%s %s\n",
               i, header, (footer + 1), bt_size(chunk), bt_size(chunk),
               splinter, previnuse, used, free, "USED",
               (chunk == last) ? "<-- last" : "");
    }

    if (bt_free(chunk)) { /* free chunk */
      free_chunks++;
      assert(bt_size(footer) == bt_size(header));
      assert(!bt_previnuse(bt_next(chunk)) || chunk == last);
    } else { /* used chunk */
      assert(bt_previnuse(bt_next(chunk)) || chunk == last);
    }
    i++;
  }

  word_t *root = seglists;
  for (; root != root_end; root = seg_nextroot(root)) {
    chunk = seg_fd(root);
    for (; chunk != root; chunk = seg_fd(chunk)) {
      free_chunks--;
    }
  }
  assert(free_chunks == 0);
}