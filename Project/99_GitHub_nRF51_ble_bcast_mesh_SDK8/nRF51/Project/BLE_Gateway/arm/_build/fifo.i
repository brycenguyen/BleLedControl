#line 1 "..\\..\\..\\rbc_mesh\\src\\fifo.c"

































 
#line 1 "..\\..\\..\\rbc_mesh\\include\\fifo.h"

































 



#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"
 
 





 







 




  
 







#line 37 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"







 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     
typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;

     
typedef   signed       __int64 intmax_t;
typedef unsigned       __int64 uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 208 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"

     







     










     











#line 272 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"



 



#line 39 "..\\..\\..\\rbc_mesh\\include\\fifo.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdbool.h"
 






 





#line 25 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdbool.h"



#line 40 "..\\..\\..\\rbc_mesh\\include\\fifo.h"

 
typedef void (*fifo_memcpy)(void* dest, const void* src);

typedef struct
{
  void* elem_array;
  uint32_t elem_size;
  uint32_t array_len;
  uint32_t head;
  uint32_t tail;
  fifo_memcpy memcpy_fptr;  
} fifo_t;

void fifo_init(fifo_t* p_fifo);
uint32_t fifo_push(fifo_t* p_fifo, const void* p_elem);
uint32_t fifo_pop(fifo_t* p_fifo, void* p_elem);
uint32_t fifo_peek_at(fifo_t* p_fifo, void* p_elem, uint32_t elem);
uint32_t fifo_peek(fifo_t* p_fifo, void* p_elem);
void fifo_flush(fifo_t* p_fifo);
uint32_t fifo_get_len(fifo_t* p_fifo);
_Bool fifo_is_full(fifo_t* p_fifo);
_Bool fifo_is_empty(fifo_t* p_fifo);



#line 36 "..\\..\\..\\rbc_mesh\\src\\fifo.c"
#line 1 "..\\..\\..\\rbc_mesh\\include\\rbc_mesh_common.h"

































 

#line 39 "..\\..\\..\\rbc_mesh\\include\\rbc_mesh_common.h"





 
#line 64 "..\\..\\..\\rbc_mesh\\include\\rbc_mesh_common.h"

#line 73 "..\\..\\..\\rbc_mesh\\include\\rbc_mesh_common.h"

















#line 97 "..\\..\\..\\rbc_mesh\\include\\rbc_mesh_common.h"











        

        
#line 37 "..\\..\\..\\rbc_mesh\\src\\fifo.c"
#line 1 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\nrf_error.h"


































  
 




 

 




 




 

#line 73 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\nrf_error.h"





 
#line 38 "..\\..\\..\\rbc_mesh\\src\\fifo.c"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"
 
 
 
 




 








 












#line 38 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"


  
  typedef unsigned int size_t;








extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 185 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 201 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 224 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 239 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 262 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 494 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"



 

#line 39 "..\\..\\..\\rbc_mesh\\src\\fifo.c"



 
static void* s_fifo_at(fifo_t* p_fifo, uint32_t index)
{
  return ((uint8_t*) p_fifo->elem_array + p_fifo->elem_size * index);
}



 
void fifo_init(fifo_t* p_fifo)
{
     
    uint32_t i = 32;
    while (!((p_fifo->array_len >> --i) & 0x01));
    p_fifo->array_len = (1 << i);

    p_fifo->head = 0;
    p_fifo->tail = 0;
}

uint32_t fifo_push(fifo_t* p_fifo, const void* p_elem)
{
    if (fifo_is_full(p_fifo))
    {
        return ((0x0) + 4);
    }

  void* p_dest = s_fifo_at(p_fifo, p_fifo->head & (p_fifo->array_len - 1));

  if (p_fifo->memcpy_fptr)
    p_fifo->memcpy_fptr(p_dest, p_elem);
  else
    memcpy(p_dest, p_elem, p_fifo->elem_size);

  ++p_fifo->head;
  return ((0x0) + 0);
}

uint32_t fifo_pop(fifo_t* p_fifo, void* p_elem)
{
  if (fifo_is_empty(p_fifo))
  {
    return ((0x0) + 14);
  }

  void* p_src = s_fifo_at(p_fifo, p_fifo->tail & (p_fifo->array_len - 1));

  if (p_fifo->memcpy_fptr)
    p_fifo->memcpy_fptr(p_elem, p_src);
  else
    memcpy(p_elem, p_src, p_fifo->elem_size);

  ++p_fifo->tail;

  return ((0x0) + 0);
}

uint32_t fifo_peek_at(fifo_t* p_fifo, void* p_elem, uint32_t elem)
{
  if (fifo_get_len(p_fifo) <= elem)
  {
    return ((0x0) + 14);
  }

  void* p_src = s_fifo_at(p_fifo, (p_fifo->tail + elem) & (p_fifo->array_len - 1));

  if (p_fifo->memcpy_fptr)
    p_fifo->memcpy_fptr(p_elem, p_src);
  else
    memcpy(p_elem, p_src, p_fifo->elem_size);

  return ((0x0) + 0);
}

uint32_t fifo_peek(fifo_t* p_fifo, void* p_elem)
{
  return fifo_peek_at(p_fifo, p_elem, 0);
}

void fifo_flush(fifo_t* p_fifo)
{
  p_fifo->tail = p_fifo->head;
}

uint32_t fifo_get_len(fifo_t* p_fifo)
{
  return (p_fifo->head - p_fifo->tail);
}

_Bool fifo_is_full(fifo_t* p_fifo)
{
  return (p_fifo->tail + p_fifo->array_len == p_fifo->head);
}

_Bool fifo_is_empty(fifo_t* p_fifo)
{
  return (p_fifo->tail == p_fifo->head);
}
