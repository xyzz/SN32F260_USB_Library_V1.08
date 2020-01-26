#ifndef COMPILER_H_
#define COMPILER_H_

#if defined(__GNUC__)   // Code Red tools
  #define PRE_PACK
  #define POST_PACK     __attribute__((packed))

// The following two typedefs are required as GCC does
// not directly support a method of hinting that a variable
// (as opposed to a structure) should be accessed with
// the assumption that it is unaligned, which can be done
// in Keil using, for example, __packed uint32_t *p;
typedef struct { uint32_t value __attribute__(( packed ));
}unaligned_uint32;

typedef struct { uint16_t value __attribute__(( packed ));
}unaligned_uint16;

#else                   // Other toolchain
  #define PRE_PACK      __packed
  #define POST_PACK
  #define PACKVAR		__attribute__((packed))

#endif


#endif /* COMPILER_H_ */
