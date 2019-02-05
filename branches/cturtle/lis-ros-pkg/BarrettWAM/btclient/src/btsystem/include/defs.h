/*-----------------------------------------------------------------------------
defs.h -- Various definitions

Copyright (c) 1994 JANZ Computer AG
All Rights Reserved

Created 94/10/11 by Soenke Hansen
Version 1.10 of 97/02/03

Basic type definitions: bytes and words.
Conversion functions for words (16 bit unsigned).

-----------------------------------------------------------------------------*/

#ifndef defs_DEFINED
#define defs_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

#ifdef OS_9
#ifdef _UCC
#define _PARAMS(args) args
#else
#ifdef __STDC__
#define _PARAMS(args) args
#else
#define _PARAMS(args) () 
#endif
#endif
#endif

/* Function prototypes by default */
#ifndef _PARAMS
#define _PARAMS(args) args
#endif

/* Use of typedef's or #define's.  Some compilers (e.g., Ultra C for OS-9)
   don't like typedef's in prototype declarations. */
#ifndef TYPEDEFS
#define TYPEDEFS 1	/* default is with typedef's */
#endif

/* Generic object type */
#if TYPEDEFS == 1
typedef char * OBJECT;
#else
#define OBJECT char *	/* known to be dangerous! */
#endif
#define NIL_OBJECT (OBJECT)0

/* Return values (BYTE_t) */
#define SUCCESS		   0
#define FAILURE		0xff

/* Basic data types */
#if TYPEDEFS == 1
typedef unsigned char	BYTE_t;		/* 8 bit unsigned integer */
typedef unsigned short	WORD_t;		/* 16 bit unsigned integer */
typedef unsigned long	LWORD_t;	/* 32 bit unsigned integer */
#else
#define BYTE_t unsigned char
#define WORD_t unsigned short
#define LWORD_t unsigned long
#endif

/* Elaborate data types */
#ifndef OS_9
#if TYPEDEFS == 1
typedef void (*Funcptr)(void);		/* Having this makes life easier */
#endif
#endif

/* Convert between words and pairs of bytes */
#define word_read(p, w) ((w) = ((WORD_t)(p)[0] << 8)|((WORD_t)(p)[1]))
#define word_write(p, w) (((p)[0] = (BYTE_t)((w)>>8)), \
                          ((p)[1] = (BYTE_t)((w)&0x00ff)))

/* Extract bytes from word */
#define word_to_lsb(w)	((BYTE_t)((w)&0x00ff))
#define word_to_msb(w)	((BYTE_t)((w)>>8))

/* Assemble bytes into word */
#define word_from_bytes(lsb, msb) (((WORD_t)(msb) << 8)|((WORD_t)(lsb)))

/* Convert between long word and an array of 4 bytes */
#define lword_from_bytes(b) ((LWORD_t)(b)[0] + ((LWORD_t)(b)[1] << 8) + \
		((LWORD_t)(b)[2] << 16) + ((LWORD_t)(b)[3] << 24))
#define lword_to_bytes(b, lw) do { (b)[0] = (BYTE_t)(lw); \
	(b)[1] = (BYTE_t)(lw >> 8); (b)[2] = (BYTE_t)(lw >> 16); \
	(b)[3] = (BYTE_t)(lw >> 24); } while (0)


/* Compare 16 bit words which are assumed to have circular distance < 2^15 */
#define word_cmp(w1, w2) ((w1) - (w2) <= 0x7fff ? ((w1) == (w2) ? 0 : 1) : (-1))


#ifdef __cplusplus
}
#endif

#endif /* !defs_DEFINED */
