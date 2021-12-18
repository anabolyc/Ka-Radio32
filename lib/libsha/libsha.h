#ifndef __SHA_H__
#define __SHA_H__

#include <stdint.h>
#include <sys/types.h>

typedef struct
{
    uint32_t hash[5]; // hash value
    size_t count;
    unsigned char buffer[64]; // a single 512 bit block buffer
} SHA1Context;

void SHA1Init(SHA1Context *context);

void SHA1Update(SHA1Context *context, const unsigned char *data, size_t len);

void SHA1Final(unsigned char digest[20], SHA1Context *context);

#endif