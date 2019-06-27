#include "prng.h"

// From http://burtleburtle.net/bob/rand/smallprng.html.

void SmallPRNG::SetSeed(uint32_t seed) {
  u4 i;
  ranctx* x = &context_;
  x->a = 0xf1ea5eed, x->b = x->c = x->d = seed;
  for (i=0; i<20; ++i) {
    (void)Get();
  }
}

#define rot(x,k) (((x)<<(k))|((x)>>(32-(k))))

uint32_t SmallPRNG::Get() {
  ranctx* x = &context_;
  u4 e = x->a - rot(x->b, 27);
  x->a = x->b ^ rot(x->c, 17);
  x->b = x->c + x->d;
  x->c = x->d + e;
  x->d = e + x->a;
  return x->d;
}