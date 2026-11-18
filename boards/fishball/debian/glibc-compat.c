/*
 * Run the C tools, cross-built against a newer glibc (the build host's
 * arm-linux-gnueabihf toolchain), on Debian 12's glibc 2.36.
 *
 * With _GNU_SOURCE, glibc >= 2.38 headers redirect strtol / strtoul to
 * __isoc23_strtol / __isoc23_strtoul (GLIBC_2.38: C23 also accepts "0b"
 * prefixes), and libm >= 2.38 has a new fmod (GLIBC_2.38). Defining those
 * here, on top of the GLIBC_2.4 versions every armhf glibc has, keeps the
 * binaries dynamically linked (so NSS host lookups use the board's own
 * modules, which a static glibc cannot) and loadable on glibc 2.36.
 * build-debs.sh links this object into every tool and fails if a symbol
 * above GLIBC_2.36 is still needed.
 *
 * Built without _GNU_SOURCE and without <stdlib.h> / <math.h>, so nothing
 * here is redirected again.
 */

extern long strtol_v24(const char *, char **, int);
extern unsigned long strtoul_v24(const char *, char **, int);
extern long long strtoll_v24(const char *, char **, int);
extern unsigned long long strtoull_v24(const char *, char **, int);
extern double fmod_v24(double, double);
__asm__(".symver strtol_v24,strtol@GLIBC_2.4");
__asm__(".symver strtoul_v24,strtoul@GLIBC_2.4");
__asm__(".symver strtoll_v24,strtoll@GLIBC_2.4");
__asm__(".symver strtoull_v24,strtoull@GLIBC_2.4");
__asm__(".symver fmod_v24,fmod@GLIBC_2.4");

long __isoc23_strtol(const char *s, char **end, int base)
{
	return strtol_v24(s, end, base);
}

unsigned long __isoc23_strtoul(const char *s, char **end, int base)
{
	return strtoul_v24(s, end, base);
}

long long __isoc23_strtoll(const char *s, char **end, int base)
{
	return strtoll_v24(s, end, base);
}

unsigned long long __isoc23_strtoull(const char *s, char **end, int base)
{
	return strtoull_v24(s, end, base);
}

double fmod(double x, double y)
{
	return fmod_v24(x, y);
}
