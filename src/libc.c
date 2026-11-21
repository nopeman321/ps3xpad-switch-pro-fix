#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void *memset(void *m, int c, size_t n)
{
	char *s = (char *) m;

	while (n-- != 0)
	{
		*s++ = (char) c;
	}

	return m;
}

void *memcpy(void *dst0, const void *src0, size_t len0)
{
	char *dst = (char *)dst0;
	char *src = (char *)src0;

	void *save = dst0;

	while (len0--)
		*dst++ = *src++;

	return save;
}

int memcmp(const void* s1, const void* s2,size_t n)
{
    const unsigned char *p1 = s1, *p2 = s2;
    while(n--)
        if( *p1 != *p2 )
            return *p1 - *p2;
        else
            *p1++,*p2++;
    return 0;
}


size_t strlen(const char *s) {
    const char *p = s;
    while (*s) ++s;
    return s - p;
}


char *strchr(const char *s, int c)
{
    while (*s != (char)c)
        if (!*s++)
            return 0;
    return (char *)s;
}


char *strstr(const char *s1, const char *s2)
{
    size_t n = strlen(s2);
    if (n == 0) return (char *)s1;
    while (*s1) {
        // Ensure there are at least n characters left in s1
        if (strlen(s1) < n) break;
        if (strncmp(s1, s2, n) == 0)
            return (char *)s1;
        s1++;
    }
    return 0;
}


int strncasecmp (__const char *s1, __const char *s2, size_t n)
{
  int c1, c2;
  while (n > 0)
    {
      c1 = *((unsigned char *)(s1++));
      if (c1 >= 'A' && c1 <= 'Z')
        c1 = c1 + ('a' - 'A');
      c2 = *((unsigned char *)(s2++));
      if (c2 >= 'A' && c2 <= 'Z')
        c2 = c2 + ('a' - 'A');
      if (c1 != c2)
        {
          return (c1 - c2);
        }
      if (c1 == '\0')
        {
          return 0;
        }
      --n;
    }
  return 0;
}

int strcasecmp (__const char *s1, __const char *s2)
{
    int c1, c2;
    while (*s1 && *s2)
    {
        c1 = *((unsigned char *)s1);
        c2 = *((unsigned char *)s2);
        if (c1 >= 'A' && c1 <= 'Z') c1 = c1 + ('a' - 'A');
        if (c2 >= 'A' && c2 <= 'Z') c2 = c2 + ('a' - 'A');
        if (c1 != c2)
            return c1 - c2;
        s1++; s2++;
    }
    // One or both strings ended
    c1 = *s1 ? *((unsigned char *)s1) : 0;
    c2 = *s2 ? *((unsigned char *)s2) : 0;
    if (c1 >= 'A' && c1 <= 'Z') c1 = c1 + ('a' - 'A');
    if (c2 >= 'A' && c2 <= 'Z') c2 = c2 + ('a' - 'A');
    return c1 - c2;
}

int strncmp(const char* s1, const char* s2, size_t n)
{
    if (n == 0) return 0;
    while (n && *s1 && *s2 && (*s1 == *s2))
    {
        s1++; s2++; n--;
    }
    if (n == 0) return 0;
    return (unsigned char)*s1 - (unsigned char)*s2;
}

int strcmp(const char* s1, const char* s2)
{
    while(*s1 && (*s1==*s2))
        s1++,s2++;
    return *(const unsigned char*)s1-*(const unsigned char*)s2;
}


char *strcpy(char *dest, const char* src)
{
    char *ret = dest;
    while (*dest++ = *src++)
        ;
    return ret;
}


char *strncpy(char *dest, const char *src, size_t n)
{
    char *ret = dest;
    do {
        if (!n--)
            return ret;
    } while (*dest++ = *src++);
    while (n--)
        *dest++ = 0;
    return ret;
}

char *strcat(char *dest, const char *src)
{
    char *ret = dest;
    while (*dest)
        dest++;
    while (*dest++ = *src++)
        ;
    return ret;
}

char *strtok_r(char *str, const char *delim, char **saveptr)
{
    // If str is NULL, continue from saved pointer
    if (str == NULL) {
        str = *saveptr;
        if (str == NULL) return NULL;
    }
    // Skip leading delimiters
    str += strspn(str, delim);
    if (*str == '\0') {
        *saveptr = NULL;
        return NULL;
    }
    // Find end of token
    char *end = str + strcspn(str, delim);
    if (*end != '\0') {
        *end = '\0';
        *saveptr = end + 1;
    } else {
        *saveptr = NULL;
    }
    return str;
}

/* Existing non-reentrant strtok kept for compatibility, but now forwards to strtok_r */
char *strtok(char *str, const char *delim)
{
    static char *p = NULL;
    return strtok_r(str, delim, &p);
}

size_t strcspn(const char *s1, const char *s2)
{
    size_t ret=0;
    while(*s1)
        if(strchr(s2,*s1))
            return ret;
        else
            s1++,ret++;
    return ret;
}

size_t strspn(const char *s1, const char *s2)
{
    size_t ret=0;
    while(*s1 && strchr(s2,*s1++))
        ret++;
    return ret;
}

