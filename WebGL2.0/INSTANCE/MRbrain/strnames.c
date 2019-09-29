#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <string.h>

 /* reverse:  reverse string s in place */
 void reverse(char s[])
 {
     int i, j;
     char c;

     for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
         c = s[i];
         s[i] = s[j];
         s[j] = c;
     }
}

/* itoa:  convert n to characters in s */
 void itoa(int n, char s[])
 {
     int i, sign;

     if ((sign = n) < 0)  /* record sign */
         n = -n;          /* make n positive */
     i = 0;
     do {       /* generate digits in reverse order */
         s[i++] = n % 10 + '0';   /* get next digit */
     } while ((n /= 10) > 0);     /* delete it */
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
}

int fd;
short gray;

int main(int argc, char** argv)
{
char out[20];
char buf[20];
for(int i=1; i<110; i++)
{
     itoa(i, buf);
     strcpy(out, "MRbrain.");
     strcat(out, buf);
     fd = open(out, O_RDONLY);
     //printf("%d\n", fd);
     //close(fd);
//     printf("%s\n", out);

}
char data[256*256*2];
for(int j = 3; j<112; j++)
{
int n = read( j,  data, 256*256*2);
//printf("%d\n", n);

for(int i = 0; i< 256*256*2; i+=2)
{
     gray = 256*data[i]+data[i+1];
     printf("%d, ", gray);
}
}
return 0;
}
