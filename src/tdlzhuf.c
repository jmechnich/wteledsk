/* tdlzhuf.c  module to perform lzss-huffman decompression
   as is used in teledisk.exe
   Conditionally compiled main to convert *.td0 advanced 
   compression file back normal compression file.
   derived from lzhuf.c 1.0 per below 

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
    http://www.gnu.org/licenses/gpl.txt


    11/10/02 this was working with struct tdlzhuf * passed to
    both Decode() and init_Decode() as first arg.  Save this as
    tdlzhuf1.c and then make this structure locally static and try
    to switch to unsigned shorts where it matters so works in linux.

Started with program below: 
 * LZHUF.C English version 1.0
 * Based on Japanese version 29-NOV-1988
 * LZSS coded by Haruhiko OKUMURA
 * Adaptive Huffman Coding coded by Haruyasu YOSHIZAKI
 * Edited and translated to English by Kenji RIKITAKE

In summary changes by WTK:
  wrote a new conditionally compiled main() 
  remove Encode() modules and arrays
  make remaing arrays and variables static to hide from external modules
  add struct tdlzhuf to provide state retension between calls to Decode()
  change from fgetc(FILE *) to read(int fp) so read
     a block at a time into an input buffer.  Now the
     Decode() routine can be called instead of read()
     by a user, ie from wteledisk.c
  change size of data elements for Linux, 
     int -> short
     unsigned int -> unsigned short
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#ifndef msdos
#include <unistd.h>
#endif

/* WTK adds top level control structure to give Decode()
   a memory between calls
*/
#define BUFSZ           512     // new input buffer

static struct tdlzhuf {
int fp;  // source file handle, opened by caller
         // next four variables were local in original main()
         // we need to save these values between calls to Decode()
unsigned short r,
               bufcnt,bufndx,bufpos,  // string buffer
         // the following to allow block reads from input in next_word()
               ibufcnt,ibufndx; // input buffer counters
unsigned char  inbuf[BUFSZ];    // input buffer
} tdctl;




/* LZSS Parameters */

#define N        4096    /* Size of string buffer */
#define F        60    /* Size of look-ahead buffer */
#define THRESHOLD    2
#define NIL        N    /* End of tree's node  */

static unsigned char
        text_buf[N + F - 1];
static short    match_position, match_length,
        lson[N + 1], rson[N + 257], dad[N + 1];

void InitTree(void)  /* Initializing tree */
{
    int  i;

    for (i = N + 1; i <= N + 256; i++)
        rson[i] = NIL;            /* root */
    for (i = 0; i < N; i++)
        dad[i] = NIL;            /* node */
}

void InsertNode(int r)  /* Inserting node to the tree */
{
    int  i, p, cmp;
    unsigned char  *key;
    unsigned c;

    cmp = 1;
    key = &text_buf[r];
    p = N + 1 + key[0];
    rson[r] = lson[r] = NIL;
    match_length = 0;
    for ( ; ; ) {
        if (cmp >= 0) {
            if (rson[p] != NIL)
                p = rson[p];
            else {
                rson[p] = r;
                dad[r] = p;
                return;
            }
        } else {
            if (lson[p] != NIL)
                p = lson[p];
            else {
                lson[p] = r;
                dad[r] = p;
                return;
            }
        }
        for (i = 1; i < F; i++)
            if ((cmp = key[i] - text_buf[p + i]) != 0)
                break;
        if (i > THRESHOLD) {
            if (i > match_length) {
                match_position = ((r - p) & (N - 1)) - 1;
                if ((match_length = i) >= F)
                    break;
            }
            if (i == match_length) {
                if ((c = ((r - p) & (N - 1)) - 1) < match_position) {
                    match_position = c;
                }
            }
        }
    }
    dad[r] = dad[p];
    lson[r] = lson[p];
    rson[r] = rson[p];
    dad[lson[p]] = r;
    dad[rson[p]] = r;
    if (rson[dad[p]] == p)
        rson[dad[p]] = r;
    else
        lson[dad[p]] = r;
    dad[p] = NIL;  /* remove p */
}

void DeleteNode(int p)  /* Deleting node from the tree */
{
    int  q;

    if (dad[p] == NIL)
        return;            /* unregistered */
    if (rson[p] == NIL)
        q = lson[p];
    else
    if (lson[p] == NIL)
        q = rson[p];
    else {
        q = lson[p];
        if (rson[q] != NIL) {
            do {
                q = rson[q];
            } while (rson[q] != NIL);
            rson[dad[q]] = lson[q];
            dad[lson[q]] = dad[q];
            lson[q] = lson[p];
            dad[lson[p]] = q;
        }
        rson[q] = rson[p];
        dad[rson[p]] = q;
    }
    dad[q] = dad[p];
    if (rson[dad[p]] == p)
        rson[dad[p]] = q;
    else
        lson[dad[p]] = q;
    dad[p] = NIL;
}

/* Huffman coding parameters */

#define N_CHAR      (256 - THRESHOLD + F)
                /* character code (= 0..N_CHAR-1) */
#define T         (N_CHAR * 2 - 1)    /* Size of table */
#define R         (T - 1)            /* root position */
#define MAX_FREQ    0x8000
                    /* update when cumulative frequency */
                    /* reaches to this value */

typedef unsigned char uchar;

/*
 * Tables for encoding/decoding upper 6 bits of
 * sliding dictionary pointer
 */

/* decoder table */
static uchar d_code[256] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
    0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
    0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
    0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
    0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
    0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
    0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
    0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
    0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
    0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09,
    0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A,
    0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
    0x0C, 0x0C, 0x0C, 0x0C, 0x0D, 0x0D, 0x0D, 0x0D,
    0x0E, 0x0E, 0x0E, 0x0E, 0x0F, 0x0F, 0x0F, 0x0F,
    0x10, 0x10, 0x10, 0x10, 0x11, 0x11, 0x11, 0x11,
    0x12, 0x12, 0x12, 0x12, 0x13, 0x13, 0x13, 0x13,
    0x14, 0x14, 0x14, 0x14, 0x15, 0x15, 0x15, 0x15,
    0x16, 0x16, 0x16, 0x16, 0x17, 0x17, 0x17, 0x17,
    0x18, 0x18, 0x19, 0x19, 0x1A, 0x1A, 0x1B, 0x1B,
    0x1C, 0x1C, 0x1D, 0x1D, 0x1E, 0x1E, 0x1F, 0x1F,
    0x20, 0x20, 0x21, 0x21, 0x22, 0x22, 0x23, 0x23,
    0x24, 0x24, 0x25, 0x25, 0x26, 0x26, 0x27, 0x27,
    0x28, 0x28, 0x29, 0x29, 0x2A, 0x2A, 0x2B, 0x2B,
    0x2C, 0x2C, 0x2D, 0x2D, 0x2E, 0x2E, 0x2F, 0x2F,
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
    0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,
};

static uchar d_len[256] = {
    0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
    0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
    0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
    0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
    0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
    0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
    0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
    0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
    0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
    0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
    0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
    0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
    0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
    0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
    0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
    0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
    0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
    0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
    0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
    0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
    0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
    0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
    0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
    0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
    0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
    0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
    0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
    0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
    0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
    0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
    0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
    0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
};

static unsigned short freq[T + 1];    /* cumulative freq table */

/*
 * pointing parent nodes.
 * area [T..(T + N_CHAR - 1)] are pointers for leaves
 */
short prnt[T + N_CHAR];

/* pointing children nodes (son[], son[] + 1)*/
short son[T];

static unsigned short getbuf = 0;
static uchar getlen = 0;

/* this is old code duplicated in GetBit() and GetByte()
    while (getlen <= 8) {
        if ((i = getc(infile)) < 0) i = 0;
        getbuf |= i << (8 - getlen);
        getlen += 8;
    }

replace this with next_word() routine that handles block read
*/
int next_word(int fp)
{
    if(tdctl.ibufndx >= tdctl.ibufcnt)
    {
        tdctl.ibufndx = 0;
        tdctl.ibufcnt = read(fp,tdctl.inbuf,BUFSZ);
        if(tdctl.ibufcnt <= 0)
            return(-1);
    }
    while (getlen <= 8) { // typically reads a word at a time
        getbuf |= tdctl.inbuf[tdctl.ibufndx++] << (8 - getlen);
        getlen += 8;
    }
    return(0);
}


int GetBit(int fp)    /* get one bit */
{
    short i;
    if(next_word(fp) < 0)
        return(-1);
    i = getbuf;
    getbuf <<= 1;
    getlen--;
        if(i < 0)
        return(1);
    else
        return(0);
}

int GetByte(int fp)    /* get a byte */
{
    unsigned short i;
        if(next_word(fp) != 0)
        return(-1);
    i = getbuf;
    getbuf <<= 8;
    getlen -= 8;
        i = i >> 8;
    return((int) i);
}



/* initialize freq tree */

void StartHuff()
{
    int i, j;

    for (i = 0; i < N_CHAR; i++) {
        freq[i] = 1;
        son[i] = i + T;
        prnt[i + T] = i;
    }
    i = 0; j = N_CHAR;
    while (j <= R) {
        freq[j] = freq[i] + freq[i + 1];
        son[j] = i;
        prnt[i] = prnt[i + 1] = j;
        i += 2; j++;
    }
    freq[T] = 0xffff;
    prnt[R] = 0;
}


/* reconstruct freq tree */

void reconst()
{
    short i, j, k;
    unsigned short f, l;

    /* halven cumulative freq for leaf nodes */
    j = 0;
    for (i = 0; i < T; i++) {
        if (son[i] >= T) {
            freq[j] = (freq[i] + 1) / 2;
            son[j] = son[i];
            j++;
        }
    }
    /* make a tree : first, connect children nodes */
    for (i = 0, j = N_CHAR; j < T; i += 2, j++) {
        k = i + 1;
        f = freq[j] = freq[i] + freq[k];
        for (k = j - 1; f < freq[k]; k--);
        k++;
        l = (j - k) * 2;
        
        /* movmem() is Turbo-C dependent
           rewritten to memmove() by Kenji */
        
        /* movmem(&freq[k], &freq[k + 1], l); */
        (void)memmove(&freq[k + 1], &freq[k], l);
        freq[k] = f;
        /* movmem(&son[k], &son[k + 1], l); */
        (void)memmove(&son[k + 1], &son[k], l);
        son[k] = i;
    }
    /* connect parent nodes */
    for (i = 0; i < T; i++) {
        if ((k = son[i]) >= T) {
            prnt[k] = i;
        } else {
            prnt[k] = prnt[k + 1] = i;
        }
    }
}


/* update freq tree */

void update(int c)
{
    int i, j, k, l;

    if (freq[R] == MAX_FREQ) {
        reconst();
    }
    c = prnt[c + T];
    do {
        k = ++freq[c];

        /* swap nodes to keep the tree freq-ordered */
        if (k > freq[l = c + 1]) {
            while (k > freq[++l]);
            l--;
            freq[c] = freq[l];
            freq[l] = k;

            i = son[c];
            prnt[i] = l;
            if (i < T) prnt[i + 1] = l;

            j = son[l];
            son[l] = i;

            prnt[j] = c;
            if (j < T) prnt[j + 1] = c;
            son[c] = j;

            c = l;
        }
    } while ((c = prnt[c]) != 0);    /* do it until reaching the root */
}


short DecodeChar(int fp)
{
    int ret;
    unsigned short c;

    c = son[R];

    /*
     * start searching tree from the root to leaves.
     * choose node #(son[]) if input bit == 0
     * else choose #(son[]+1) (input bit == 1)
     */
    while (c < T) {
                if((ret = GetBit(fp)) < 0)
            return(-1);
        c += (unsigned) ret;
        c = son[c];
    }
    c -= T;
    update(c);
    return c;
}

short DecodePosition(int fp)
{
    short bit;
    unsigned short i, j, c;

    /* decode upper 6 bits from given table */
        if((bit=GetByte(fp)) < 0)
        return(-1);
    i = (unsigned short) bit;
    c = (unsigned short)d_code[i] << 6;
    j = d_len[i];

    /* input lower 6 bits directly */
    j -= 2;
    while (j--) {
                if((bit = GetBit(fp)) < 0)
                     return(-1);
        i = (i << 1) + bit;
    }
    return(c | (i & 0x3f));
}

/* DeCompression 

split out initialization code to init_Decode()

*/

void init_Decode(int fp)
{
    int i;
    tdctl.ibufcnt= tdctl.ibufndx = 0; // input buffer is empty
    tdctl.bufcnt = 0;
    StartHuff();
    for (i = 0; i < N - F; i++)
        text_buf[i] = ' ';
    tdctl.r = N - F;
    tdctl.fp = fp;
}


int Decode(unsigned char *buf, int len)  /* Decoding/Uncompressing */
{
    short c,pos;
    int  count;  // was an unsigned long, seems unnecessary
    for (count = 0; count < len; ) {
            if(tdctl.bufcnt == 0) {
                if((c = DecodeChar(tdctl.fp)) < 0)
                    return(count); // fatal error
                if (c < 256) {
                    *(buf++) = c;
                    text_buf[tdctl.r++] = c;
                    tdctl.r &= (N - 1);
                    count++;                
                } 
                else {
                    if((pos = DecodePosition(tdctl.fp)) < 0)
                           return(count); // fatal error
                    tdctl.bufpos = (tdctl.r - pos - 1) & (N - 1);
                    tdctl.bufcnt = c - 255 + THRESHOLD;
                    tdctl.bufndx = 0;
                 }
            }
            else { // still chars from last string
                while( tdctl.bufndx < tdctl.bufcnt && count < len ) {
                    c = text_buf[(tdctl.bufpos + tdctl.bufndx) & (N - 1)];
                    *(buf++) = c;
                    tdctl.bufndx++;
                    text_buf[tdctl.r++] = c;
                    tdctl.r &= (N - 1);
                    count++;
                }
                // reset bufcnt after copy string from text_buf[]
                if(tdctl.bufndx >= tdctl.bufcnt) 
                    tdctl.bufndx = tdctl.bufcnt = 0;
        }
    }
    return(count); // count == len, success
}

#ifdef DOFILE
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#ifndef MSDOS
#define O_BINARY 0
#endif
unsigned char obuf[BUFSZ];

main(int argc, char *argv[])
{
    unsigned char *ptr,head[12],chead[10];
    int fp,fo,rd;
    long off=0;
    unsigned short crc=0;


    if(argc < 3)
    {
        printf("\nusage: tdconv <in-file> <out-file>");
        exit(0);
    }
    else if((fo = open(argv[2],O_RDWR|O_BINARY|O_CREAT|O_TRUNC,S_IREAD|S_IWRITE))
            == EOF)
    {
      printf("\nfailed to open output file %s",argv[2]);
      exit(1);
    }
    else if((fp = open(argv[1],O_RDONLY|O_BINARY)) == EOF ||
            read(fp,head,sizeof(head)) != sizeof(head))
        printf("\nfailed to open %s",argv[1]);
    else if(strncmp(head,"td",2) != 0)
        printf("\nthis is not a compressed teledisk image");
    else
    {
        off = sizeof(head);
        init_Decode(fp);

        head[0] = 'T';
        head[1] = 'D';  // make it a normal compression image!
        // but now need to fix crc
        crc = do_crc(head,10);
        *((unsigned short *)head +5) = crc;
        if(write(fo,head,sizeof(head)) != sizeof(head))
            printf("\nerror writing header");
        else
        do
        {
           if((rd = Decode(obuf,BUFSZ)) > 0)
               write(fo,obuf,rd);
           off += rd;
        }  while(rd == BUFSZ);
    }
    printf("\nwrote %ld bytes to output file\n",off);
}

#endif

