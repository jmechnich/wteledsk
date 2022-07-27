/* wteledsk.c  - teledisk *.td0 file decoding

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

   see wteldskc.c for development history notes
   see wteledsk.htm for a summary of key structures and algorithm

   Under MSDOS file redirection causes a run time stack overflow 
      unless a large stack size is forced with /ST:4096

   Define DCOMP if advanced decompression handling required and
      link with tdlzhuf.obj

   Define DCRC if crc testing desired and link with tdcrc.obj

   In code below both AKAI and DUMP are defined, for your application
   you may choose to comment these out.  With DUMP defined you need
   to supply a dump module (see protype below define).

   There is also a DISK define which I have used to recreate
   physical disks with the PC's low level bios routines.  This
   code is not part of the general distribution and DISK should
   not be defined.

*/

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <malloc.h>
#include <string.h> // define memset
#ifdef MSDOS
#include <io.h>         // for lseek
#else
#include <unistd.h>
#ifndef O_BINARY
#define O_BINARY 0      // not defined for Linux gcc
#endif
#define strnicmp strncasecmp
#endif
#include <sys/types.h>
#include <sys/stat.h>   /* for open() defines */

typedef unsigned char BYTE;
typedef unsigned short WORD;

// below are return defines from do_sector() so main knows whats going on
// size of block is returned in ctl!
#define AKAI_SEC 100 // special return flag for unnumbered AKAI sector   
#define AKAI_END 101 // termination for V2.16 phantom last track
#define SKIP_MASK 0x200 // bit 9 set if skipping a record, # skipped in low byte
#define REPT_MASK 0x400 // set bit 10 if repeating a sector, sec # in low byte

#define AKAI 1 // defining this makes it V2.16 compatible
               // not required for V2.12 ie the DEC images
#define BLKSZ 2048 
/* 2048 is good for lots of stuff, dynamically allocated in main
   average size of AKAI is 1024, DEC RX50 512
*/


#define SHSZ  6  // sector header size, this is base size, its variable

//#define DUMP 1  // enable dump options, if defined you must supply
                // your own dump routine per the prototype below

extern int dump(FILE *fp,unsigned char *buf,unsigned len,unsigned adr);

// force big stack was getting overflow if redirected output

unsigned adr; // global address for dump() calls, ultimately remove

#pragma pack(1)  // BYTE pack structures in MSC


#define COMMENT 0x80

struct file_head {
char sig[3];  // signature "TD", nul terminated string
BYTE  unkwn,  // doesn't seem to be used?
      ver,    // for v2.11 through v2.16 seems to be 0x15
      data1,  // 0x00 (except for akai)
      type,   // see bios drive type 1-4,
      flag,   // among other things can indicate comment field
      data2,  // often 0x00
      sides;  // 0x01 (always with rx50 or 0x02 with most MSDOS disks)
WORD  crc;  //   checksum of 1st 0xA bytes in record
};
/* 
   in most data1 = 0, but in akai its 0x02
   flag value above has been 0x80, 0x81, and 0
       see COMMENT define above, bitmaped to comment header below included
       looks like low order bits are physical drive number used
   if sides above is right, its odd, did each side separately on
   2 sided MSDOS disk, value was 0x01 for side 1 only and 0x02
   for side two only and for both sides.  
*/

struct com_head {
WORD crc, //  checksum of 8 bytes from &len to end of record
     len;     // length of string data region following date
BYTE yr,mon,day,
     hr,min,sec; // date and time info....
};
/* note com_head.len only makes sense as a byte, 
      but high byte of word has been 0....
   year won't wrap into there until 21xx  

   track_rec.crc below calculates crc for 1st three bytes
   of record, but only used the low byte of calculated value
   for comparison.   
*/

struct track_rec {
BYTE nsec,trk,head,crc;
};

struct sec_rec {
BYTE trk,head,sec;
BYTE secsz,  // sector size as power of 2 = 128 * (2 ** secsz)
     cntrl,  // see note below, 0x10 forces a truncated record
     crc, // if there is sector data, this is low byte of crc for entire sector
          // its NOT crc of 1st 5 bytes, see DSCRC
     unknwn[2],  
     flag;       // controls extra bytes and use

}; /* 
      flag value determins processing.  see notes in do_sector()
      secsz has always been 2 for 512 byte sector images, 
      however secsz = 3 for 1024 byte sector AKAI images.
      apparently sectors size = 128 * (2**secsz)
      cntrl often 0, but if 0x10 then record only 6 bytes long
         if clear its the full 9 byte record and there is extra data
         in unknwn[] I currently ignore, may be initializtion info?
   */


struct pat_rec {
BYTE flag,count,pat[16]; // if flag > 0, repeat pat[] count times
};
/* my V2.14 Teledisk only uses flag == 1 for RX50, ie 2 bytes of repeated data
   but AKAI records, V2.16,  seem to use at least 2 - 4 to create pattern sizes 
   which are a power of 2, ie 2**flag, see do_read()

   The case where flag = 0 is different, in this case count is
   the number of bytes <= 0xff to read from file to disk
*/

struct rep_rec {
     WORD count;
     BYTE pat[2];
};
/* this record only seems to occur once after sec_rec in the case
   that sec_rec.flag is valid and equal to 1
   I suspect count is redundant, that pat[] always repeated for entire
   sector?
*/


// control flags: values for globa ctl setup in main()
#define DHEAD  0x8
#define DBLOCK 0x10
#define DCNT   0x20  // limit range of dump
#define DWRITE 0x40  // write to disk sectors
#define WARN   0x80 // just a warning if skip sectors
#define REPEAT  0x100 // flag to write repeated sectors
#define PHANTOM 0x200 // flag to display phantom sec_rec.nsec value
#define BLKSZMSK 0x7 // low order 3 bits, only expect to use 2 of them


int block_size(unsigned ctl)
{
    int j,sz=128;
    j = ctl & BLKSZMSK;
    while(j-- > 0)
        sz *=2;
    return(sz);
}

unsigned char docomp = 0; // global flag set to 1 if advanced compression

#ifdef DCOMP
extern int Decode(unsigned char *buf,int len);
extern void init_Decode(int fp);
#else
// dummy decompression to satisfy linker
int Decode(unsigned char *buf,int len)
{
    printf("\nfatal error decompression not supported in this version\n");
    exit(-1);
}
#endif

#ifdef DCRC
extern unsigned short do_crc(unsigned char *b, unsigned short len);
extern void updt_crc(unsigned short *crc,unsigned char *b, unsigned short len);
#endif

int cread(int fp,void *buf,int len)
{
    int ret;
    if(docomp)
    {
        // see init_Decode() called from main which sets file handle
        ret = Decode(buf,len);
    }
    else
        ret = read(fp,buf,len);
    return(ret);
}

#ifdef DCOMP
#define read  cread  // redefine read
#endif

// append data to sbuf
int do_read(
int fd,
int *off,  // caller sets to 0, update as append data
BYTE *sbuf, // sector buffer
unsigned ctl)
{
    char buf[20];
    struct pat_rec  *prec = (struct pat_rec *)buf;

    int suc=0,i=0,j=0,npat=0,rd=0,blksz=0;
    blksz = block_size(ctl);
    while(suc == 0 && *off < blksz)
    {
         if((rd = read(fd,buf,2)) != 2)
              suc = 1;
         else if((i=buf[0]) > 0 && i < 5) // fill with pattern 
         {
             npat = 2;
             while(i-- > 1) 
                 npat *=2;  // bytes to repeat is a power of 2
             if((i = read(fd,&buf[rd],npat)) != npat)
                  suc = 1;
             else
             {
                  for(i=0; i < prec->count;i++)  // times to repeat pattern
                  {
                      for(j=0;j<npat;j++)  // # chars in pattern
                          *(sbuf + (*off)++) = prec->pat[j];
                  }
                  rd += npat;
             }
         }
         else if(buf[0] == 0)
         {    // in this case count is # chars to read from file <= 0xff
              if((i = read(fd,sbuf+ *off,(int)prec->count)) != prec->count)
                   suc = 1;
              else
              {
                   *off += i;
              }
         }
         else
         {
              suc = 2;
              printf("\nunknwn flag in do_read");
         }
#ifdef DUMP
         if(ctl & DHEAD)
             dump(stdout,buf,rd,adr);
#endif
         adr += rd;
         if(buf[0] == 0) // add this in after dump so just dump header
              adr += i; // update adr as also read from file to append

    }
    return(suc);
}

// step through headers and write it out
int do_sector(
int fd,
struct track_rec *trk, // current track record
BYTE *sbuf,
int sec,   // current sector #
WORD *ctl)
{
    BYTE *cptr,buf[30]; // biggest record I've seen is 15 bytes
    struct sec_rec *psec=(struct sec_rec *) buf;
    struct rep_rec *rbuf = (struct rep_rec *) (buf+sizeof(struct sec_rec));
    // if it exists the rep_rec immediately follows a full sec_rec when sec_rec.flag=1
    int  off=0,i,blksz=128,rd,suc=0;
    if((rd = read(fd,buf,SHSZ)) != SHSZ) // base record, 6 bytes long always there
         suc = 1;
    else if(trk->trk != psec->trk || trk->head != psec->head)
    {
         printf("\nsector record mismatch, trk or head value unexpected\n");
         suc = 2;
    }
#ifdef AKAI
    else if(!(*ctl & WARN) && (psec->sec & 0x60) != 0x60 && sec != psec->sec)
    {
         printf("\nsector record mismatch\n");
             suc = 2; // its a fatal error
    }
#else
    else if(!(*ctl & WARN) && sec != psec->sec)
    {
         printf("\nsector record mismatch\n");
             suc = 2; // its a fatal error
    }
#endif
    else if(psec->cntrl != 0x10) // 9 byte record, ie full struct I defined
    // cntrl == 0x10 is oddball
    {    
         if ((i = read(fd,&buf[SHSZ],3)) != 3)  // read last 3 bytes
             suc = 1;
         else
         {
             rd += i; 
             if(psec->flag == 1)
             {
                // rep_rec immediate follow, read now so dump() displays
                if((i= read(fd,rbuf,4)) != 4)
                    suc = 1;
                else
                    rd+= i;
             }
         }
    }
    // else its just a 6 byte record
    // see fudge after dump below, which may adjust fields 

#ifdef DUMP
#ifdef AKAI
    if((*ctl & PHANTOM) && (psec->sec & 0x60) == 0x60)
    {
         printf("Phantom sector # 0x%x on logical track %d\n",
         psec->sec,2*trk->trk+trk->head);
    }
#endif
    if(*ctl & DHEAD)
         dump(stdout,buf,rd,adr);
#endif
    adr += rd;


    if(psec->sec == 0x65)  // termination flag
        return(AKAI_END);  // end of valid data, early termination
    /*
         this return is done here as found some AKAI images
         where it used a 2048 phantom block size.  This was a
         problem when BLKSZ was 1024!  Have increased BLKSZ
         in case this happens elsewhere, but since I ignore
         phantom sector data, it makes sense to return here
    */

    *ctl &= ~BLKSZMSK; // clear block size bits
    i = psec->secsz;
    while(i-- > 0)
         blksz *=2;
    if(blksz <= BLKSZ)
        *ctl |= psec->secsz;
    else
    {
         printf("\nsector size of %d is too large for %d byte allocated buffer",
                  blksz,BLKSZ);
         suc = 3;
    }

    if(psec->cntrl == 0x10) // fudge it now as 9 byte record
    // data in file only 6 bytes long, set up last bytes
    {
         // don't really understand this one, seems to work with this fudge
         // should it be a NUL sector?
         psec->unknwn[0] = 0;
         psec->unknwn[1] = 5;
         psec->flag = 1; //fake a skipped record type
    }
    if(suc != 0)
         printf("\n abort");
    else if(psec->flag == 0)  // read entire sector from record
    {
       if((off = read(fd,sbuf,blksz)) != blksz)
          suc = 1;
       else
          adr += off;
    }
    else if(psec->flag == 1)   // skip sector its empty
    {
          if(psec->cntrl == 0x10) // just do a skip make buf blank, no crc?
              memset(sbuf,0,blksz); // optional extra data may be set info?
          else // note read rbuf above so the dump() displays it
          {   // new logic 10/19/02
                if(rbuf->count *2 != blksz)
                {
                    printf("\nrbuf->count %x does not match sector size %x",
                         rbuf->count,blksz);
                    suc = 3;
                }
                else 
                {   // fill with pattern
                    rd = rbuf->count;
                    cptr = sbuf;
                    while(rd-- > 0)
                    {
                        for(i=0;i<2;i++,cptr++)
                              *cptr = rbuf->pat[i];
                    }
                }        
                // now do crc!
          }
          off = blksz;
    }
    else if(psec->flag == 2)  // fragmented read
    {
         suc = do_read(fd,&off,sbuf,*ctl);
    } 
    else
    {
         suc = 2;
         printf("\nunknwn flag in do_sector");
    }

    if(suc == 1)
         printf("\nerror reading sector data");
    else if(suc == 0)
    {
#ifdef AKAI
        if((psec->sec & 0x60) == 0x60)
             suc = AKAI_SEC;  // tell caller its screwy "phantom" record
        else 

#endif
        if((*ctl & WARN) && sec != psec->sec)
        {
             if(sec < psec->sec)
             {
                 printf("\nWarning skipping  %u sectors (%u through %u) on track %u\n",
                     psec->sec - sec,sec,psec->sec-1,trk->trk);
                 suc = SKIP_MASK | ((psec->sec - sec) & 0xff); // # to skip
             }
             else
             {
                 printf("\nWarning repeating sector  %u on track %u\n",
                     psec->sec,trk->trk);
                 suc = REPT_MASK | (psec->sec & 0xff); // repeated sector
             }
        }

#ifdef DCRC
        // lets look at crc now, suspect if cntrl != 0x10 it should be valid
        // there is valid data in sbuf
        if(suc == 0 && psec->cntrl != 0x10)
        {
            rd = do_crc(sbuf,blksz);

            if( (rd & 0xff) != psec->crc)
                printf("\nwarning sector crc error");
        }
#endif

    }

    return(suc);
}


// advance over skipped blocks if writing output
int write_skip(int fo,int skipped,int blksz,int trkcnt)
{
     char skip_msg[25];
     long offset,fpos;
     int rd=0,suc=0;
     offset = (long)blksz * skipped -16;
     sprintf(skip_msg,"Skip %2d blocks ",skipped);
     if(((rd=write(fo,skip_msg,16)) != 16 ||
        (fpos = lseek(fo,offset,SEEK_CUR)) < 0L))
     {
         printf("\nfatal seek error skipping %d block(s)",skipped);
         suc = -1;
     }
     
     return(suc);
}

/* this doesn't happen unless the image is questionable
   could make it smarter but on good images this doesn't occur
   so I'm not spending a lot of time here.  Just curious
   The pro\wps?.td0 images do a lot of this.  I added
   some logic in main to see how often skipped sectors are rewritten
*/
int write_repeat(int fo,unsigned char *buf,int blksz,int csec,int rsec)
{
    long fpos=0,rpos=0,off;
    int rd=0,suc=1;
    // csec is expected sector, rsec is # of sector repeating
    // calculate # bytes we seek backward from current position
    off = (rsec - csec) * blksz;
    if(off < 0 && (fpos = lseek(fo,0L,SEEK_CUR)) > 0L &&
       (rpos = lseek(fo,off,SEEK_CUR)) >= 0L &&
       (rd = write(fo,buf,blksz)) == blksz &&
       lseek(fo,fpos,SEEK_SET) > 0L)
         suc = 0;
    return(suc);
}


char *mons[] = {"Jan","Feb","Mar","April","May","June","July",
               "Aug","Sept","Oct","Nov","Dec"};

#define MAXSEC 20 // size of array to see which sectors in a track written

int main(int argc,char *argv[])
{
    int cnt=0,csec=0,fd=EOF,fo=EOF,rd,i=-1,tnul = 0,nnul = 0,suc=-1,seccnt=0,trkcnt=0;
    int blksz=0,skip_cnt=0,rept_cnt=0,max_sec=0;
    int skipped=0,ovrwrite=0,tot_rept=0,tot_skip=0;
    WORD ctl=0;
#ifdef DCRC
    WORD crc=0;
#endif
    long flen=0,bloc=0;
    BYTE drvb=0,*block=0,written[MAXSEC+1]={0};
    char drive=0;
    struct file_head fh_buf,*fhead=&fh_buf;
    struct com_head  ch_buf,*chead=&ch_buf;
    struct track_rec trk;

    if(argc > 2)
         for(i=2;i<argc;i++)
         {
              if(strnicmp(argv[i],"-dh",3) == 0)
                   ctl |= DHEAD;
              else if(strnicmp(argv[i],"-db",3) == 0)
                   ctl |= DBLOCK;
              else if(strnicmp(argv[i],"-n",2) == 0)
              {
                   ctl |= DCNT;
                   cnt = atoi(argv[i]+2);
              }
              else if(strnicmp(argv[i],"-s",2) == 0)
              {
                   ctl |= WARN;
              }
              else if(strnicmp(argv[i],"-r",2) == 0)
              {
                   ctl |= WARN; // just warn
                   ctl |= REPEAT; // rewrite repeats
              }
              else if(strnicmp(argv[i],"-p",2) == 0)
              {
                   ctl |= PHANTOM;
              }
              else if(strnicmp(argv[i],"-w",2) == 0)
              {
                 drive = toupper(*argv[1]);
                 drvb = drive - 'A';
                 if(drvb < 2)
                     ctl |= DWRITE;
                 else
                     printf("\nerror for -w<drv> must be A or B");

              }
              else if(strnicmp(argv[i],"-o",2) == 0)
                   if((fo=open(argv[i]+2,O_BINARY|O_RDWR|O_CREAT|O_TRUNC,
                        S_IREAD|S_IWRITE)) == EOF)
                        printf("\nfailed to open output file: %s",argv[i]+2);
         }
    if(argc < 2)
    {
         printf("\nusage: wteledsk <filename> [-n#] -o<outputfile> [-p] [-s] [-r]");
#ifdef DISK
         printf(" [-w<drv>]");
#endif
#ifdef DUMP
         printf(" [-dh] [-db]\n       -d to dump headers or restrutured block data");         
#endif

         printf("\n       -n limit scan to first n sectors");
         printf("\n       -o output restructured blocks to a file");
         printf("\n       -p display phantom sec_rec.nsec values");
         printf("\n       -s warn instead of fatal error on skipped sectors");
         printf("\n       -r if repeated sector found, write it out again");
#ifdef DISK
         printf("\n       -w write restructured blocks to floppy disk <drv> sectors");
#endif
         printf("\n Version 1.01 ");
#ifdef DCOMP
         putchar('A');  // supports 'new' advanced compression
#endif
#ifdef DCRC
         putchar('C');  // doing CRC checks
#endif 
    }
    else if ((fd = open(argv[1],O_BINARY|O_RDONLY)) == EOF)
         printf("\ncould not open %s",argv[1]);
    else if((rd = read(fd,fhead,sizeof(struct file_head))) == 
             sizeof(struct file_head) && 
             strnicmp((char *)fhead,"TD",2) ==0 ) 
    { // always have a file header
         flen = lseek(fd,0L,SEEK_END); // length of file
         lseek(fd,(long)rd,SEEK_SET); // back to where we were
         // was flen = filelength(fd); // remove for linux
#ifdef DUMP
        if(ctl & DHEAD)
             dump(stdout,(BYTE *)fhead,sizeof(struct file_head),0); // fhead
#endif
        // 1st two chars determine type of compression
        if(strncmp((char *)fhead,"td",2) == 0)
        {
#ifdef  DCOMP
             docomp = 1; // set global flag to redirect read()

             init_Decode(fd); // initialize decompression routine
#else
             printf("\nthis file not supported, it uses advanced compression\n");
             exit(0);
#endif
        }

#ifdef DCRC
        crc = do_crc((BYTE *)fhead,0xa); // check 1st 10 bytes 
        if(fhead->crc != crc)
              printf("\nwarning crc error in file header");
#endif
        if((block = (unsigned char *) malloc(BLKSZ)) == NULL)
        {
             printf("\nfailed to allocate space for sector buffer");
             exit(1);
        }

        if(fhead->flag & COMMENT) // there is a comment block
        {                         
            rd = 1;
            // if chead exists, it immediately follows fhead
            if(read(fd,chead,sizeof(struct com_head)) !=
               sizeof(struct com_head))
                printf("\nfailed to read comment header");
            else if(BLKSZ < chead->len) // not likely!
                printf("\nallocated buffer too small for comment of length %d",
                       chead->len);
            else if(read(fd,block,chead->len) != chead->len)
            {
                printf("\nfailed to read comment");
            }
            else
                rd = 0;

            if(rd != 0)
            {
              if(block) free(block);
              exit(1);  // one of fatal errors above
            }

#ifdef DCRC
            // check all but 1st 2 bytes of comment region
            crc = do_crc((BYTE *)chead+2,8); // header
            updt_crc(&crc,block,chead->len); // comment data
            if(chead->crc != crc)
                 printf("\nwarning crc error in comment header");
#endif

#ifdef DUMP
        if(ctl & DHEAD)
             dump(stdout,(BYTE *)chead,sizeof(struct com_head),sizeof(struct file_head)); // chead
        // we just printed comment, don't bother to dump
#endif
            putchar('\n');
            // this includes old logic that coounted # of NULs 
            // convert each NUL to CR\LR
            for(i=0;i<chead->len;i++)
            {
                 if(block[i] == 0)
                 {
                     putchar('\n');
                     nnul++;tnul++;
                     if(tnul >= 8)
                           break;
                 }
                 else
                 {
                     putchar(block[i]);
                     nnul = 0;  // clear consecutive NUL count
                 }
            }
            printf("\ncreated %s %02d, %4d  %02d:%02d:%02d",
                 mons[chead->mon],chead->day,chead->yr+1900,
                 chead->hr,chead->min,chead->sec);
            printf("\nstring len 0x%2x  start variable 0x%2x = 0x%2x with %d NULs",
                chead->len,chead->len+0x16,i+1,tnul);
         }
         else
         {
              printf("\n no comment data included in file");
         }
         printf("\nfile length 0x%lx = %ld\n\n",flen,flen);
         suc = 0;

    }
    else if(rd >= 0x10)
         printf("\ninvalid id %2.2s", (char*) fhead);
    else
         printf("\nread error, header not long enough");
    adr = sizeof(struct file_head); // set track data to start after fhead
    if(fhead->flag & COMMENT)
         adr += sizeof(struct com_head) + chead->len;  // skip variable length comment
    bloc = adr; // skip block header, set global file offset
    
    if(suc == 0 && fd != EOF)
    {
             while(suc == 0 && (i=read(fd,&trk,sizeof(struct track_rec))) 
                               == sizeof(struct track_rec))
             {  // do tracks
                  memset(written,0,MAXSEC); // clear written array
#ifdef DUMP
                  if(ctl & DHEAD)
                  {
                      printf("\n%2d sectors for head %d physical track %2d (decimal)\n",
                           trk.nsec,trk.head,trk.trk);
                      dump(stdout,(BYTE *)&trk,i,adr);
                  }
#endif
                  if(trk.nsec == 0xff)
                  {
                        printf("\nNormal EOF on track %d #sectors in track record = 0xff",
                                trk.trk);
                        break; // end of file
                  }

#ifdef DCRC
                  // note last crc when trk.nsec == 0xff doesn't match
                  crc = do_crc((BYTE *)&trk,3); // check 1st 3 bytes 
                  if(trk.crc != (crc & 0xff))  // byte crc
                      printf("\nwarning crc error in track record 0x%x",
                                trk.trk);
#endif
                  rept_cnt = skip_cnt = 0; // assume in sync
                  adr += i; // read track record
                  for(i=1;suc == 0 && i<=trk.nsec;i++)
                  {
                     // allow early abort for testing
                     if((ctl & DCNT) && cnt-- <=0)
                        break;
                     skipped = 0; // assume signal no skip occured
                     if((suc = do_sector(fd,&trk,block,i+skip_cnt-rept_cnt,&ctl)) == 0 ||
                         suc == AKAI_SEC || (suc &  SKIP_MASK) || (suc & REPT_MASK) )
                     {
#ifdef DUMP
                         if(ctl & DBLOCK)
                         {
                            dump(stdout,block,BLKSZ,0);
                            putchar('\n');
                         }
#endif
                         blksz = block_size(ctl);

                         if(suc & SKIP_MASK)
                         {
                             skipped =  (suc & 0xff);
                             if(fo != EOF && write_skip(fo,skipped,blksz,trkcnt) < 0)
                             {
                                  suc++;
                                  break;
                             }

                             skip_cnt += skipped;
                             tot_skip += skipped;
                             suc = 0; // all is well
                         }
                         /* code above skips forward in file if skip a sector
                            but still need to write sector just filled in by
                            do_sector().  As of 10/03/02 somehow starting
                            at offset 0x18 block has been corrupted by the
                            warning message via printf() in do_sector()??
                         */

                         if(suc == AKAI_SEC)
                         {
                             suc = 0;
                             skip_cnt--; // ignore this sector for now
                                   // must decrement skip_cnt for do_sector() check
                         } // skip output below for AKAI_SEC
                         else if(suc & REPT_MASK)
                         {
                             csec = i+skip_cnt-rept_cnt; // current sector #
                             if(ctl & REPEAT)
                             {
                                 if(fo != EOF  && csec < MAXSEC &&
                                    written[csec] == 0 &&
                                    write_repeat(fo,block,blksz,csec,suc & 0xff))
                                 {
                                    printf("\nfatal error writting repeated sector");
                                    suc++;
                                    break;
                                 }
                                 if((csec = (suc & 0xff))  < MAXSEC && written[csec] == 0)
                                 {
                                     written[csec] = 1;
                                     seccnt++;
                                     ovrwrite++; // overwrote a skipped sector
                                     printf("Rewrite skipped sector %d\n",csec);
                                 }
                             }
                             rept_cnt++;
                             tot_rept++;
                             suc = 0; // clear suc so looping continues
                         }
                         else
                         {  // process this sector, prossibly writting to disk
                            seccnt ++; // # physical sectors, ignores phantom & skipped
#ifdef DISK              
               // conditionally allow direct write to a physical disk drive
                            if(ctl & DWRITE)
                            {
                               if(blksz != 512)
                               {
                                    printf("\nfatal error, direct write with block size %d not allowed",blksz);
                                    suc++;
                               }
                               else if((rd=aputsecs(drvb,(int)trk.head,(int)trk.trk,
                                               i+skip_cnt- rept_cnt,i+skip_cnt- rept_cnt,block)) != 0) 
                               {
                                    printf("\nerror writing sector to disk logical track %d sector %d",
                                            (int)trk.trk,i+skip_cnt- rept_cnt);
                                    suc++;
                               }
                            } 
#endif
                            if(fo != EOF && 
                                (rd = write(fo,block,blksz)) != blksz)
                            {
                                printf("\n data write error logical track %d sector %d",
                                     (int)trk.trk,i+skip_cnt- rept_cnt);
                                suc++;
                            }
                            else if((csec = i+skip_cnt-rept_cnt) < MAXSEC)
                                written[csec] = 1; // wrote it
                         } // special write skip for AKAI_SEC
                     }
                  }
                  if(suc == AKAI_END)
                        break; // don't do error check below on phantom track

                  /*
                   add error checks below because found at least one test
                   case where skipped last sector in track.  No way to
                   detect this with information I currently understand!
                   By doing both cases below can catch on 1st track
                   do_sector() will warn/catch if out of sequence
                  */
                  if(max_sec < trk.nsec + skip_cnt - rept_cnt)
                  {
                      max_sec = trk.nsec + skip_cnt - rept_cnt;
                      if(trkcnt > 0)
                          printf("\nnew max sector/track %d on logical track %d\n",
                               max_sec,trkcnt);
                      /* don't see how to fix on track 0 if it doesn't
                         write last sector, as don't know for sure till
                         have written track 2.  Could add cmd line option
                         but that's pretty clunky
                      */
                  }
                  else if(max_sec > trk.nsec + skip_cnt - rept_cnt)
                  {
                          printf("\nmax sector/track %d on logical track %d less the others",
                               trk.nsec + skip_cnt - rept_cnt,trkcnt);
                          i = max_sec -(trk.nsec + skip_cnt - rept_cnt);
                          tot_skip += i;
                          if(ctl & WARN)
                             printf("\nWarning skipping %d sectors at end of logical track %d\n",
                             i,trkcnt);
                          else
                             putchar('\n');

                          if(fo != EOF && write_skip(fo,i,blksz,trkcnt) < 0)
                               break; // fatal

                  }
                  trkcnt++; // increment my track count

             }
             if(suc == AKAI_END)
             {
                 printf("\nEarly termination, found AKAI END sector flag 0x65");
             }

             if((bloc = lseek(fd,0L,SEEK_CUR)) > 0)
                   printf("\nfinal file position 0x%lx",bloc);
             if(flen == bloc) printf("  => this is EOF");
             printf("\nparsed %d logical tracks, %d sectors with data ",
                    trkcnt,seccnt);
             printf("\n found %d skipped sectors and %d repeated",
                        tot_skip,tot_rept);
             if(ctl & REPEAT)
                printf(" (%d skipped overwritten)",ovrwrite);
             else printf(" (ignored)\n");
             printf("\nmax data sectors/track  %d",max_sec);
             if(seccnt != (i = trkcnt * max_sec))
                   printf("\n  WARNING implies total sectors should be %d",i); 
	     
    }
    putchar('\n'); // make linux output a little prettier
    if(block) free(block);
    
    return 0;
}
