# wteledsk
converts teledisk to raw images

### About
wteledsk was originally written by [Will Kranz](http://www.willsworks.net/) for reading Sydex Teledisk images from vintage computers. For more details see [here](http://www.willsworks.net/file-format/teledisk).

The image format was used as well to make images from non-standard floppy disks used in electronic music equipment, such as the AKAI S1000 sampler, or e.g. the General Music S2/3 Music Processor. This software has been used successfully with floppy images of the latter.

### Usage
```
usage: wteledsk <filename> [-n#] -o<outputfile> [-p] [-s] [-r]
       -n limit scan to first n sectors
       -o output restructured blocks to a file
       -p display phantom sec_rec.nsec values
       -s warn instead of fatal error on skipped sectors
       -r if repeated sector found, write it out again
 Version 1.01 A
```

The most useful invocation for converting GEM S2 image files seems to be `wteledsk INFILE.TD0 -oOUTFILE.IMG -s`.
