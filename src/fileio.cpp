//#include "pch.h"
#include "fileio.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>


int Get2Bytes(FILE *fp)
{
	int     ret;

	ret = fgetc(fp);
	ret += fgetc(fp) * 0x100;
	return(ret);
}

int Get4Bytes(FILE *fp)
{
	int     ret;

	ret = fgetc(fp);
	ret += fgetc(fp) * 0x100;
	ret += fgetc(fp) * 0x10000;
	ret += fgetc(fp) * 0x1000000;
	return(ret);
}

/* 入力BMPファイルの解析 */
int BMP_analysis(FILE *fpi, int *xsize, int *ysize)
{
	int	OffBits;
	int hsize;
	int BitCount;

	/* Information area offset */
	fseek(fpi, 0x0a, SEEK_SET);           /* OffBitsへ */
	OffBits = Get4Bytes(fpi);

	hsize = Get4Bytes(fpi);
	if (hsize != 40) {
		fprintf(stderr, "BMP FILEではありません。\n");
		exit(1);
	}

	*xsize = Get4Bytes(fpi);
	*ysize = Get4Bytes(fpi);

	Get2Bytes(fpi);                     /*dummy Planes */

	BitCount = Get2Bytes(fpi);
	if (BitCount != 24) {
		fprintf(stderr, "24bit colorではありません。BitCount=%d\n", BitCount);
		exit(1);
	}

	return OffBits;

}


void Load_bmp(unsigned char *srcp,char *filename, int *in_xsize, int *in_ysize)
{
    FILE *fpi, *fpo;
    unsigned char *t_srcp;
    int i;

    t_srcp=srcp;
    fpi = fopen(filename, "rb");  // 入力ファイルのオープン
    if (fpi == NULL) {
        fprintf(stderr, "%sがオープン出来ません\n", filename);
        exit(1);
    }

    int OffBits;
    int aligned_hsize;
    OffBits = BMP_analysis(fpi, in_xsize, in_ysize);
    //fprintf(stderr,"info : input bmp  : offbits=%d, xsize=%d,ysize=%d\n",
    //    OffBits, *in_xsize, *in_ysize);
    // 4byteアラインされるので
    int amari;
    amari = (*in_xsize*3)%4;
    aligned_hsize = (amari!=0) ? (*in_xsize*3) + (4-amari) :  (*in_xsize*3);
    for (i = (*in_ysize) - 1; i >= 0; i--) {
        fseek(fpi, (OffBits + (aligned_hsize) * i), SEEK_SET);        /* DATAの最後から2line目へ */
        // endian を補正するため 1byte単位で読み込み
        fread(t_srcp, (*in_xsize)*3, 1, fpi);	// bmpファイルは上下逆
        t_srcp += (*in_xsize * 3);
    }
    fclose(fpi);

}

//#define PRINT_RUNTIME

void Load_SD_4pic(FILE *fpi, unsigned char *srcp, int *in_xsize, int *in_ysize, int *load_num)
{
    unsigned char *t_srcp;
    int i;

#ifdef PRINT_RUNTIME
    struct timespec start_clock;
    struct timespec end_clock;
    struct timespec tv_result;
    clock_gettime(CLOCK_REALTIME, &start_clock);
#endif

    t_srcp=srcp;
    fread(t_srcp, (*in_xsize)*(*in_ysize)*3 * (*load_num), 1, fpi);

#ifdef PRINT_RUNTIME
    clock_gettime(CLOCK_REALTIME, &end_clock);
    tv_result.tv_sec = end_clock.tv_sec - start_clock.tv_sec;
    tv_result.tv_nsec = end_clock.tv_nsec - start_clock.tv_nsec;
    if (tv_result.tv_nsec < 0) {
        tv_result.tv_sec--;
        tv_result.tv_nsec += (1 * 1000 * 1000 * 1000);
    }

    printf("SDリード時間(%d枚分)=%05.2f ms\n",*load_num,
        sub_timespec_ms(&tv_result, &end_clock, &start_clock)
         );
#endif

}

// timespec差分算出関数
double sub_timespec_ms(struct timespec* res, const struct timespec* a, const struct timespec* b)
{
    
    if (a->tv_nsec >= b->tv_nsec) {
        res->tv_nsec = a->tv_nsec - b->tv_nsec;
        res->tv_sec = a->tv_sec - b->tv_sec;
    } else {
        res->tv_nsec = 1000000000 + a->tv_nsec - b->tv_nsec;
        res->tv_sec = a->tv_sec - b->tv_sec - 1;
    }
    return res->tv_sec * 1000 +
        (double) res->tv_nsec / (1 * 1000 * 1000);
}

void *thread_func_4pic (void *arg)
{
    int i;
    struct Load_SD_data *pd = (struct Load_SD_data *)arg;

    /* argで指定された情報に基づいて処理 */
    Load_SD_4pic(pd->fpi, pd->srcp, pd->in_xsize, pd->in_ysize, pd->frame_num);
    

    return NULL;
}
