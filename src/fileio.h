#pragma once
#include <stdio.h>
#include <pthread.h>

// スレッド実行に必要なデータを格納する構造体
struct Load_SD_data {
    unsigned char *srcp;
    //char *filename;
    int *in_xsize;
    int *in_ysize;
    FILE *fpi;
    int *frame_num;
};

int Get2Bytes(FILE *fp);
int Get4Bytes(FILE *fp);
int BMP_analysis(FILE *fpi, int *xsize, int *ysize);
void Load_bmp(unsigned char *srcp,char *filename, int *in_xsize, int *in_ysize);
void Load_SD_4pic(FILE *fpi, unsigned char *srcp, int *in_xsize, int *in_ysize, int *load_num);
double sub_timespec_ms(struct timespec* res, const struct timespec* a, const struct timespec* b);
void *thread_func_4pic(void *arg);
