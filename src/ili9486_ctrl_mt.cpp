// rootじゃないと走らない
// sudo ./xxxx
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <time.h>
#include <wiringPi.h>
#define LOAD_PIC_NUM 2
#include "fileio.h"
//#define PRINT_RUNTIME
#include <pthread.h>
#define NUM_THREAD 2

#define BLOCK_SIZE (4 * 1024)
#define PI2_PERI_BASE 0x3f000000
#define PI1_PERI_BASE 0x20000000
#define PERI_BASE PI2_PERI_BASE // Raspberry 0/1 or 2/3
#define GPIO_BASE PERI_BASE + 0x00200000

#define LCD_REG_WAIT_TIME 50
#define LCD_HSIZE 320
#define LCD_VSIZE 480

#define PIC_NUM 2693

// BCM番号
#define LCD_RESX  26
#define LCD_CSX   19
#define LCD_DCX   13
#define LCD_WRX   6
#define LCD_RDX   5
/*
D7	GPIO1
D6	GPIO0
D5	GPIO12
D4	GPIO16
D3	GPIO20
D2	GPIO21
D1	GPIO25
D0	GPIO7
*/

#define LCD_SET_DATA(val)  (\
        ((val&0xc0)>>6) |\
        ((val&0x20)<<7) |\
        ((val&0x10)<<12) |\
        ((val&0x08)<<17) |\
        ((val&0x04)<<19) |\
        ((val&0x02)<<24) |\
        ((val&0x01)<<7) \
        )


// CSX=1にする
volatile void LCD_BUS_END( volatile unsigned int *gpset0)
{
    *gpset0 = (1<<LCD_CSX );
    //usleep(10);
    for(int i=0; i<LCD_REG_WAIT_TIME; i++);    // wait
}

volatile void LCD_COM_WRITE( volatile unsigned int *gpset0,
                    volatile unsigned int *gpclr0,
                    unsigned int val )
{
    // CSX=0, WRX=0, COMMAND=0x2c
    *gpclr0 = (1<<LCD_CSX ) |
              (1<<LCD_WRX ) |
              (1<<LCD_DCX ) |
              //( ( (~(val))&0xff )<<LCD_DO);
              LCD_SET_DATA( (~(val))&0xff );
    *gpset0 =  LCD_SET_DATA(val) ;
    // WRX=1
    *gpset0 = (1<<LCD_WRX );
    usleep(10);
    *gpset0 = (1<<LCD_DCX );
    usleep(10);
}

volatile void LCD_DATA_WRITE( volatile unsigned int *gpset0,
                    volatile unsigned int *gpclr0,
                    unsigned int val )
{
    // WRX=0
    *gpset0 = LCD_SET_DATA(val) ;
    *gpclr0 = (1<<LCD_WRX ) |
              LCD_SET_DATA( (~(val))&0xff );

    usleep(10);
    // WRX=1
    *gpset0 = (1<<LCD_WRX );
    usleep(10);
}

volatile void LCD_REG_SET( volatile unsigned int *gpset0,
                    volatile unsigned int *gpclr0,
                    unsigned char *table)
{
    unsigned int cmd;
    int tcnt;   // 転送回数
    int i;
    
    cmd = (unsigned int) *table;
    tcnt = (int)*(table+1);
    LCD_COM_WRITE(gpset0,gpclr0,cmd);
    if(tcnt!=0) {
        for(i=0; i<tcnt; i++) {
            LCD_DATA_WRITE(gpset0,gpclr0,*(table+2+i));
        }
    }
    LCD_BUS_END(gpset0);
    usleep(10);    
}                    

// Graphc lib
// 指定されたサイズ、色で矩形を塗りつぶす
void fill_box (unsigned char *dstp,
            volatile unsigned int *gpset0,
            volatile unsigned int *gpclr0,
            int starth, int startv, int hsize, int vsize,
            unsigned int col)
{
    int pt_y,pt_x;
    unsigned char *t_dstp;
    unsigned int temp_gpio;
    int i;
    t_dstp = dstp;
    LCD_COM_WRITE(gpset0,gpclr0,0x2c);  // memory write
    for(pt_y=startv; pt_y<startv+vsize; pt_y++) {
        for(pt_x=starth; pt_x<starth+hsize; pt_x++) {
            *t_dstp++ = col>>8;
            *t_dstp++ = col&0xff;
            
            // MSB 8bit write
            // WRX=0, pixdata MSB8bit送信
            temp_gpio = col>>8;
            //*gpset0 = (temp_gpio <<LCD_DO );
            *gpset0 = LCD_SET_DATA(temp_gpio) ;
            *gpclr0 = (1<<LCD_WRX ) |
                    //( ( (~(temp_gpio))&0xff )<<LCD_DO);
                    LCD_SET_DATA( (~(temp_gpio))&0xff );
            // WRX=1, pixdata MSB8bit送信
            *gpset0 = (1<<LCD_WRX );
            temp_gpio = col&0xff;
            *gpset0 = LCD_SET_DATA(temp_gpio) ;
            *gpclr0 = (1<<LCD_WRX ) |
                    LCD_SET_DATA( (~(temp_gpio))&0xff );
            // WRX=1, pixdata MSB8bit送信
            *gpset0 = (1<<LCD_WRX );
            for(i=0; i<LCD_REG_WAIT_TIME; i++);    // wait
        }
    }
    LCD_BUS_END(gpset0);
}

// 指定された矩形をLCDのグラフィックバッファへ転送
void DISP_rect (unsigned char *init_dstp,
            volatile unsigned int *gpset0,
            volatile unsigned int *gpclr0,
            int starth, int startv, int hsize, int vsize, int vram_hsize)
{
    int pt_y,pt_x;
    unsigned char *dstp;
    unsigned int temp_gpio;
    unsigned int reordered_gpio;
    int i;
    LCD_COM_WRITE(gpset0,gpclr0,0x2c);  // memory write
    for(pt_y=startv; pt_y<startv+vsize; pt_y++) {
        dstp = init_dstp + (pt_y*vram_hsize+starth)*2;
        for(pt_x=starth; pt_x<starth+hsize; pt_x++) {
            // MSB 8bit write
            // WRX=0, pixdata MSB8bit送信
            temp_gpio = *dstp;
            reordered_gpio=LCD_SET_DATA(temp_gpio);
            *gpset0 = reordered_gpio ;
            *gpclr0 = (1<<LCD_WRX ) |
                      (~(reordered_gpio))&0x02311083;

            // WRX=1, pixdata MSB8bit送信
            *gpset0 = (1<<LCD_WRX );
            dstp++;
            
            // LSB 8bit write
            // WRX=0
            temp_gpio = *dstp;
            reordered_gpio=LCD_SET_DATA(temp_gpio);
            *gpset0 = reordered_gpio ;
            *gpclr0 = (1<<LCD_WRX ) |
                      (~(reordered_gpio))&0x02311083;
            // WRX=1
            *gpset0 = (1<<LCD_WRX );
            dstp++;
        }
    }
    LCD_BUS_END(gpset0);
}
    
void chg_888_to_565(
    unsigned char *init_srcp, unsigned char *init_dstp,
    int starth, int startv, int hsize, int vsize, int vram_hsize)
{
    unsigned char *srcp;
    unsigned char *dstp;
    // RGB565作成
    int pt_y,pt_x;
    int r8,g8,b8;
    unsigned char gh3,gl3;

    srcp = init_srcp;
    //fprintf(stdout,"RGB565: srcp=%d\n",srcp);
    
    for(pt_y=startv; pt_y<(startv+vsize); pt_y++) {
        dstp = init_dstp + ((pt_y*vram_hsize+starth)*2);
        for(pt_x=starth; pt_x<starth+hsize; pt_x++) {
            b8 = (*srcp)&0xf8;      // val=MSB5 : 1111_1000b
            //b8 = b8 + (((*srcp)&0x04)<<1); // 四捨五入
            //if(b8>0xf8) b8=0xf8;
            srcp++;
            g8 = (*srcp)&0xfc;      // val=1111_1100b
            //g8 = g8 + (((*srcp)&0x02)<<1); // 四捨五入
            //if(g8>0xfc) g8=0xfc;
            srcp++;
            r8 = (*srcp)&0xf8;      // val=MSB5 : 1111_1000b
            //r8 = r8 + (((*srcp)&0x04)<<1); // 四捨五入
            //if(r8>0xf8) r8=0xf8;
            srcp++;
            gh3=((g8&0xe0)>>5);       // VAL=LSB3
            gl3=((g8&0x1c)>>2);       // val=LSB3
            b8 = (b8>>3);   // val=LSB5
            *dstp++ = r8 | gh3;
            *dstp++ = ((gl3)<<5) | b8;
        }
    }

}
int main()
{
    FILE *fp_table;
    FILE *fpi, *fpo;

    int i;
    char *map;

    volatile unsigned int *gpset0;
    volatile unsigned int *gpclr0;

    // 入力 bmpをワークメモリへ
    unsigned char *srcp, *init_srcp;		//入力BMPデータへのポインタ
    unsigned char *dstp, *init_dstp;		//出力画像データへのポインタ
    // source画像取り込みバッファは２面とする(for multi thread)
    unsigned char *srcp2, *init_srcp2;		//入力BMPデータへのポインタ

    int in_xsize,in_ysize;
    int frame_num;
    
    srcp = (unsigned char *)malloc(LCD_HSIZE*LCD_VSIZE * 3* LOAD_PIC_NUM);
    if (srcp == NULL) {
        fprintf(stderr, "Error:メモリの確保が出来ません。\n");
        exit(1);
    }
    init_srcp = srcp;	// 保持

    srcp2 = (unsigned char *)malloc(LCD_HSIZE*LCD_VSIZE * 3 * LOAD_PIC_NUM);
    if (srcp2 == NULL) {
        fprintf(stderr, "Error:メモリの確保が出来ません。\n");
        exit(1);
    }
    init_srcp2 = srcp2;	// 保持


    dstp = (unsigned char *)malloc(LCD_HSIZE*LCD_VSIZE * 2);    // RGB565
    if (dstp == NULL) {
        fprintf(stderr, "Error:メモリの確保が出来ません。\n");
        exit(1);
    }
    init_dstp = dstp;

// GPIOをレジスタ制御するために領域確保
    int g_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (g_fd < 0) {
        printf("Could not open /dev/mem fd\n");
        return -1;
    }

    unsigned int* gpio_mmap = ( unsigned int*) mmap(
        NULL,
        BLOCK_SIZE,
        PROT_READ|PROT_WRITE,
        MAP_SHARED,
        g_fd,
        GPIO_BASE
        );
    if (gpio_mmap == MAP_FAILED) {
	    printf("Could not gpio_mmap %d\n",gpio_mmap);
	    close(g_fd);
	    g_fd=0;
	    return -2;
    }
    gpset0 = (volatile unsigned int *) (gpio_mmap + 0x1c/4);    
    gpclr0 = (volatile unsigned int *) (gpio_mmap + 0x28/4);    

    if(wiringPiSetupGpio() == -1) return 1;

    // 各種信号初期化
    *gpset0 =  (1<<LCD_RESX)|
               (1<<LCD_CSX) |
               (1<<LCD_WRX) |
               (1<<LCD_RDX)
               ;
    *gpclr0 =  (1<<LCD_DCX);

    // 出力設定
    pinMode(1,  OUTPUT);    //D7
    pinMode(0,  OUTPUT);    //D6
    pinMode(12, OUTPUT);    //D5
    pinMode(16, OUTPUT);    //D4
    pinMode(20, OUTPUT);    //D3
    pinMode(21, OUTPUT);    //D2
    pinMode(25, OUTPUT);    //D1
    pinMode(7,  OUTPUT);    //D0
    pinMode(LCD_RESX,  OUTPUT);
    pinMode(LCD_CSX,   OUTPUT);
    pinMode(LCD_DCX,   OUTPUT);
    pinMode(LCD_WRX,   OUTPUT);
    pinMode(LCD_RDX,   OUTPUT);

    usleep(100);
    *gpclr0 =  (1<<LCD_RESX);   // RESETX=0
    usleep(100);
    *gpset0 =  (1<<LCD_RESX);   // RESETX=1
    usleep(100);


    unsigned char reg_tbl[256]; // LCDへの設定レジスタ
    int pt_reg_tbl;

    // load cat bmp file
    int MV;
    int vram_hsize, vram_vsize;
    char filename[]="cat_bmp/railgun_op_bmp.img";
    fpi = fopen(filename, "rb");  // 入力ファイルのオープン
    if (fpi == NULL) {
        fprintf(stderr, "%sがオープン出来ません\n", filename);
        exit(1);
    }
    // header 読み取り
    int reserved;
    fread(&in_xsize, 4, 1, fpi);
    fread(&in_ysize, 4, 1, fpi);
    fread(&frame_num, 4, 1, fpi);
    fread(&reserved, 4, 1, fpi);
    fprintf(stdout,"hsize=%d, vsize=%d, frame_num=%d\n",in_xsize,in_ysize,frame_num);
    //Load_bmp(srcp,filename,&in_xsize,&in_ysize);
    if(in_xsize>in_ysize) {
        MV=1;
        vram_hsize=LCD_VSIZE;
        vram_vsize=LCD_HSIZE;
    }
    else {
        MV=0;
        vram_hsize=LCD_HSIZE;
        vram_vsize=LCD_VSIZE;
    }
    
    LCD_COM_WRITE(gpset0,gpclr0,0x36);  // MAD CTL MY(b7)=1,
    if(MV==0)
        LCD_DATA_WRITE(gpset0,gpclr0,0x80|(MV<<5)|0x08); // MY=1,MX=0, MV=1, BGR 上下反転
    else
        LCD_DATA_WRITE(gpset0,gpclr0,0x80|0x40|(MV<<5)|0x08); // MY=1,MX=1, MV=1, BGR 上下反転
    LCD_BUS_END(gpset0);
    usleep(10);
    
    // command 
    LCD_COM_WRITE(gpset0,gpclr0,0x01);  // SoftReset
    LCD_BUS_END(gpset0);
    usleep(5000);   // 5ms
    LCD_COM_WRITE(gpset0,gpclr0,0x38);  // IDLEMODE OFF
    LCD_BUS_END(gpset0);
    usleep(10);

    LCD_COM_WRITE(gpset0,gpclr0,0x3a);  // Interface Pixel Format=16bit/pixel
    LCD_DATA_WRITE(gpset0,gpclr0,0x55);
    LCD_BUS_END(gpset0);
    
    // Column address
    //pt_reg_tbl=0;
    //reg_tbl[pt_reg_tbl++] = 0x2a;   // command
    //reg_tbl[pt_reg_tbl++] = 4;      // 回数
    //reg_tbl[pt_reg_tbl++] = 0x00;
    //reg_tbl[pt_reg_tbl++] = 0x00;
    //reg_tbl[pt_reg_tbl++] = (((320-1)&0xff00)>>8);
    //reg_tbl[pt_reg_tbl++] = (320-1)&0xff;
    //LCD_REG_SET(gpset0,gpclr0,reg_tbl);

    // Page address
    //pt_reg_tbl=0;
    //reg_tbl[pt_reg_tbl++] = 0x2b;   // command
    //reg_tbl[pt_reg_tbl++] = 4;      // 回数
    //reg_tbl[pt_reg_tbl++] = 0x00;
    //reg_tbl[pt_reg_tbl++] = 0x00;
    //reg_tbl[pt_reg_tbl++] = ((480-1)&0xff00)>>8;
    //reg_tbl[pt_reg_tbl++] = (480-1)&0xff;
    //LCD_REG_SET(gpset0,gpclr0,reg_tbl);

    // Power Control 2
    pt_reg_tbl=0;
    reg_tbl[pt_reg_tbl++] = 0xc1;   // command
    reg_tbl[pt_reg_tbl++] = 1;      // 回数
    reg_tbl[pt_reg_tbl++] = 0x41;
    LCD_REG_SET(gpset0,gpclr0,reg_tbl);

    // Power Control 3
    pt_reg_tbl=0;
    reg_tbl[pt_reg_tbl++] = 0xc2;   // command
    reg_tbl[pt_reg_tbl++] = 1;      // 回数
    reg_tbl[pt_reg_tbl++] = 0x44;
    LCD_REG_SET(gpset0,gpclr0,reg_tbl);

    //VCOM Control
    pt_reg_tbl=0;
    reg_tbl[pt_reg_tbl++] = 0xc5;   // command
    reg_tbl[pt_reg_tbl++] = 4;      // 回数
    reg_tbl[pt_reg_tbl++] = 0x00;
    //reg_tbl[pt_reg_tbl++] = 0x91;
    reg_tbl[pt_reg_tbl++] = 0x11;
    //reg_tbl[pt_reg_tbl++] = 0x80;
    reg_tbl[pt_reg_tbl++] = 0x80;
    reg_tbl[pt_reg_tbl++] = 0x00;
    LCD_REG_SET(gpset0,gpclr0,reg_tbl);
    
    //PGAMCTRL
    pt_reg_tbl=0;
    reg_tbl[pt_reg_tbl++] = 0xe0;   // command
    reg_tbl[pt_reg_tbl++] = 15;     // 回数
    reg_tbl[pt_reg_tbl++] = 0x0f;
    reg_tbl[pt_reg_tbl++] = 0x1f;
    reg_tbl[pt_reg_tbl++] = 0x1c;
    reg_tbl[pt_reg_tbl++] = 0x0c;
    reg_tbl[pt_reg_tbl++] = 0x0f;
    reg_tbl[pt_reg_tbl++] = 0x08;
    reg_tbl[pt_reg_tbl++] = 0x48;
    reg_tbl[pt_reg_tbl++] = 0x98;
    reg_tbl[pt_reg_tbl++] = 0x37;
    reg_tbl[pt_reg_tbl++] = 0x0a;
    reg_tbl[pt_reg_tbl++] = 0x13;
    reg_tbl[pt_reg_tbl++] = 0x04;
    reg_tbl[pt_reg_tbl++] = 0x11;
    reg_tbl[pt_reg_tbl++] = 0x0d;
    reg_tbl[pt_reg_tbl++] = 0x00;
    LCD_REG_SET(gpset0,gpclr0,reg_tbl);
    
    //NGAMCTRL
    pt_reg_tbl=0;
    reg_tbl[pt_reg_tbl++] = 0xe1;   // command
    reg_tbl[pt_reg_tbl++] = 15;     // 回数
    reg_tbl[pt_reg_tbl++] = 0x0f;
    reg_tbl[pt_reg_tbl++] = 0x32;
    reg_tbl[pt_reg_tbl++] = 0x2e;
    reg_tbl[pt_reg_tbl++] = 0x0b;
    reg_tbl[pt_reg_tbl++] = 0x0d;
    reg_tbl[pt_reg_tbl++] = 0x05;
    reg_tbl[pt_reg_tbl++] = 0x47;
    reg_tbl[pt_reg_tbl++] = 0x75;
    reg_tbl[pt_reg_tbl++] = 0x37;
    reg_tbl[pt_reg_tbl++] = 0x06;
    reg_tbl[pt_reg_tbl++] = 0x10;
    reg_tbl[pt_reg_tbl++] = 0x03;
    reg_tbl[pt_reg_tbl++] = 0x24;
    reg_tbl[pt_reg_tbl++] = 0x20;
    reg_tbl[pt_reg_tbl++] = 0x00;
    LCD_REG_SET(gpset0,gpclr0,reg_tbl);

    LCD_COM_WRITE(gpset0,gpclr0,0x11);  // Sleepout
    LCD_BUS_END(gpset0);
    usleep(5000);   // 5ms

    LCD_COM_WRITE(gpset0,gpclr0,0x29);  // DisplayON
    LCD_BUS_END(gpset0);
    usleep(10);

    dstp = init_dstp;
    // まずframe全体を黒で埋める
    fill_box(dstp, gpset0, gpclr0, 0,0,  LCD_HSIZE, LCD_VSIZE,0x0000);
    fill_box(dstp, gpset0, gpclr0, 0,0,  LCD_HSIZE, LCD_VSIZE,0x0000);

#ifdef PRINT_RUNTIME
    struct timespec tv_start_clock;
    struct timespec tv_end_clock;
    struct timespec tv_result;
    struct timespec tv_start_clock2;
    struct timespec tv_end_clock2;
    clock_gettime(CLOCK_REALTIME, &tv_start_clock);
#endif

    int load_frame_num;
    int wp=0;   // SDカードリード位置 +4づつ動く
    int rp=0;   // DISP表示位置 +1づつ
    int last_rp = (PIC_NUM/LOAD_PIC_NUM)*LOAD_PIC_NUM;
    int last_frame_num = PIC_NUM - last_rp;
    int image_no;

    //multi thread準備
    int src_buf_no=0;      // multi thread用 source画像面
    // スレッドを格納する配列
    pthread_t t[NUM_THREAD];
    // スレッドを実行する上で必要な情報を格納する配列
    struct Load_SD_data Load_SD_d[NUM_THREAD];

    // 1つ目のスレッドを作成 4枚分リード
    fseek(fpi, (16), SEEK_SET); // 画像のTOPへ
    srcp = init_srcp;
    Load_SD_d[src_buf_no&0x1].srcp = srcp;
    Load_SD_d[src_buf_no&0x1].in_xsize = &in_xsize;
    Load_SD_d[src_buf_no&0x1].in_ysize = &in_ysize;
    Load_SD_d[src_buf_no&0x1].fpi=fpi;
    Load_SD_d[src_buf_no&0x1].frame_num = &load_frame_num;
    pthread_create(&t[src_buf_no&0x1], NULL, thread_func_4pic,
        &Load_SD_d[src_buf_no&0x1]);

    for(image_no=0;  image_no<PIC_NUM; image_no++) {
    //for(image_no=0;  image_no<50; image_no++) {
        // multi threadではSDカードリードスレッドが終わるのを待つ
        // 終わったら次フレームのSDカードリードスレッドを生成
        // 前フレームのSDカードリードのスレッドの終了を待機
        // ただし4枚ごとに実行
        if((image_no&(LOAD_PIC_NUM-1))==0) { // &0x3
            pthread_join(t[src_buf_no&0x1], NULL);

            src_buf_no++;
            if(src_buf_no&0x1)  // src_buf_noは0始まり
                srcp = init_srcp2;  // 2回目以降
            else
                srcp = init_srcp;

            // load multi bmp
            wp+=LOAD_PIC_NUM;
            fseek(fpi, (16+ in_xsize * in_ysize * 3 * wp), SEEK_SET);
            Load_SD_d[src_buf_no&0x1].srcp = srcp;
            Load_SD_d[src_buf_no&0x1].in_xsize = &in_xsize;
            Load_SD_d[src_buf_no&0x1].in_ysize = &in_ysize;
            Load_SD_d[src_buf_no&0x1].fpi=fpi;
            // 最終読み込みかどうかの判断
            load_frame_num = (last_rp==rp) ? last_frame_num:LOAD_PIC_NUM;
            Load_SD_d[src_buf_no&0x1].frame_num = &load_frame_num;
            pthread_create(&t[src_buf_no&0x1], NULL, thread_func_4pic,
                &Load_SD_d[src_buf_no&0x1]);
        }
        dstp = init_dstp;
#ifdef PRINT_RUNTIME        
        clock_gettime(CLOCK_REALTIME, &tv_start_clock2);
#endif
        // RGB888 -> RGB565
        if(image_no&LOAD_PIC_NUM) // 4,5,6,7, 12,13,...
            chg_888_to_565(init_srcp2+(in_xsize*in_ysize*3*(image_no&(LOAD_PIC_NUM-1))),
                dstp, ((320-270)/2), 0, 270, 480, vram_hsize);
        else {          // 0,1,2,3, 8,9,...
            chg_888_to_565(init_srcp+(in_xsize*in_ysize*3*(image_no&(LOAD_PIC_NUM-1))),
                dstp, ((320-270)/2), 0, 270, 480, vram_hsize);
        }
        //chg_888_to_565(init_srcp+(in_xsize*in_ysize*3*(image_no&(LOAD_PIC_NUM-1))),
        //                dstp, ((320-270)/2), 0, 270, 480, vram_hsize);
        //chg_888_to_565(init_srcp, dstp, 0, 0, in_xsize, in_ysize, vram_hsize);
        // LCDへデータ送信
        DISP_rect(dstp,gpset0, gpclr0, 0, 0, vram_hsize, vram_vsize, vram_hsize);
        rp++;   // update disp pointer
#ifdef PRINT_RUNTIME
        clock_gettime(CLOCK_REALTIME, &tv_end_clock2);
        printf("frame_no:%d, %lf ms\n",
            image_no,
            sub_timespec_ms(&tv_result, &tv_end_clock2, &tv_start_clock2)
            );
#endif
    }

#ifdef PRINT_RUNTIME
    clock_gettime(CLOCK_REALTIME, &tv_end_clock);
    printf("total: %lf sec (frame count=%d), average frame rate=%05.2ffps\n",
        sub_timespec_ms(&tv_result, &tv_end_clock, &tv_start_clock)/1000,
        PIC_NUM,
        (PIC_NUM*1000)/sub_timespec_ms(&tv_result, &tv_end_clock, &tv_start_clock) );
#endif
    return 0;
}