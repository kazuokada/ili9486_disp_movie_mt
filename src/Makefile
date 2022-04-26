ili9486_disp: ili9486_ctrl_mt.o fileio.o
	g++ ili9486_ctrl_mt.o fileio.o -o ili9486_disp -lwiringPi -lpthread

fileio.o: fileio.cpp fileio.h
	g++ -c fileio.cpp

ili9486_ctrl_mt.o: ili9486_ctrl_mt.cpp fileio.o
	g++ -c ili9486_ctrl_mt.cpp

