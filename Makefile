CXX = g++
CC = gcc
CFLAGS = -c -Wall -O2 -std=c++11 -g
EXEFLAG = -g -O2 -std=c++11
objdir = .obj/
exedir = bin/
objfile = $(objdir)CSR.o $(objdir)PathQueryHandler.o

TARGET = $(exedir)demo

$(exedir)demo: $(objdir)demo.o
	$(CXX) $(EXEFLAG) -o $(exedir)demo $(objdir)demo.o $(objfile) 

# 编译src下的其他文件，只需要复制下面两行，修改demo为对应的文件名即可
$(objdir)demo.o: src/demo.cpp $(objfile)
	$(CXX) $(CFLAGS) src/demo.cpp -o $(objdir)demo.o


$(objdir)CSR.o: include/gstore/CSR.cpp include/gstore/CSR.h
	$(CXX) $(CFLAGS) include/gstore/CSR.cpp -o $(objdir)CSR.o

$(objdir)PathQueryHandler.o: include/gstore/PathQueryHandler.cpp include/gstore/PathQueryHandler.h $(objdir)CSR.o
	$(CXX) $(CFLAGS) include/gstore/PathQueryHandler.cpp -o $(objdir)PathQueryHandler.o

all: $(TARGET)
	@echo "Compilation ends successfully!"

clean:
	rm -rf $(objdir)*.o
	rm -rf bin/*