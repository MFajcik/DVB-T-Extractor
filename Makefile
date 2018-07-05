#init
#### info
#
# 'make'         builds executable file 'bms2' 
# 'make clean'   removes all .o and executable files
# @2016 Martin Fajcik FIT@VUT v Brne    xfajci00@stud.fit.vutbr.cz

COMPILER=g++
CXXFLAGS=-std=c++11 -O2 -pedantic

bms2: bms2obj
	$(COMPILER) bms2.o -o bms2
    
bms2obj: bms2.cpp
	$(COMPILER) $(CXXFLAGS) -c bms2.cpp -o bms2.o

clean:
	rm -f *.o *~ bms2