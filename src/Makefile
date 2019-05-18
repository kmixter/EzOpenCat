all:
	make -f Makefile.host 
	make -f Makefile.arduino

clean:
	make -f Makefile.host clean
	make -f Makefile.arduino clean

install:
	make -f Makefile.host
	make -f Makefile.arduino install

run:
	make -f Makefile.host
	make -f Makefile.arduino run

serial:
	make -f Makefile.arduino serial
