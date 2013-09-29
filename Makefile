all:
	gcc -Wall -O2 -g -o s6prog s6prog.c -lusb -lftdi

clean:
	rm -rf p
