all:	libs
	cd prog; make demo;
	cd prog; make randwalk;
	cd prog; make fmstudio;

libs:
	cd basic; make;
	cd robots; make;
	cd gui; make;
	cd sbl; make;

clean:
	cd basic; make clean;
	cd robots; make clean;
	cd gui; make clean;
	cd sbl; make clean;
	cd prog; make clean;

cleanall:	clean
	rm -f lib/*.a
	cd prog; make cleanall;

depend:
	cd basic; make depend;
	cd robots; make depend;
	cd gui; make depend;
	cd sbl; make depend;
	cd prog; make depend;

html:
	mkdir doc_tmp;
	rm -f doc/classes/*;
	cp basic/*.H doc_tmp;
	cp basic/*.h doc_tmp;
	cp gui/*.H doc_tmp;
	cp sbl/*.H doc_tmp;
	cp robots/*.H doc_tmp;
	cd doc; doc++ -P -m -d classes -a -u ../doc_tmp/*
	rm -r doc_tmp;
