
SRC	=	bananapi_m5.tex \
		bananapi_m2s.tex \
		bananapi_cm4.tex \
		bananapi_rpicm4.tex \
		bananapi_cm5io.tex \
		bananapi_cm5bpicm4io.tex \
		bananapi_m4berry.tex \
		bananapi_m4zero.tex \
		bananapi_f3.tex \
		bananapi_f5.tex \
		bananapi_ai2n.tex

ifneq ($V,1)
Q ?= @
endif


all:		${SRC}
	$Q echo	Generating DVI
	$Q latex  pins.tex

pins.dvi:	pins.tex
	$Q latex  pins.tex

pdf:	pins.dvi
	$Q dvipdf pins.dvi


.PHONY:	clean
clean:
	$Q rm -f *.dvi *.aux *.log *.ps *.toc *.bak *~
