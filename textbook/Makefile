.PHONY: all

all:
	mkdir -p ../tmp/pdfs/latex ../output/pdf
	latexmk -xelatex -interaction=nonstopmode -halt-on-error -outdir=../tmp/pdfs/latex -jobname=course_outline outline.tex
	cp ../tmp/pdfs/latex/course_outline.pdf ../output/pdf/course_outline.pdf
