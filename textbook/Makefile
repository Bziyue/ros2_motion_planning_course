.PHONY: all course outline

all: course

course:
	mkdir -p ../tmp/pdfs/course ../output/pdf
	latexmk -xelatex -interaction=nonstopmode -halt-on-error -outdir=../tmp/pdfs/course -jobname=course course.tex
	cp ../tmp/pdfs/course/course.pdf ../output/pdf/course.pdf

outline:
	mkdir -p ../tmp/pdfs/latex ../output/pdf
	latexmk -xelatex -interaction=nonstopmode -halt-on-error -outdir=../tmp/pdfs/latex -jobname=course_outline outline.tex
	cp ../tmp/pdfs/latex/course_outline.pdf ../output/pdf/course_outline.pdf
