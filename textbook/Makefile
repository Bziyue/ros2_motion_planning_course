.PHONY: all course outline

PYTHON ?= /usr/bin/python3

all: course

course:
	mkdir -p ../tmp/pdfs/course ../output/pdf
	latexmk -xelatex -interaction=nonstopmode -halt-on-error -outdir=../tmp/pdfs/course -jobname=course course.tex
	cp ../tmp/pdfs/course/course.pdf ../output/pdf/course.pdf
	$(PYTHON) ../scripts/render_pdf_review.py ../output/pdf/course.pdf ../tmp/pdfs/review/course --latex-log ../tmp/pdfs/course/course.log

outline:
	mkdir -p ../tmp/pdfs/latex ../output/pdf
	latexmk -xelatex -interaction=nonstopmode -halt-on-error -outdir=../tmp/pdfs/latex -jobname=course_outline outline.tex
	cp ../tmp/pdfs/latex/course_outline.pdf ../output/pdf/course_outline.pdf
	$(PYTHON) ../scripts/render_pdf_review.py ../output/pdf/course_outline.pdf ../tmp/pdfs/review/outline --latex-log ../tmp/pdfs/latex/course_outline.log
