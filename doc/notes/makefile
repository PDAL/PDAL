PDFS = \
    apps.pdf \
    future.pdf \
    goals.pdf \
    intro.pdf \
    io.pdf \
    misc.pdf \
    pipeline.pdf \
    stages.pdf \
    use-cases.pdf

%.pdf : %.txt
	rst2pdf $<

.PHONY: all clean

all: $(PDFS)

clean:
	rm -f $(PDFS)
