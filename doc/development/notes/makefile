PDFS = \
    apps.pdf \
    bindings.pdf \
    c-api.pdf \
    coding-conventions.pdf \
    core-classes.pdf \
    docs.pdf \
    future.pdf \
    goals.pdf \
    intro.pdf \
    io.pdf \
    misc.pdf \
    pipeline.pdf \
    stages.pdf \
    testing.pdf \
    use-cases.pdf

%.pdf : %.txt
	rst2pdf $<

.PHONY: all clean

all: $(PDFS)

clean:
	rm -f $(PDFS)
